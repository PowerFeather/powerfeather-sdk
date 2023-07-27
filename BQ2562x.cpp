#include <BQ2562x.h>

namespace PowerFeather
{
	// Initialize the board
	// batteryCapacity - advertised capacity of the battery
	// chargeRate - charge rate of the battery expressed as fraction of the capacity, 
	//              ex. 2000mAh, 0.5 charge rate = 1000mA charge current
	//
	// event handler - when significant events occur
	//      - change in external power source
	//      - enable pin pulled down low
	//      - charging started/stopped
	//      - temperature too high or too low (hardware set)
	//      - battery state of charge low
	//      - input current exceeded
	//      - charge current exceeded
	//
	//  charger status
	//      - charging changed
	//      - vbus changed
	//  charger fault
	//      - bat overvoltage or overcurrent
	//      - sys overvoltage or short
	//      - otg overvoltage,
	//      - tshut thermal shutdown
	//
	// take OCV for gauge, then enable charging
	// enable 3V3 by default
	// enable 5V by default

	template <typename T>
	bool BQ2562x::writeReg(uint8_t reg, uint8_t start, uint8_t end, T value)
	{
		static_assert(sizeof(T) == 1 || sizeof(T) == 2);
		assert(end < sizeof(T) * CHAR_BIT);
		assert(start <= end);
		T data = 0;
		bool res = _i2c.read(_address, reg, data);
		if (res)
		{
			uint8_t bits = end - start + 1;
			T mask = ((0b1 << bits) - 1) << start;
			value <<= start;
			data = (data & ~mask) | (mask & value);
			res = _i2c.write(_address, reg, data);
		}
		return res;
	}

	bool BQ2562x::writeReg(uint8_t address, uint8_t bit, bool value)
	{
		return bit < CHAR_BIT ? writeReg(address, bit, bit, static_cast<uint8_t>(value))
								: writeReg(address, bit, bit, static_cast<uint16_t>(value));
	}

	template <typename T>
	bool BQ2562x::readReg(uint8_t address, uint8_t start, uint8_t end, T &value)
	{
		static_assert(sizeof(T) == 1 || sizeof(T) == 2);
		assert(end < sizeof(T) * CHAR_BIT);
		assert(start <= end);
		T data = 0;
		bool res = _i2c.read(address, address, data);
		if (res)
		{
			value = (data << (((sizeof(value) * CHAR_BIT) - 1) - end)) >> start;
		}
		return res;
	}

	bool BQ2562x::readReg(uint8_t address, uint8_t bit, bool &value)
	{
		uint8_t value1 = 0;
		uint16_t value2 = 0;
		bool res = bit < CHAR_BIT ? readReg(address, bit, bit, value1)
								: readReg(address, bit, bit, value2);
		value = value1 | value2;
		return res;
	}

	template <typename T>
	bool BQ2562x::readReg(uint8_t address, T& value)
	{
		return readReg(address, 0, (sizeof(value) * CHAR_BIT) - 1, value);
	}

	bool BQ2562x::setChargeCurrent(uint16_t current)
	{
		current /= 40;
		if (current >= 0x1 && current <= 0x32)
		{
			return writeReg(0x02, 5, 11, current);
		}
		return false;
	}

	bool BQ2562x::enableWD(bool enable)
	{
		return writeReg(0x16, 0, 1, static_cast<uint8_t>(enable));
	}

	bool BQ2562x::enableTS(bool enable)
	{
		return writeReg(0x1a, 7, !enable);
	}

	uint8_t BQ2562x::getFault()
	{
		uint8_t data;
		readReg(0x1f, data);
		return data;
	}

	void BQ2562x::enableCharging(bool state)
	{
		writeReg(0x16, 5, state);
	}

    bool BQ2562x::setOTGMode(BQ2562x::OTGMode mode)
    {
        // return _charger.writeReg(0x18, 6, enable);
		return false;
    }

    bool BQ2562x::setOTGVoltage(float voltage)
    {
        // return _charger.writeReg(0x18, 6, enable);
		return false;
    }
}