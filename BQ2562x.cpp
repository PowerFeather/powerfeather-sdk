#include <BQ2562x.h>

namespace PowerFeather
{
	#define BYTE(x)		static_cast<uint8_t>(x)
	#define SHORT(x)	static_cast<uint16_t>(x)
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
	bool BQ2562x::writeReg(T reg, uint8_t start, uint8_t end, T value)
	{
		static_assert(sizeof(T) == 1 || sizeof(T) == 2);
		assert(end < sizeof(T) * CHAR_BIT);
		assert(start <= end);
		T data = 0;
		bool res = _i2c.read(_i2c_address, reg, data);
		if (res)
		{
			uint8_t bits = end - start + 1;
			T mask = ((0b1 << bits) - 1) << start;
			value <<= start;
			data = (data & ~mask) | (mask & value);
			res = _i2c.write(_i2c_address, reg, data);
		}
		return res;
	}

	template <typename T>
	bool BQ2562x::writeReg(T address, uint8_t bit, bool value)
	{
		return writeReg(address, bit, bit, static_cast<T>(value));
	}

	template <typename T>
	bool BQ2562x::writeReg(T address, T value)
	{
		return writeReg(address, 0, ((sizeof(value) * CHAR_BIT) - 1), value);
	}

	template <typename T>
	bool BQ2562x::readReg(T address, uint8_t start, uint8_t end, T &value)
	{
		static_assert(sizeof(T) == 1 || sizeof(T) == 2);
		assert(end < sizeof(T) * CHAR_BIT);
		assert(start <= end);
		T data = 0;
		bool res = _i2c.read(_i2c_address, address, data);
		if (res)
		{
			value = (data << (((sizeof(value) * CHAR_BIT) - 1) - end)) >> start;
		}
		return res;
	}

	template <typename T>
	bool BQ2562x::readReg(T address, uint8_t bit, bool &value)
	{
		T value2 = 0;
		bool res = readReg(address, bit, bit, value2);
		value = value2;
		return res;
	}

	template <typename T>
	bool BQ2562x::readReg(T address, T& value)
	{
		return readReg(static_cast<T>(address), 0, (sizeof(value) * CHAR_BIT) - 1, value);
	}

	bool BQ2562x::setChargeCurrent(uint16_t current)
	{
		current /= 40;
		if (current >= 0x1 && current <= 0x32)
		{
			return writeReg(SHORT(0x02), 5, 11, current);
		}
		return false;
	}

	bool BQ2562x::enableWD(bool enable)
	{
		return writeReg(BYTE(0x16), 0, 1, BYTE(enable));
	}

	bool BQ2562x::enableTS(bool enable)
	{
		bool inspect = 0;
		readReg(BYTE(0x1a), 7, inspect);
		printf("before: %d\n", inspect);
		bool res = writeReg(BYTE(0x1a), 7, !enable);
		readReg(BYTE(0x1a), 7, inspect);
		printf("after: %d\n", inspect);
		return res;
	}

	uint8_t BQ2562x::getFault()
	{
		uint8_t data = 0;
		readReg(BYTE(0x1f), data);
		return data;
	}

	uint8_t BQ2562x::getPartInformation()
	{
		uint8_t data = 0;
		readReg(BYTE(0x38), data);
		return data;
	}

	void BQ2562x::enableCharging(bool state)
	{
		writeReg(BYTE(0x16), 5, state);
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

    float BQ2562x::getVBUSVoltage()
    {
		uint16_t value = 0;
		readReg(SHORT(0x2c), value);
		printf("value: %d\n", value);
		return value / 1000.0f;
    }

	void BQ2562x::enableADC(bool enable, ADCRate rate, ADCSampling sampling, ADCAverage average, ADCAverageInit averageInit)
	{
		uint8_t value = enable << 7;

		if (enable)
		{
			switch (rate)
			{
			case ADCRate::Oneshot:
				/* code */
				break;

			case ADCRate::Continuous:
				/* code */
				break;
			
			default:
				break;
			}

			switch (sampling)
			{
			case ADCSampling::Bits_12:
				/* code */
				break;

			case ADCSampling::Bits_11:
				/* code */
				break;

			case ADCSampling::Bits_10:
				/* code */
				break;

			case ADCSampling::Bits_9:
				/* code */
				break;
			
			default:
				break;
			}

			switch (average)
			{
			case ADCAverage::Single:
				/* code */
				break;

			case ADCAverage::Running:
				/* code */
				break;
			
			default:
				break;
			}

			switch (averageInit)
			{
			case ADCAverageInit::Existing:
				/* code */
				break;

			case ADCAverageInit::New:
				/* code */
				break;
			
			default:
				break;
			}
		}

		writeReg(BYTE(0x26), value);
	}

    // bool BQ2562x::checkADC()
	// {
	// 	bool value;
	// 	readReg(BYTE(0x1d), 6, value);
	// 	return value;
	// }

    float BQ2562x::getBatteryVoltage()
	{
		uint16_t data = 0;
		readReg(SHORT(0x30), 1, 12, data);
		return (data) / 1000.0f;
	}

    BQ2562x::ChargeStat BQ2562x::getChargeStat()
    {
		uint8_t value = 0;
		readReg(BYTE(0x1e), 3, 4, value);

		ChargeStat res = ChargeStat::Terminated;

		switch (value)
		{
		case 0x01:
			res = ChargeStat::Trickle;
			break;

		case 0x02:
			res = ChargeStat::Taper;
			break;

		case 0x03:
			res = ChargeStat::TopOff;
			break;
		
		case 0x00:
		default:
			break;
		}

		return res;
    }

}