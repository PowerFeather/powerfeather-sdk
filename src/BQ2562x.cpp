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
	bool BQ2562x::writeReg(Register reg, T value)
	{
		uint8_t last = (reg.size * CHAR_BIT) - 1;

		assert(reg.size <= sizeof(value));
		assert(reg.start <= reg.end);
		assert(reg.end <= last);

		uint16_t data = 0;

		if (readReg(Register{reg.address, reg.size, 0, last}, data))
		{
			uint8_t bits = reg.end - reg.start + 1;
			uint16_t mask = ((0b1 << bits) - 1) << reg.start;
			data = (data & ~mask) | ((value << reg.start) & mask);
			return _i2c.write(_i2cAddress, reg.address, reinterpret_cast<uint8_t*>(&data), reg.size);
		}

		return false;
	}

	template <typename T>
	bool BQ2562x::readReg(Register reg, T& value)
	{
		assert(reg.size <= sizeof(value));
		assert(reg.start <= reg.end);
		assert(reg.end <= (reg.size * CHAR_BIT) - 1);

		uint16_t data = 0;
        if (_i2c.read(_i2cAddress, reg.address, reinterpret_cast<uint8_t*>(&data), reg.size))
		{
			int left = (((sizeof(data) * CHAR_BIT) - 1) - reg.end);
			data <<= left;
			data >>= left + reg.start;
			value = data;
			return true;
        }
		return false;
	}

	template <typename T>
	bool BQ2562x::writeReg(T address, uint8_t start, uint8_t end, T value)
	{
		// T data = 0;

		// if (readReg(address, data))
		// {
		// 	uint8_t bits = end - start + 1;
		// 	T mask = ((0b1 << bits) - 1) << start;
		// 	value <<= start;
		// 	data = (data & ~mask) | (mask & value);

		// 	return _i2c.write(_i2cAddress,  reinterpret_cast<uint8_t*>(&data), sizeof(data));
		// }

		return false;
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
	bool BQ2562x::readReg(T address, uint8_t start, uint8_t end, T& value)
	{
		value = 0;
		// static_assert(sizeof(value) == sizeof(uint8_t) || sizeof(value) == sizeof(uint16_t));
		// assert(end < (sizeof(value) * CHAR_BIT));
		// assert(start <= end);
		// assert((end - start) < (sizeof(T) * CHAR_BIT));

        // if (!_i2c.read(_i2cAddress, nt8_t*>(&address), 1, reinterpret_cast<uint8_t*>(&value), sizeof(value)))
		// {
        //     return false;
        // }

		// int left = (((sizeof(value) * CHAR_BIT) - 1) - end);
		// value <<= left;
		// value >>= left + start;
		return true;
	}

	template <typename T>
	bool BQ2562x::readReg(T address, uint8_t bit, bool &value)
	{
		T data = 0;
		bool res = readReg(address, bit, bit, data);
		value = data;
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
		return writeReg(Registers::Charger_Control_0_WATCHDOG, enable);
	}

	bool BQ2562x::enableTS(bool enable)
	{
		// uint16_t inspect = 0;
		// if (readReg(Registers::NTC_Control_0_TS_IGNORE, inspect))
		// {
		// 	printf("before: %d\n", inspect);
		// }
		bool res = writeReg(Registers::NTC_Control_0_TS_IGNORE, !enable);
		// if (res && readReg(Registers::NTC_Control_0_TS_IGNORE, inspect))
		// {
		// 	printf("after: %d\n", inspect);
		// }
		return res;
	}

	uint8_t BQ2562x::getStat(int num)
	{
		uint8_t data = 0;
		readReg(BYTE(0x1d + num), data);
		return data;
	}

	uint8_t BQ2562x::getFault()
	{
		uint8_t data = 0;
		readReg(BYTE(0x1f), data);
		return data;
	}

	uint8_t BQ2562x::getFlags(int num)
	{
		uint8_t data = 0;
		readReg(BYTE(0x20 + num), data);
		return data;
	}

	bool BQ2562x::getPartInformation(uint8_t& value)
	{
		return readReg(Registers::Part_Information, value);
	}

	void BQ2562x::enableCharging(bool state)
	{
		writeReg(Registers::Charger_Control_0_EN_CHG, state);
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

    float BQ2562x::getVBUS()
    {
		uint16_t value = 0;
		readReg(SHORT(0x2c), 2, 14, value);
		return (value * 3.97f) / 1000.0f;
    }

    float BQ2562x::getIBAT()
    {
		uint16_t value = 0;
		readReg(SHORT(0x2a), 2, 15, value);

		float partial = 0.0f;

		if (value >= 0x38AD && value <= 0x3FFF)
		{
			partial = (0x3FFF - value + 1) * -4.0f;
		}
		else
		{
			partial = value * 4.0f;
		}

		return partial / 1000.0f;
    }

    float BQ2562x::getIBUS()
    {
		uint16_t value = 0;
		readReg(SHORT(0x28), 1, 15, value);

		float partial = 0.0f;

		if (value >= 0x7830 && value <= 0x7FFF)
		{
			partial = (0x7FFF - value + 1) * -2.0f;
		}
		else
		{
			partial = value * 4.0f;
		}

		return partial / 1000.0f;
    }

    float BQ2562x::getVBAT()
    {
		uint16_t value = 0;
		readReg(SHORT(0x30), 1, 12, value);
		return (value * 1.99f) / 1000.0f;
    }

    void BQ2562x::setVINDPM(float voltage)
    {
		uint16_t value = 0;
		writeReg(SHORT(0x08), 5, 13, static_cast<uint16_t>((voltage * 1000.0) / 40));
		readReg(SHORT(0x08), 5, 13, value);
		printf("vindpm: %d\n", value * 40);
    }

	void BQ2562x::enableADC(bool enable, ADCRate rate, ADCSampling sampling, ADCAverage average, ADCAverageInit averageInit)
	{
		uint8_t value = enable << 7;

		if (enable)
		{
			switch (rate)
			{
			case ADCRate::Continuous:
				value &= ~(0b1 << 6);
				break;

			case ADCRate::Oneshot:
			default:
				value |= 0b1 << 6;
				break;
			}

			switch (sampling)
			{
			case ADCSampling::Bits_12:
				value &= ~(0b1 << 5);
				value &= ~(0b1 << 4);
				break;

			case ADCSampling::Bits_11:
				value &= ~(0b1 << 5);
				value |= 0b1 << 4;
				break;

			case ADCSampling::Bits_10:
				value |= 0b1 << 5;
				value &= ~(0b1 << 4);
				break;

			case ADCSampling::Bits_9:
				value |= 0b1 << 5;
				value |= 0b1 << 4;
				break;
			
			default:
				break;
			}

			switch (average)
			{
			case ADCAverage::Single:
				value &= ~(0b1 << 3);
				break;

			case ADCAverage::Running:
				value |= 0b1 << 3;
				break;
			
			default:
				break;
			}

			switch (averageInit)
			{
			case ADCAverageInit::Existing:
				value &= ~(0b1 << 2);
				break;

			case ADCAverageInit::New:
				value |= 0b1 << 2;
				break;
			
			default:
				break;
			}
		}

		writeReg(BYTE(0x26), value);
	}

    // bool BQ2562x::getADCDoneStat()
	// {
	// 	bool value;
	// 	readReg(BYTE(0x1d), 6, value);
	// 	return value;
	// }

    // bool BQ2562x::setADCDoneStat(bool value)
	// {
	// 	readReg(BYTE(0x1d), 6, value);
	// 	return value;
	// }

    float BQ2562x::getBatteryVoltage()
	{
		uint16_t data = 0;
		readReg(SHORT(0x30), 1, 12, data);
		return (data) / 1000.0f;
	}

	BQ2562x::VBUSStat BQ2562x::getVBUSStat()
	{
		uint8_t data = 0;
		readReg(BYTE(0x1e), data);
		if ((data & 0b111) == 0b100)
		{
			return VBUSStat::Adapter;
		}
		return VBUSStat::None;
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

    void BQ2562x::setBATFETControl(BATFETControl control)
    {
		uint8_t value = 0x0;
		switch (control)
		{
		case BATFETControl::ShutdownMode:
			value = 0x01;
			break;
		case BATFETControl::ShipMode:
			value = 0x02;
			break;
		case BATFETControl::SystemPowerReset:
			value = 0x03;
			break;
		case BATFETControl::Normal:
		default:
			break;
		}

		writeReg(BYTE(0x18), 0, 1, value);
	}

	void BQ2562x::setBATFETDelay(BATFETDelay delay)
	{
		bool value = false;
		switch (delay)
		{
		case BATFETDelay::Delay10s:
			value = true;
			break;
		case BATFETDelay::Delay20ms:
		default:
			break;
		}

		writeReg(BYTE(0x18), 2, value);
	}

	float BQ2562x::getBatteryTemperature()
	{
		uint16_t value;
		readReg(SHORT(0x34), 0, 11, value);
		return value * 0.0961;
	}
}