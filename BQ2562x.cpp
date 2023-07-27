#include <BQ2562x.h>



namespace PowerFeather
{
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
		return writeReg(0x16, 0, 1, static_cast<uint8_t>(enable ? 0x1 : 0x0));
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
}