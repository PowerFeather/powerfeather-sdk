#include "ArduinoMasterI2C.h"

#ifdef ARDUINO

namespace PowerFeather
{
    bool ArduinoMasterI2C::init(uint8_t port, uint8_t sdaPin, uint8_t sclPin, uint32_t freq)
    {
        _wire = port == 0 ? &Wire : &Wire1;
        return _wire->begin(sdaPin, sclPin, freq);
    }

    bool ArduinoMasterI2C::write(uint8_t address, uint8_t reg, const uint8_t *buffer, size_t len)
    {
        _wire->beginTransmission(address);
        if (_wire->write(reg))
        {
            if (_wire->write(buffer, len) != len)
            {
                return false;
            }
        }

        return (_wire->endTransmission(true) == 0);
    }

    bool ArduinoMasterI2C::read(uint8_t address, uint8_t *buf, size_t len)
    {
        size_t recv = _wire->requestFrom(address, (uint8_t) len);

        if (recv != len)
        {
            return false;
        }

        for (uint16_t i = 0; i < len; i++)
        {
            buf[i] = _wire->read();
        }

        return true;
    }

    bool ArduinoMasterI2C::read(uint8_t address, uint8_t reg, uint8_t *buf, size_t len)
    {
        _wire->beginTransmission(address);
        if (_wire->write(reg))
        {
            if (_wire->endTransmission(false) == 0)
            {
                return read(address, buf, len);
            }
        }

        return false;
    }
}

#endif