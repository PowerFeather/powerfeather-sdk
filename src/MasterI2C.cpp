#include <MasterI2C.h>

namespace PowerFeather
{
    bool MasterI2C::init(uint8_t port, uint8_t sdaPin, uint8_t sclPin, uint32_t freq)
    {
        _wire = port == 0 ? &Wire : &Wire1;
        return _wire->begin(sdaPin, sclPin, freq);
    }

    bool MasterI2C::write(uint8_t address, const uint8_t *buffer, size_t len)
    {
        _wire->beginTransmission(address);

        if (_wire->write(buffer, len) != len)
        {
            return false;
        }

        return (_wire->endTransmission(false) == 0);
    }

    bool MasterI2C::_read(uint8_t address, uint8_t *buffer, size_t len)
    {
        size_t recv = _wire->requestFrom(address, (uint8_t) len);

        if (recv != len)
        {
            return false;
        }

        for (uint16_t i = 0; i < len; i++)
        {
            buffer[i] = _wire->read();
        }

        return true;
    }

    bool MasterI2C::read(uint8_t address, uint8_t *buffer, size_t len)
    {
        size_t pos = 0;
        while (pos < len)
        {
            size_t readLen = (len - pos);
            if (!_read(address, buffer + pos, readLen))
            {
                return false;
            }
            pos += readLen;
        }
        return true;
    }


    bool MasterI2C::writeThenRead(uint8_t address, const uint8_t *writeBuf, size_t writeLen, uint8_t *readBuf, size_t readLen)
    {
        if (write(address, writeBuf, writeLen))
        {
            return read(address, readBuf, readLen);
        }

        return false;
    }
}
