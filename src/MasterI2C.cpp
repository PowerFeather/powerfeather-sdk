#include <MasterI2C.h>

namespace PowerFeather
{
    bool MasterI2C::init(uint8_t port, uint8_t sdaPin, uint8_t sclPin, uint32_t freq)
    {
        _wire = port == 0 ? &Wire : &Wire1;
        return _wire->begin(sdaPin, sclPin, freq);
    }

    /**
     * I2C Write.
     *
     * @param buffer
     * @param len
     * @return True on successful I2C operation
     */
    bool MasterI2C::write(uint8_t address, const uint8_t *buffer, size_t len) {
        _wire->beginTransmission(address);

        // Write the data itself
        if (_wire->write(buffer, len) != len) {
            return false;
        }

        if (_wire->endTransmission(false) == 0) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * I2C Read helper.
     *
     * @param buffer
     * @param len
     * @return True on successful I2C operation
     */
    bool MasterI2C::_read(uint8_t address, uint8_t *buffer, size_t len) {
        size_t recv = _wire->requestFrom(address, (uint8_t) len);

        if (recv != len) {
            // Not enough data available to fulfill our obligation!
            return false;
        }

        for (uint16_t i = 0; i < len; i++) {
            buffer[i] = _wire->read();
        }

        return true;
    }

    /**
     * I2C Read.
     * @param buffer
     * @param len
     * @return True on successful I2C operation
     */
    bool MasterI2C::read(uint8_t address, uint8_t *buffer, size_t len) {
        size_t pos = 0;
        while (pos < len) {
            size_t read_len = (len - pos);
            if (!_read(address, buffer + pos, read_len))
                return false;
            pos += read_len;
        }
        return true;
    }

}
