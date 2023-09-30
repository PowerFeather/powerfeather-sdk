#include <LC709204F.h>

namespace PowerFeather
{
    bool LC709204F::readReg(uint8_t reg, uint16_t& value)
    {
        // return _i2c.read(_i2c_address, reg, value);
        return false;
    }

    uint16_t LC709204F::getChipID()
    {
        uint16_t data = 0;
        readReg(0x09, data);
        return data;
    }
}