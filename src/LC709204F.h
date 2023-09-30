#pragma once

#include <MasterI2C.h>
namespace PowerFeather
{
    class LC709204F
    {
    public:
        LC709204F(MasterI2C& i2c):_i2c(i2c) {}
        bool readReg(uint8_t reg, uint16_t& value);
        uint16_t getChipID();
    private:
        static constexpr uint8_t _i2c_address = 0x0b;
        MasterI2C& _i2c;
    };
}