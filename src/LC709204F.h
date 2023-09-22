#pragma once

#include <MasterI2C.h>


namespace PowerFeather
{
    class LC70924F
    {
    public:
        LC70924F(MasterI2C& i2c):_i2c(i2c) {}
        bool writeCmd(uint8_t cmd, uint16_t value);
        bool readCmd(uint8_t cmd, uint16_t& value);
    private:
        static constexpr uint8_t _i2c_address = 0x6a;
        MasterI2C& _i2c;
    };
}