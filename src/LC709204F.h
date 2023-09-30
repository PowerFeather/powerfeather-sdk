#pragma once

#include <MasterI2C.h>
namespace PowerFeather
{
    class LC709204F
    {
    public:
        LC709204F(MasterI2C& i2c):_i2c(i2c) {}

    private:
        static constexpr uint8_t _i2cAddress = 0x0b;
        MasterI2C& _i2c;

        bool readWord(uint8_t command, uint16_t *data);
        bool writeWord(uint8_t command, uint16_t data);
        uint8_t computeCRC8(uint8_t *data, int len);
        bool writeThenRead(const uint8_t *writeBuf, size_t writeLen, uint8_t *readBuf, size_t readLen);
    };
}