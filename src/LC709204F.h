#pragma once

#include <MasterI2C.h>
namespace PowerFeather
{
    class LC709204F
    {
    public:

        class Registers
        {
        public:
            static constexpr uint8_t Cell_Voltage = 0x09;
            static constexpr uint8_t RSOC = 0x0d;
            static constexpr uint8_t State_Of_Health = 0x32;
            static constexpr uint8_t TimeToEmpty = 0x03;
            static constexpr uint8_t TimeToFull = 0x05;
        };

        LC709204F(MasterI2C& i2c):_i2c(i2c) {}

        uint16_t getCellVoltage();
        uint16_t getRSOC();
        uint16_t getSOH();
        uint16_t getTimeToEmpty();
        uint16_t getTimeToFull();

    private:
        static constexpr uint8_t _i2cAddress = 0x0b;

        MasterI2C& _i2c;

        bool readReg(uint8_t reg, uint16_t& data);
        bool writeReg(uint8_t reg, uint16_t data);
        uint8_t computeCRC8(uint8_t *data, int len);
    };
}