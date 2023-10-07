#pragma once

#include <MasterI2C.h>
namespace PowerFeather
{
    class LC709204F
    {
    public:

        enum class Registers
        {
            Cell_Voltage = 0x09,
            RSOC = 0x0d,
            State_Of_Health = 0x32,
            TimeToEmpty = 0x03,
            TimeToFull = 0x05,

            APA = 0x0b,
            Change_Of_The_Parameter = 0x12,
            Status_Bit = 0x16,
            IC_Power_Mode = 0x15,
        };

        enum class ChangeOfParameter
        {
            Nominal_3V7_Charging_4V2 = 0x00,
            UR18650ZY = 0x01,
            ICR18650_26H = 0x02,
            Nominal_3V8_Charging_4V35= 0x03,
            Nominal_3V85_Charging_4V4= 0x04,
        };

        LC709204F(MasterI2C& i2c):_i2c(i2c) {}

        uint16_t getCellVoltage();
        uint16_t getRSOC();
        uint16_t getSOH();
        uint16_t getTimeToEmpty();
        uint16_t getTimeToFull();

        bool setAPA(uint16_t capacity);
        bool setChangeOfParameter(ChangeOfParameter param);
        bool enableTSENSE(bool tsense1, bool tsense2);
        bool enableOperation(bool enable);

    private:
        static constexpr uint8_t _i2cAddress = 0x0b;

        MasterI2C& _i2c;

        bool readReg(Registers reg, uint16_t& data);
        bool writeReg(Registers reg, uint16_t data);
        uint8_t computeCRC8(uint8_t *data, int len);
    };
}