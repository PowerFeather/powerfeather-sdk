#pragma once

#include <tuple>

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
            Cycle_Count = 0x17,
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

        bool getCellVoltage(uint16_t& mV);
        bool getRSOC(uint8_t& rsoc);
        bool getSOH(uint8_t& rsoh);
        bool getTimeToEmpty(uint16_t& minutes);
        bool getTimeToFull(uint16_t& minutes);
        bool getBatteryCycles(uint16_t& cycles);
        bool getOperation(bool& enabled);

        bool setAPA(uint16_t mAh);
        bool setChangeOfParameter(ChangeOfParameter param);
        bool enableTSENSE(bool tsense1, bool tsense2);
        bool enableOperation(bool enable);

    private:

        static constexpr std::tuple<uint16_t, uint8_t> _apaTable[] = {
            { 50, 0x13 },
            { 100, 0x15 },
            { 200, 0x18 },
            { 500, 0x21 },
            { 1000, 0x2D },
            { 2000, 0x3A },
            { 3000, 0x3F },
            { 4000, 0x42 },
            { 5000, 0x44 },
            { 6000, 0x45 },
        };

        static constexpr uint8_t _i2cAddress = 0x0b;

        MasterI2C& _i2c;

        bool readReg(Registers reg, uint16_t& data);
        bool writeReg(Registers reg, uint16_t data);
        uint8_t computeCRC8(uint8_t *data, int len);
    };
}