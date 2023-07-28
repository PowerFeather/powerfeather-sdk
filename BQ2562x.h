#pragma once

#include <MasterI2C.h>


namespace PowerFeather
{
    class BQ2562x
    {
    public:
        enum class OTGMode
        {
            None,
            Boost,
            Bypass
        };

        enum class VBUSStat
        {
            None,
            Adapter 
        };

        enum class ChargeStat
        {
            Terminated,
            Trickle,
            Taper,
            TopOff
        };

        BQ2562x(MasterI2C &i2c):_i2c(i2c) {}

        template <typename T>
        bool writeReg(T address, uint8_t start, uint8_t end, T value);
        template <typename T>
        bool writeReg(T address, uint8_t bit, bool value);
        template <typename T>
        bool readReg(T address, uint8_t start, uint8_t end, T& value);
        template <typename T>
        bool readReg(T address, uint8_t bit, bool& value);
        template <typename T>
        bool readReg(T address, T& value);
        
        bool setOTGMode(OTGMode mode);
        bool setOTGVoltage(float voltage);
        bool setChargeCurrent(uint16_t current);
        void enableCharging(bool state);
        bool enableSTAT(bool enable);
        bool enableTS(bool enable);
        bool enableWD(bool enable);
        uint8_t getFault();

        VBUSStat getVBUSStat();
        ChargeStat getChargeStat();

    private:
        static constexpr uint8_t _i2c_address = 0x6a;
        MasterI2C& _i2c;
    };
}