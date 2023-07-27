#pragma once

#include <MasterI2C.h>


namespace PowerFeather
{
    class BQ2562x
    {
    public:
        BQ2562x(MasterI2C &i2c):_i2c(i2c) {}

        template <typename T>
        bool writeReg(uint8_t address, uint8_t start, uint8_t end, T value);
        bool writeReg(uint8_t address, uint8_t bit, bool value);
        template <typename T>
        bool readReg(uint8_t address, uint8_t start, uint8_t end, T& value);
        bool readReg(uint8_t address, uint8_t bit, bool& value);
        template <typename T>
        bool readReg(uint8_t address, T& value);

        bool setChargeCurrent(uint16_t current);
        void enableCharging(bool state);
        bool enableSTAT(bool enable);
        bool enableTS(bool enable);
        uint8_t getFault();
        bool enableWD(bool enable);
    private:
        static constexpr uint8_t _address = 0x6a;
        MasterI2C& _i2c;
    };
}