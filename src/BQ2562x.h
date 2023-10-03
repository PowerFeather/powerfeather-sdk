#pragma once

#include <MasterI2C.h>

#define BYTE(x)		static_cast<uint8_t>(x)
#define SHORT(x)	static_cast<uint16_t>(x)

namespace PowerFeather
{
    class BQ2562x
    {
    public:

        struct Register
        {
            uint8_t address;
            uint8_t size;
            uint8_t start;
            uint8_t end;
        };

        class Registers
        {
        public:
            static constexpr Register Charger_Control_0_EN_CHG =             { 0x16, 1, 5, 5 };
            static constexpr Register Charger_Control_0_WATCHDOG =           { 0x16, 1, 0, 1 };

            static constexpr Register NTC_Control_0_TS_IGNORE =              { 0x1a, 1, 7, 7 };

            static constexpr Register Charger_Status_1 =                     { 0x1e, 1, 0, 7 };
            static constexpr Register Charger_Status_1_VBUS_STAT =           { 0x1e, 1, 0, 2 };

            static constexpr Register Part_Information =                     { 0x38, 1, 0, 7 };
            static constexpr Register Part_Information_PN =                  { 0x38, 1, 3, 5 };
            static constexpr Register Part_Information_DEV_REV =             { 0x38, 1, 0, 2 };
        };

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

        enum class ADCRate
        {
            Oneshot,
            Continuous,
        };

        enum class ADCSampling
        {
            Bits_12,
            Bits_11,
            Bits_10,
            Bits_9,
            LastValue
        };

        enum class ADCAverage
        {
            Single,
            Running,
            LastValue
        };

        enum class ADCAverageInit
        {
            Existing,
            New,
            LastValue
        };

        enum class BATFETControl
        {
            Normal,
            ShutdownMode,
            ShipMode,
            SystemPowerReset
        };

        enum class BATFETDelay
        {
            Delay20ms,
            Delay10s,
        };

        BQ2562x(MasterI2C &i2c):_i2c(i2c) {}

        template <typename T>
        bool writeReg(Register reg, T value);
        template <typename T>
        bool readReg(Register reg, T& value);

        template <typename T>
        bool writeReg(T address, uint8_t start, uint8_t end, T value);
        template <typename T>
        bool writeReg(T address, uint8_t bit, bool value);
        template <typename T>
        bool writeReg(T address, T value);
        template <typename T>
        bool readReg(T address, uint8_t start, uint8_t end, T& value);
        template <typename T>
        bool readReg(T address, uint8_t bit, bool& value);
        template <typename T>
        bool readReg(T address, T& value);
        
        float getBatteryTemperature();
        bool setOTGMode(OTGMode mode);
        bool setOTGVoltage(float voltage);
        bool setChargeCurrent(uint16_t current);
        void enableCharging(bool state);
        bool enableSTAT(bool enable);
        bool enableTS(bool enable);
        bool enableWD(bool enable);
        float getBatteryVoltage();
        float getVBUS();
        float getIBAT();
        float getVBAT();
        float getIBUS();
        void enableADC(bool enable, ADCRate rate = ADCRate::Oneshot, ADCSampling sampling = ADCSampling::LastValue, 
                        ADCAverage average = ADCAverage::LastValue, ADCAverageInit averageInit = ADCAverageInit::LastValue);
        bool checkADC();
        bool getPartInformation(uint8_t& value);
        void setVINDPM(float voltage);
        void setBATFETControl(BATFETControl control);
        void setBATFETDelay(BATFETDelay delay);

        uint8_t getStat(int num);
        uint8_t getFlags(int num);
        uint8_t getFault();

        VBUSStat getVBUSStat();
        ChargeStat getChargeStat();

    private:
        static constexpr uint8_t _i2cAddress = 0x6a;
        MasterI2C& _i2c;
    };
}