
#include <cstdbool>
#include <cstdint>

#include <driver/rtc_io.h>
#include <driver/i2c.h>

namespace PowerFeather
{
    class Board
    {
    private:
        class MasterI2C
        {
        public:
            bool init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq);
            template <typename T>
            bool read(uint8_t address, uint8_t reg, T &data);
            template <typename T>
            bool write(uint8_t address, uint8_t reg, T data);
        private:
            uint32_t _port;
        };

        class Charger
        {
        public:
            Charger(MasterI2C &i2c):_i2c(i2c) {}
            template <typename T>
            bool writeReg(uint8_t address, uint8_t start, uint8_t end, T value);
            bool writeReg(uint8_t address, uint8_t bit, bool value);
            template <typename T>
            bool readReg(uint8_t address, uint8_t start, uint8_t end, T& value);
            bool readReg(uint8_t address, uint8_t bit, bool& value);
            template <typename T>
            bool readReg(uint8_t address, T& value);
        private:
            static constexpr uint8_t _address = 0x6a;
            MasterI2C& _i2c;
        };

        class FuelGauge
        {
        public:
            FuelGauge(MasterI2C& i2c):_i2c(i2c) {}
            bool writeCmd(uint8_t cmd, uint16_t value);
            bool readCmd(uint8_t cmd, uint16_t& value);
        private:
            static constexpr uint8_t _address = 0x6a;
            MasterI2C& _i2c;
        };

    public:
        static constexpr gpio_num_t UserLedPin = static_cast<gpio_num_t>(6);
        static constexpr gpio_num_t UserButtonPin = static_cast<gpio_num_t>(0);

        enum class PowerInput
        {
            USB,
            DC,
            Battery,
        };

        // Initialize the board
        // batteryCapacity - advertised capacity of the battery
        // chargeRate - charge rate of the battery expressed as fraction of the capacity, 
        //              ex. 2000mAh, 0.5 charge rate = 1000mA charge current
        //
        // event handler - when significant events occur
        //      - change in external power source
        //      - enable pin pulled down low
        //      - charging started/stopped
        //
        //
        //      - temperature too high or too low (hardware set)
        //      - battery state of charge low
        //      - input current exceeded
        //      - charge current exceeded
        //
        //  charger status
        //      - charging changed
        //      - vbus changed
        //  charger fault
        //      - bat overvoltage or overcurrent
        //      - sys overvoltage or short
        //      - otg overvoltage,
        //      - tshut thermal shutdown
        //
        // take OCV for gauge, then enable charging
        // enable 3V3 by default
        // enable 5V by default
        Board(uint16_t batteryCapacity = 0, bool useTSPin = false);

        bool init();

        bool setChargeFactor(float factor);
        void enableHeader3V3(bool enable);
        void enableStemma3V3(bool enable);
        bool enableHeader5VOnBattery(bool enable);
        bool getEnablePin();
        void setEnablePin(bool value);
        void enableCharging(bool state);
        void enableGauge(bool enable);

        Charger& getCharger() { return _charger; }
        FuelGauge& getFuelGauge() {return _fuelGauge; }

    protected:
        // Input
        static constexpr gpio_num_t ChargerIntPin = static_cast<gpio_num_t>(5);
        static constexpr gpio_num_t GaugeAlarmPin = static_cast<gpio_num_t>(21);
        static constexpr gpio_num_t GaugeRegPin = static_cast<gpio_num_t>(7);
        static constexpr gpio_num_t VDDTypePin = static_cast<gpio_num_t>(38);

        // Output
        static constexpr gpio_num_t EnableHeader3V3Pin = static_cast<gpio_num_t>(4);
        static constexpr gpio_num_t EnableStemma3V3Pin = static_cast<gpio_num_t>(14);

        // Input + Output
        static constexpr gpio_num_t EnablePin = static_cast<gpio_num_t>(13);

        static constexpr gpio_num_t SCL0Pin = static_cast<gpio_num_t>(47);
        static constexpr gpio_num_t SDA0Pin = static_cast<gpio_num_t>(48);

    private:
        MasterI2C _masterI2C {};
        Charger _charger {_masterI2C};
        FuelGauge _fuelGauge {_masterI2C};

        template <typename T>
        bool _readI2C(uint8_t address, uint8_t reg, T &data);
        template <typename T>
        bool _writeI2C(uint8_t address, uint8_t reg, T data);


        uint16_t _batteryCapacity;
        bool _useTSPin;

        template <typename T>
        bool _setRegisterValue(uint8_t address, uint8_t start, uint8_t end, T value);
        bool _setRegisterValue(uint8_t address, uint8_t bit, bool value);
        template <typename T>
        bool _setRegisterValue(uint8_t address, T value);

        template <typename T>
        bool _getRegisterValue(uint8_t address, uint8_t start, uint8_t end, T &value);
        bool _getRegisterValue(uint8_t address, uint8_t bit, bool &value);
        template <typename T>
        bool _getRegisterValue(uint8_t address, T &value);

        bool _enableChargerStatLed(bool enable);
        bool _enableChargerTS(bool enable);
        uint8_t _getChargerFault();
        bool _enableChargerWd(bool enable);
        bool _initRTCPin(int pin, rtc_gpio_mode_t mode);
        bool _initDigitalPin(int pin, gpio_mode_t mode);
        void _setRTCPin(gpio_num_t pin, bool value);
    };
}
