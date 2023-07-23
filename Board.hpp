
#include <cstdbool>
#include <cstdint>

#include <driver/rtc_io.h>

namespace PowerFeather
{
    class Board
    {
    public:
        static constexpr gpio_num_t UserLedPin = static_cast<gpio_num_t>(6);
        static constexpr gpio_num_t UserButtonPin = static_cast<gpio_num_t>(0);

        enum class PowerInput
        {
            USB,
            DC,
            Battery,
        };

        // The board has two 3.3V rails which can be individually switched off/on.
        enum class PowerOutput
        {
            Header5V,
            Header3V3,
            Stemma3V3
        };

        enum class BatteryModeHeader5V
        {
            None, // Disabled when PowerInput = Battery
            Bypass, // Battery value when 
            Reg,
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

        bool setBatteryModeHeader5V(Board::BatteryModeHeader5V mode, float voltage = 5.0f);
        bool setVoltageHeader5V(float voltage);

        bool setChargeFactor(float factor);
        void enableHeader3V3(bool enable);
        void enableStemma3V3(bool enable);
        void enableHeader5V(bool enable);
        bool getEnablePin();
        void setEnablePin(bool value);
        void enableCharging(bool state);
        void enableGauge(bool enable);

    protected:
        static constexpr gpio_num_t EnablePin = static_cast<gpio_num_t>(13);
        static constexpr gpio_num_t ChargerIntPin = static_cast<gpio_num_t>(5);
        static constexpr gpio_num_t GaugeAlarmPin = static_cast<gpio_num_t>(21);

        static constexpr gpio_num_t EnableHeader3V3Pin = static_cast<gpio_num_t>(4);
        static constexpr gpio_num_t EnableStemma3V3Pin = static_cast<gpio_num_t>(14);
        static constexpr gpio_num_t GaugeRegPin = static_cast<gpio_num_t>(7);
        static constexpr gpio_num_t VDDTypePin = static_cast<gpio_num_t>(38);
        static constexpr gpio_num_t SCL0Pin = static_cast<gpio_num_t>(47);
        static constexpr gpio_num_t SDA0Pin = static_cast<gpio_num_t>(48);
    private:

        uint16_t _batteryCapacity;
        bool _useTSPin;
        uint8_t _i2cNum;

        bool _enableChargerStatLed(bool enable);
        bool _enableChargerTS(bool enable);
        uint8_t _getChargerFault();

        bool _readI2C(uint8_t address, uint8_t reg, uint8_t *data);
        bool _writeI2C(uint8_t address, uint8_t reg, uint8_t data);
        bool _setChargerRegister(uint8_t address, uint8_t bit, bool value);
        bool _initRTCPin(int pin, rtc_gpio_mode_t mode);
        bool _enableRTCPin(bool enable);
        void _setRTCPin(gpio_num_t pin, bool value);
    };
}
