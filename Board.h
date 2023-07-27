#include <driver/rtc_io.h>

#include <BQ2562x.h>
#include <LC709204F.h>

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

        // Initialize the board
        // batteryCapacity - advertised capacity of the battery
        // chargeRate - charge rate of the battery expressed as fraction of the capacity, 
        //              ex. 2000mAh, 0.5 charge rate = 1000mA charge current
        //
        // event handler - when significant events occur
        //      - change in external power source
        //      - enable pin pulled down low
        //      - charging started/stopped
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

        void enableHeader3V3(bool enable);
        void enableStemma3V3(bool enable);
        bool enableHeader5VOnBattery(bool enable);
        bool getEnablePin();
        void setEnablePin(bool value);

        BQ2562x& getCharger() { return _charger; }
        LC70924F& getFuelGauge() {return _fuelGauge; }

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
        BQ2562x _charger {_masterI2C};
        LC70924F _fuelGauge {_masterI2C};

        uint16_t _batteryCapacity;
        bool _useTSPin;

        bool _initRTCPin(int pin, rtc_gpio_mode_t mode);
        bool _initDigitalPin(int pin, gpio_mode_t mode);
        void _setRTCPin(gpio_num_t pin, bool value);
    };
}
