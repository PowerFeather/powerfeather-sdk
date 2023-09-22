#include <driver/rtc_io.h>

#include <MasterI2C.h>
#include <BQ2562x.h>
#include <LC709204F.h>

namespace PowerFeather
{
    class Board
    {
    public:
        enum class PowerInput
        {
            USB,
            DC,
            Battery,
        };

        class Pin
        {
            class GP // General-Purpose
            {
            // Pins that are not connected to anything on the board, and
            // can be freely configured/used by the user.
            public:
                static constexpr gpio_num_t A0 =        GPIO_NUM_10;
                static constexpr gpio_num_t A1 =        GPIO_NUM_9;
                static constexpr gpio_num_t A2 =        GPIO_NUM_8;
                static constexpr gpio_num_t A3 =        GPIO_NUM_3;
                static constexpr gpio_num_t A4 =        GPIO_NUM_2;
                static constexpr gpio_num_t A5 =        GPIO_NUM_1;

                static constexpr gpio_num_t D5 =        GPIO_NUM_15;
                static constexpr gpio_num_t D6 =        GPIO_NUM_16;
                static constexpr gpio_num_t D9 =        GPIO_NUM_17;
                static constexpr gpio_num_t D10 =       GPIO_NUM_18;
                static constexpr gpio_num_t D11 =       GPIO_NUM_45;
                static constexpr gpio_num_t D12 =       GPIO_NUM_12;
                static constexpr gpio_num_t D13 =       GPIO_NUM_11;

                static constexpr gpio_num_t TX =        GPIO_NUM_44;
                static constexpr gpio_num_t RX =        GPIO_NUM_42;
                static constexpr gpio_num_t MISO =      GPIO_NUM_41;
                static constexpr gpio_num_t MOSI =      GPIO_NUM_40;
                static constexpr gpio_num_t SCK =       GPIO_NUM_39;
                static constexpr gpio_num_t TX0 =       GPIO_NUM_43;
            };

            class FF // Fixed-function
            {
            // Pins that are connected to a component on the board and has a
            // pre-determined/pre-configured function.
            public:
                static constexpr gpio_num_t ALARM =     GPIO_NUM_7; // FuelGauge ALARM, Input
                static constexpr gpio_num_t INT =       GPIO_NUM_5; // Charger INT, Input

                static constexpr gpio_num_t LED =       GPIO_NUM_46; // Board LED, Output
                static constexpr gpio_num_t BTN =       GPIO_NUM_0; // Board BTN, Input
                static constexpr gpio_num_t EN =        GPIO_NUM_13; // Board EN, Input

                static constexpr gpio_num_t SCL =       GPIO_NUM_36; // Board SCL, shared with STEMMA QT, 5,1k pullup, Unconfigured
                static constexpr gpio_num_t SDA =       GPIO_NUM_45; // Board SDA, shared with STEMMA QT, 5.1k pullup, Unconfigured
            };

        private:
            class FFI // Fixed function internal
            {
                class InternalFixedFunction
                {
                public:
                    static constexpr gpio_num_t USB_DP =    GPIO_NUM_4;
                    static constexpr gpio_num_t USB_DM =    GPIO_NUM_14;

                    static constexpr gpio_num_t EN_3V3 =    GPIO_NUM_4;
                    static constexpr gpio_num_t EN_SQT =    GPIO_NUM_14;
                    static constexpr gpio_num_t EN =        GPIO_NUM_13;

                    static constexpr gpio_num_t SCL0 =      GPIO_NUM_47;
                    static constexpr gpio_num_t SDA0 =      GPIO_NUM_48;

                    static constexpr gpio_num_t SRC =       GPIO_NUM_38; // Board VDD_TYPE, Input
                };
            }
        }

        bool init();

        void enable3V3(bool enable);
        void enableSQT(bool enable);
        void enable5VOnBattery(bool enable);
        void set5V(float voltage);
        void setVBATMin(float voltage);

        void setEN(bool value);
        void getEN(bool value);

        // Enable the battery temperature sense pin.
        void enableTS(bool enable);
        // Get the power input currently supplying power to the board.
        PowerInput getPowerInput();

        void enterShipMode();
        void enterShutdownMode();

        void enableCharging(bool enable);
        void setChargingMaxCurrent(float current);
        void setChargingTimer(uint32_t ms);

        // Get current battery voltage measurement.
        float getBatteryVoltage();

        // Get an estimate of battery state-of-charge from 0 to 1, i.e. getting .68 means
        // that charge is 68% of capacity.
        float getBatteryCharge();

        // Get an estimate of battery state-of-health from 0 to 1, i.e. getting 0.68 means
        // the battery only has max capacity 68% of the original design capacity.
        float getBatteryHealth();

        // Returns remaining battery runtime when discharging;
        // returns time-to-full charge when charging.
        uint32_t getBatteryTimeLeft();

        // Get the battery temperature, if TS is enabled.
        float getBatteryTemp();

        BQ2562x& getCharger() { return _charger; }
        LC70924F& getFuelGauge() { return _fuelGauge; }

    private:
        static constexpr int _i2c_port = 1;
        static constexpr uint32_t _i2c_freq = 400000;
        static constexpr uint32_t _i2c_timeout = 1000;

        MasterI2C _masterI2C {};
        BQ2562x _charger {_masterI2C};
        LC70924F _fuelGauge {_masterI2C};

        bool _initDigitalPin(gpio_num_t pin, gpio_mode_t mode);

        bool _initRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode);
        void _setRTCPin(gpio_num_t pin, bool value);
    };
}
