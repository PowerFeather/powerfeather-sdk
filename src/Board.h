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
        public:
            friend Board;
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

            // Pins that are connected to a component on the board and has a
            // pre-determined/pre-configured function.
            class FF // Fixed-function
            {
            public:
                static constexpr gpio_num_t ALARM =     GPIO_NUM_7; // FuelGauge ALARM, Input
                static constexpr gpio_num_t INT =       GPIO_NUM_5; // Charger INT, Input

                static constexpr gpio_num_t LED =       GPIO_NUM_46; // Board LED, Output
                static constexpr gpio_num_t BTN =       GPIO_NUM_0; // Board BTN, Input
                static constexpr gpio_num_t EN =        GPIO_NUM_13; // Board EN, Input

                static constexpr gpio_num_t SCL =       GPIO_NUM_36; // Board SCL, shared with STEMMA QT, 5,1k pullup, Unconfigured
                static constexpr gpio_num_t SDA =       GPIO_NUM_35; // Board SDA, shared with STEMMA QT, 5.1k pullup, Unconfigured
            };

        private:
            class FFI // Fixed function internal
            {
            public:
                static constexpr gpio_num_t USB_DP =    GPIO_NUM_4;
                static constexpr gpio_num_t USB_DM =    GPIO_NUM_14;

                static constexpr gpio_num_t EN_3V3 =    GPIO_NUM_4;
                static constexpr gpio_num_t EN_SQT =    GPIO_NUM_14;
                static constexpr gpio_num_t EN =        GPIO_NUM_13;

                static constexpr gpio_num_t SCL0 =      GPIO_NUM_48;
                static constexpr gpio_num_t SDA0 =      GPIO_NUM_47;

                static constexpr gpio_num_t SRC =       GPIO_NUM_38; // Board SRC, Input
            };
        };

        bool init();

        /**
         * Enable or disable the header 3.3 V power output.
         *
         * @param enable Enable if true, disable if false.
         */
        void enable3V3(bool enable);

        /**
         * Enable or disable the STEMMA QT 3.3 V power output.
         *
         * @param enable Enable if true, disable if false.
         */
        void enableSQT(bool enable);

        /**
         * Enable or disable 5 V power output in battery-only mode.
         *
         * @param enable Enable if true, disable if false.
         */
        void enable5VOnBattery(bool enable);

        /**
         * Set 5 V power output voltage.
         *
         * Somewhat contrary to its name, the 5 V output is adjustable from 3.8 V - 5.2 V.
         * By default, it is set to 5.04V.
         *
         * @param voltage Voltage to set 5 V output to, valid input range [3.8, 5.2]
         */
        void set5V(float voltage);

        /**
         * Set VBAT minimum output voltage.
         *
         * Set the minimum output voltage on VBAT even if there is no battery or the
         * battery is fully depleted. The default value is 3.7 V.
         *
         * @param voltage Minimum voltage to set VBAT output to, valid input range [2.2, 4.3]
         */
        void setVBATMin(float voltage);

        /**
         * Set EN pin state.
         *
         * @param value EN pin is set high if true, set low if false
         */
        void setEN(bool value);

        /**
         * Get EN pin state.
         *
         * @return EN pin is high if true, low if false
         */
        bool getEN();

        // Get the power input currently supplying power to the board.

        /**
         * Determine the active power input, i.e. the input that is currently supplying power to the board.
         *
         * @return  The current active power input
         */
        PowerInput getPowerInput();

        /**
         * Enter ship mode.
         *
         * Ship mode is a low power state that consumes about 1.5 uA. It is only possible to
         * exit this mode by pulling down QON or by plugging in USB or DC.
         * 
         * Only able to enter shutdown in battery-only.
         */
        void enterShipMode();

        /**
         * Enter shutdown mode.
         *
         * Ship mode is a low power state that consumes about 1.5 uA. It is only possible
         * to exit this mode by plugging in USB or DC.
         * 
         * Only able to enter shutdown in battery-only.
         */
        void enterShutdownMode();


        /**
         * Power cycle board.
         *
         * Only able to enter shutdown in battery-only.
         */
        void doPowerCycle();


        /**
         * Enable or disable battery charging.
         */
        void enableCharging(bool enable);

        /**
         * Set maximum charging current.
         * 
         * Set value depending on preference between safety and speed. A charging 
         * current of 1C is usually a good value, i.e. if battery has capacity of 520 mAh, set charging
         * current to 520 mA (520 * 1 = 520). Check datasheet for your battery for maximum charging current.
         *
         */
        void setChargingMaxCurrent(uint32_t mA);

        /**
         * Set the safety timer, after which the charging will be terminated.
         *
         * @param ms Safety timer in minutes; set to 0 to disable safety timer (not recommended).
         */
        void setChargingSafetyTimer(uint32_t minutes);

        /**
         * Get current battery voltage measurement.
         */
        float getBatteryVoltage();

        /**
         * Get an estimate of battery state-of-charge from 0 to 1, i.e. getting .68 means
         * that charge is 68% of capacity.
         */
        float getBatteryCharge();

        /**
         * Get an estimate of battery state-of-health from 0 to 1, i.e. getting 0.68 means
         * the battery only has max capacity 68% of the original design capacity.
         */
        float getBatteryHealth();

        /** Returns remaining battery runtime when discharging;
         * returns time-to-full charge when charging.
         */
        uint32_t getBatteryTimeLeft();


        BQ2562x& getCharger() { return _charger; }
        LC709204F& getFuelGauge() { return _fuelGauge; }

    private:
        static constexpr int _i2cPort = 1;
        static constexpr uint32_t _i2cFreq = 400000;
        static constexpr uint32_t _i2cTimeout = 1000;

        MasterI2C _i2c {};
        BQ2562x _charger {_i2c};
        LC709204F _fuelGauge {_i2c};

        bool _initDigitalPin(gpio_num_t pin, gpio_mode_t mode);

        bool _initRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode);
        void _setRTCPin(gpio_num_t pin, bool value);
    };
}
