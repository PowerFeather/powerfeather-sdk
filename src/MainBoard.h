#include <driver/rtc_io.h>

#include <MasterI2C.h>
#include <BQ2562x.h>
#include <LC709204F.h>

#include <Result.h>

namespace PowerFeather
{
    class MainBoard
    {
    public:
        class Pin
        {
        public:
            friend MainBoard;

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

                static constexpr gpio_num_t SCL =       GPIO_NUM_36;
                static constexpr gpio_num_t SDA =       GPIO_NUM_35;
            };

            // Pins that are connected to a component on the board and has a
            // pre-determined/pre-configured function.
            class FF // Fixed-function
            {
            public:
                static constexpr gpio_num_t ALARM =     GPIO_NUM_21; // FuelGauge ALARM, Input
                static constexpr gpio_num_t INT =       GPIO_NUM_5; // Charger INT, Input

                static constexpr gpio_num_t LED =       GPIO_NUM_46; // MainBoard LED, Output
                static constexpr gpio_num_t BTN =       GPIO_NUM_0; // MainBoard BTN, Input
                static constexpr gpio_num_t EN =        GPIO_NUM_7; // MainBoard EN, Input
            };

        private:
            class FFI // Fixed function internal
            {
            public:
                static constexpr gpio_num_t USB_DP =    GPIO_NUM_20;
                static constexpr gpio_num_t USB_DM =    GPIO_NUM_19;

                static constexpr gpio_num_t EN_3V3 =    GPIO_NUM_4;
                static constexpr gpio_num_t EN_SQT =    GPIO_NUM_14;
                static constexpr gpio_num_t EN =        GPIO_NUM_13;

                static constexpr gpio_num_t SCL0 =      GPIO_NUM_48;
                static constexpr gpio_num_t SDA0 =      GPIO_NUM_47;

                static constexpr gpio_num_t PG =        GPIO_NUM_38;
            };
        };


        /**
         * Initialize and set defaults.
         *
         * Max input current - 1000 mA.
         * Max charging current - 250 mA.
         * Charging - disabled.
         * 3V3 and SQT enabled.
         *
         * @param current The maximum current draw.
         */
        Result init(uint16_t mAh = 0);

        /**
         * Set EN pin state.
         *
         * @param value EN pin is set high if true, set low if false
         */
        Result setEN(bool value);

        /**
         * Get EN pin state.
         *
         * @return EN pin is high if true, low if false
         */
        bool getEN();

        /**
         * Enable or disable the header 3.3 V power output.
         *
         * @param enable Enable if true, disable if false.
         */
        Result enable3V3(bool enable);

        /**
         * Enable or disable the STEMMA QT 3.3 V power output.
         *
         * @param enable Enable if true, disable if false.
         */
        Result enableSQT(bool enable);

        /*
         */
        Result setSupplyMinVoltage(uint16_t mV);

        Result getSupplyVoltage(uint16_t& mV);

        Result getSupplyCurrent(int16_t& mA);

        /**
         * Set the maximum current draw from the power supply (USB/external DC adapter).
         *
         * This includes current draw from on-board components, the load on the VS pin,
         * and the charger current. The sum of all these current draws must not exceed this
         * current. Default is 1000 mA.
         *
         * @param current The maximum current draw.
         */
        Result setSupplyMaxCurrent(uint16_t mA);


        /**
         * Check that the power supply (USB/external DC adapter), is good
         * (as determined by the battery charger).
         *
         * If false, the system is being powered by the battery.
         *
         * @return
         */

        Result checkSupplyConnected(bool& connected);
        /**
         * Set VBAT minimum output voltage.
         *
         * Set the minimum output voltage on VBAT even if there is no battery or the
         * battery is fully depleted. The default value is 3.7 V.
         *
         * @param voltage Minimum voltage to set VBAT output to, valid input range [2.2, 4.3]
         */
        Result setVBATMinVoltage(uint16_t mV);

        // Get the power input currently supplying power to the board.

        /**
         * Enter ship mode.
         *
         * Ship mode is a low power state that consumes about 1.5 uA. It is only possible to
         * exit this mode by pulling down QON or by plugging in USB or DC.
         *
         * Only able to enter shutdown in battery-only.
         */
        Result enterShipMode();

        /**
         * Enter shutdown mode.
         *
         * Ship mode is a low power state that consumes about 1.5 uA. It is only possible
         * to exit this mode by plugging in USB or DC.
         *
         * Only able to enter shutdown in battery-only.
         */
        Result enterShutdownMode();

        /**
         * Power cycle board.
         *
         * Only able to enter shutdown in battery-only.
         */
        Result doPowerCycle();


        /**
         * Enable or disable battery charging.
         */
        Result enableCharging(bool enable);

        Result enableChargingTemperatureMonitor(bool enable);

        /**
         * Set maximum charging current.
         *
         * Set value depending on preference between safety and speed. A charging
         * current of 1C is usually a good value, i.e. if battery has capacity of 520 mAh, set charging
         * current to 520 mA (520 * 1 = 520). Check datasheet for your battery for maximum charging current.
         *
         */
        Result setChargingMaxCurrent(uint16_t mA);


        /**
         * Get current battery voltage measurement.
         */
        Result getBatteryVoltage(uint16_t& mV);

        /**
         * Get an estimate of battery state-of-charge from 0 to 1, i.e. getting .68 means
         * that charge is 68% of capacity.
         */
        Result getBatteryCharge(uint8_t& percent);

        /**
         * Get an estimate of battery state-of-health from 0 to 1, i.e. getting 0.68 means
         * the battery only has max capacity 68% of the original design capacity.
         */
        Result getBatteryHealth(uint8_t& percent);

        /** Returns remaining battery runtime when discharging;
         * returns time-to-full charge when charging.
         */
        Result getBatteryTimeLeft(int& minutes);

        Result getBatteryTemperature(float& celsius);

        Result getBatteryCurrent(int16_t& mA);

        BQ2562x& getCharger() { return _charger; }
        LC709204F& getFuelGauge() { return _fuelGauge; }

        static MainBoard& get();

    private:
        MainBoard() {}

        static constexpr int _i2cPort = 1;
        static constexpr uint32_t _i2cFreq = 400000;
        static constexpr uint32_t _i2cTimeout = 1000;
        static constexpr uint32_t _defaultChargingMaxCurrent = 100;
        static constexpr uint32_t _defaultVSMaxCurrent = 500;

        MasterI2C _i2c {};
        BQ2562x _charger {_i2c};
        LC709204F _fuelGauge {_i2c};

        bool _sqtOn;

        bool _initInternalDigitalPin(gpio_num_t pin, gpio_mode_t mode);
        bool _initInternalRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode);
        bool _setRTCPin(gpio_num_t pin, bool value);
        bool _isFirst();
        Result _initChargerAndFuelGauge(uint16_t mAh);
    };

    extern MainBoard& Board;
}
