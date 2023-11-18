#include <driver/rtc_io.h>

#ifdef ARDUINO
#include <ArduinoMasterI2C.h>
#else
#include <MasterI2C.h>
#endif

#include <Mutex.h>
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

            // Pins that are not connected to anything on the board, and
            // can be freely configured/used by the user.
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

            // Pins that are connected to a component on the board and has a
            // pre-determined/pre-configured function.
            static constexpr gpio_num_t ALARM =     GPIO_NUM_21; // FuelGauge ALARM, Input
            static constexpr gpio_num_t INT =       GPIO_NUM_5; // Charger INT, Input

            static constexpr gpio_num_t LED =       GPIO_NUM_46; // MainBoard LED, Output
            static constexpr gpio_num_t BTN =       GPIO_NUM_0; // MainBoard BTN, Input
            static constexpr gpio_num_t EN =        GPIO_NUM_7; // MainBoard EN, Input

        private:
            static constexpr gpio_num_t USB_DP =    GPIO_NUM_20;
            static constexpr gpio_num_t USB_DM =    GPIO_NUM_19;

            static constexpr gpio_num_t EN_3V3 =    GPIO_NUM_4;
            static constexpr gpio_num_t EN_SQT =    GPIO_NUM_14;
            static constexpr gpio_num_t EN0 =       GPIO_NUM_13;

            static constexpr gpio_num_t SCL0 =      GPIO_NUM_48;
            static constexpr gpio_num_t SDA0 =      GPIO_NUM_47;

            static constexpr gpio_num_t PG =        GPIO_NUM_38;
        };


        /**
         * Initialize and set defaults.
         *
         * Max input current: 500 mA
         * Charging: disabled
         * 3V3: enabled
         * VSQT: enabled
         * Max charging current = 0.5C
         *
         * @param[in] capacity Battery capacity in mAh.
         */
        Result init(uint16_t capacity = 0);

        /**
         * Set EN pin state.
         *
         * @param[in] high EN pin is set high if true, set low if false.
         */
        Result setEN(bool high);

        /**
         * Enable or disable the header 3.3 V power output.
         *
         * @param[in] enable Enable if true, disable if false.
         */
        Result enable3V3(bool enable);

        /**
         * Enable or disable the STEMMA QT 3.3 V power output.
         *
         * @param[in] enable Enable if true, disable if false.
         */
        Result enableVSQT(bool enable);

        /**
         * Measure the supply voltage.
         *
         * @param[out] voltage Measured voltage in mV.
         */
        Result getSupplyVoltage(uint16_t& voltage);

        /**
         * Measure the supply current.
         *
         * @param[out] current Measured current in mA.
         */
        Result getSupplyCurrent(int16_t& current);

        /**
         * Check that the power supply, is a 'good' source as determined by the charger.
         *
         * @param[out] good If true, supply is good; if not battery is powering the board.
         * @return
         */
        Result getSupplyStatus(bool& good);

        /**
         * Enable or disable the supply.
         *
         * Disabling the supply means that the battery will power the board, even if a USB or DC
         * adapter is connected.
         *
         * @param[in] enable If true, supply is enabled; otherwise disabled.
         * @return
         */
        Result enableSupply(bool enable);

        /**
         * Sets the minimum supply voltage that should be maintained.
         *
         * This is usually the MPP (maximum power point) voltage of the power supply. The
         * voltage is maintained by automatically reducing current draw. This results to
         * the maximum power extracted from the supply.
         *
         * @param[in] voltage The maintained voltage in mV.
         */
        Result setSupplyMinVoltage(uint16_t voltage);

        /**
         * Set the maximum current draw from the power supply.
         *
         * This includes current draw from on-board components, the load on the VS pin,
         * and the charger current. The sum of all these current draws must not exceed this
         * current. Default is 1000 mA.
         *
         * @param[in] current The maximum current draw in mA.
         */
        Result setSupplyMaxCurrent(uint16_t current);

        /**
         * Set VBAT minimum output voltage.
         *
         * Set the minimum output voltage on VBAT even if there is no battery or the
         * battery is fully depleted. The default value is 3.7 V.
         *
         * @param voltage Minimum voltage to set VBAT output to, valid input range [2.2, 4.3]
         */
        Result setVBATMinVoltage(uint16_t mV);

        /**
         * Enter ship mode.
         *
         * Ship mode is a low power state that consumes about 1.5 uA. It is only possible to
         * exit this mode by pulling down QON or by plugging in supply. Only able to enter
         * ship mode if battery is powering the board.
         */
        Result enterShipMode();

        /**
         * Enter shutdown mode.
         *
         * Ship mode is a low power state that consumes about 1.4 uA. It is only possible
         * to exit this mode by plugging in supply. Only able to enter shutdown if battery
         * is powering board.
         */
        Result enterShutdownMode();

        /**
         * Power cycle board.
         *
         * This power cycles all components on-board, and all loads connected to power outputs.
         */
        Result doPowerCycle();

        /**
         * Enable battery charging.
         *
         * @param[in] enable Charging is enabled if true; otherwise disabled.
         */
        Result enableCharging(bool enable);

        /**
         * Enable temperature monitor.
         *
         * If enabled, the value of the 103AT thermistor connected on TS will be monitored.
         * Charging current will be reduced as temperature approaches 0 °C and 60 °C, and will
         * be disabled past them.
         *
         * @param[in] enable Battery temperature sensing is enabled if true; otherwise disabled.
         */
        Result enableTempSense(bool enable);

        /**
         * Enable the fuel gauge.
         *
         * Fuel gauge enabled consumes around 2 μA. Disabling the fuel gauge saves around 0.7 μA.
         *
         * @param[in] enable Fuel gauge is enabled if true; otherwise disabled.
         */
        Result enableFuelGauge(bool enable);

        /**
         * Set maximum charging current.
         *
         * Set value depending on preference between safety and speed. A charging
         * current of 1C is usually a good value, i.e. if battery has capacity of 520 mAh, set charging
         * current to 520 mA (520 * 1 = 520). Check datasheet for your battery for maximum charging current.
         *
         * @param[in] current Maximum charging current in mA.
         */
        Result setChargingMaxCurrent(uint16_t current);

        /**
         * Measure battery voltage.
         *
         * @param[out] voltage Battery voltage current in mV.
         */
        Result getBatteryVoltage(uint16_t& voltage);

        /**
         * Get an estimate of battery state-of-charge from 0 (empty) to 100 (full).
         *
         * @param[out] percent Battery charge percentage from 0 to 100.
         */
        Result getBatteryCharge(uint8_t& percent);

        /**
         * Get an estimate of battery state-of-health from 0 (dead) to 100 (healthy).
         *
         * @param[out] percent Battery health percentage from 0 to 100.
         */
        Result getBatteryHealth(uint8_t& percent);

        /**
         * Get the time left before fully empty/fully charged.
         *
         * Needs several minutes for the fuel gauge to get an estimate. In the meantime, returns
         * 65535 when charging or -65535 when discharging.
         *
         * @param[out] minutes Charge/discharge time left in minutes.
         */
        Result getBatteryTimeLeft(int& minutes);

        /**
         * Measure the battery temperature.
         *
         * Temperature sensing must be enabled via enableTempSense to get a reading.
         *
         * @param[out] celsius Battery current in °C.
         */
        Result getBatteryTemperature(float& celsius);

        /**
         * Get the number of charge/discharge cycles the battery has gone through.
         *
         * @param[out] cycles Number of battery cycles.
         */
        Result getBatteryCycles(uint16_t& cycles);

        /**
         * Measure the charge/discharge current to/from the battery.
         *
         * Returns a negative value when the battery is discharging, positive when charging.
         *
         * @param[out] current Battery current in mA.
         */
        Result getBatteryCurrent(int16_t& current);

        BQ2562x& getCharger() { return _charger; }
        LC709204F& getFuelGauge() { return _fuelGauge; }

        static MainBoard& get();

    private:
        MainBoard() {}

        static constexpr int _i2cPort = 1;
        static constexpr uint32_t _i2cFreq = 400000;
        static constexpr uint32_t _i2cTimeout = 1000;
        static constexpr uint32_t _defaultVSMaxCurrent = 3000;
        static constexpr uint32_t _defaultChargingMaxCurrent = 100;

#ifdef ARDUINO
        ArduinoMasterI2C _i2c {};
#else
        MasterI2C _i2c {};
#endif

        BQ2562x _charger {_i2c};
        LC709204F _fuelGauge {_i2c};

        bool _sqtOn;
        bool _initDone;
        Mutex _mutex;

        bool _initInternalDigitalPin(gpio_num_t pin, gpio_mode_t mode);
        bool _initInternalRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode);
        bool _setRTCPin(gpio_num_t pin, bool value);
        bool _isFirst();
        Result _initChargerAndFuelGauge(uint16_t mAh);
    };

    extern MainBoard& Board;
}
