/**
 *  POWERFEATHER 4-CLAUSE LICENSE
 *
 *  Copyright (C) 2023, PowerFeather.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *      list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *  3. Neither the name of PowerFeather nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without
 *      specific prior written permission.
 *
 *  4. This software, with or without modification, must only be run on official
 *      PowerFeather boards.
 *
 *  THIS SOFTWARE IS PROVIDED BY POWERFEATHER “AS IS” AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL POWERFEATHER OR CONTRIBUTORS BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include <driver/rtc_io.h>

#ifdef ARDUINO
#include "Utils/ArduinoMasterI2C.h"
#else
#include "Utils/MasterI2C.h"
#endif

#include "Utils/Result.h"
#include "Utils/Mutex.h"

#include "BQ2562x.h"
#include "LC709204F.h"

namespace PowerFeather
{
    class MainBoard
    {
    public:
        enum class BatteryType
        {
            Generic_3V7,
            ICR18650,
            UR18650ZY
        };

        class Pin
        {
        public:
            friend MainBoard;

            /**
             * Pin Definitions. There are three types:
             *      - Free IO
             *      - User-Managed Fixed
             *      - SDK-Managed Fixed
            */

            // Free IO
            static constexpr gpio_num_t A0 =        GPIO_NUM_10;
            static constexpr gpio_num_t A1 =        GPIO_NUM_9;
            static constexpr gpio_num_t A2 =        GPIO_NUM_8;
            static constexpr gpio_num_t A3 =        GPIO_NUM_3;
            static constexpr gpio_num_t A4 =        GPIO_NUM_2;
            static constexpr gpio_num_t A5 =        GPIO_NUM_1;

            static constexpr gpio_num_t D5 =        GPIO_NUM_15;
            static constexpr gpio_num_t D6 =        GPIO_NUM_16;
            static constexpr gpio_num_t D7 =        GPIO_NUM_37;
            static constexpr gpio_num_t D8 =        GPIO_NUM_6;
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

            // User-Managed Fixed
            static constexpr gpio_num_t ALARM =     GPIO_NUM_21; // FuelGauge ALARM
            static constexpr gpio_num_t INT =       GPIO_NUM_5;  // Charger INT

            static constexpr gpio_num_t LED =       GPIO_NUM_46; // User LED
            static constexpr gpio_num_t BTN =       GPIO_NUM_0;  // User button
            static constexpr gpio_num_t EN =        GPIO_NUM_7;  // FeatherWings enable/disable (read)

        private:
            // SDK-Managed Fixed
            static constexpr gpio_num_t USBDP =    GPIO_NUM_20;  // USB D+
            static constexpr gpio_num_t USBDM =    GPIO_NUM_19;  // USB D-

            static constexpr gpio_num_t EN_3V3 =    GPIO_NUM_4;  // 3V3 enable/disable
            static constexpr gpio_num_t EN_SQT =    GPIO_NUM_14; // VSQT enable/disable
            static constexpr gpio_num_t EN0 =       GPIO_NUM_13; // FeatherWings enable/disable (write)

            static constexpr gpio_num_t SCL0 =      GPIO_NUM_48; // STEMMA QT I2C SCL
            static constexpr gpio_num_t SDA0 =      GPIO_NUM_47; // STEMMA QT I2C SDA

            static constexpr gpio_num_t PG =        GPIO_NUM_38; // Charger Power Good
        };

        /**
         * Initialize board
         *
         * Initializes the power management and monitoring features. Sets the following defaults:
         *   - charging = disabled
         *   - 3V3 & VSQT = enabled
         *   - max charging current = 100 mA
         *   - temperature sense = disabled
         *   - supply maintain voltage = 4.6 V
         *   - battery alarms = disabled
         * If using batteries connected in parallel, specify the capacity for one cell.
         * Battery type is ignored if capacity = 0.
         *
         * @param[in] capacity Battery capacity in mAh
         * @param[in] type Battery type
         */
        Result init(uint16_t capacity = 0, BatteryType type = BatteryType::Generic_3V7);

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
        Result checkSupplyGood(bool& good);

        /**
         * Sets the minimum supply voltage that should be maintained.
         *
         * This is usually the MPP (maximum power point) voltage of the power supply. The
         * voltage is maintained by automatically reducing charging current. This results to
         * the maximum power extracted from the supply.
         *
         * @param[in] voltage The maintained voltage in mV.
         */
        Result setSupplyMaintainVoltage(uint16_t voltage);


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

        /**
         * Set alarm for when battery voltage is below a certain treshold.
         *
         * @param[in] voltage Battery voltage in mV, setting this to 0 disables the alarm.
         */
        Result setBatteryLowVoltageAlarm(uint16_t voltage);

        /**
         * Set alarm for when battery voltage is above a certain treshold.
         *
         * @param[in] voltage Battery voltage in mV, setting this to 0 disables the alarm.
         */
        Result setBatteryHighVoltageAlarm(uint16_t voltage);

        /**
         * Set alarm for when battery charge percent is below a certain treshold.
         *
         * @param[in] percent Battery charge percent from 1 to 100, setting this to 0 disables the alarm.
         */
        Result setBatteryLowChargeAlarm(uint8_t percent);

        BQ2562x& getCharger() { return _charger; }
        LC709204F& getFuelGauge() { return _fuelGauge; }

        static MainBoard& get();

    private:
        MainBoard() {}

        static constexpr int _i2cPort = 1;
        static constexpr uint32_t _i2cFreq = 100000; // TODO: use 400kHz, probably has something to do with clock stretching.
        static constexpr uint32_t _i2cTimeout = 1000;
        static constexpr uint32_t _defaultChargingMaxCurrent = 100;
        static constexpr uint32_t _minBatteryCapacity = 100;
        static constexpr uint32_t _maxBatteryCapacity = 15000;
        static constexpr uint32_t _chargerADCWaitTime = 100;
        static constexpr uint32_t _batfetCtrlWaitTime = 30;

#ifdef ARDUINO
        ArduinoMasterI2C _i2c {};
#else
        MasterI2C _i2c {};
#endif

        BQ2562x _charger {_i2c};
        LC709204F _fuelGauge {_i2c};

        bool _sqtOn { false };
        bool _fgOn { false };
        bool _initDone { false };
        uint32_t _chargerADCTime { 0 };
        uint16_t _batteryCapacity { 0 };
        BatteryType _batteryType { BatteryType::Generic_3V7 };
        Mutex _mutex { 100 };

        bool _isFirst();
        bool _setRTCPin(gpio_num_t pin, bool value);
        bool _initInternalDigitalPin(gpio_num_t pin, gpio_mode_t mode);
        bool _initInternalRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode);
        Result _initFuelGauge();
        Result _udpateChargerADC();
    };

    extern MainBoard& Board;
}
