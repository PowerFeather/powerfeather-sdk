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

    class Mainboard
    {
    public:
        enum class BatteryType
        {
            Generic_3V7, // Generic Li-ion/LiPo battery with nominal voltage of 3.7 V
            ICR18650, // Samsung IC18650
            UR18650ZY // Sanyo UR18650ZY
        };

        class Pin
        {
        public:
            friend Mainboard;

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

            static constexpr gpio_num_t SCL1 =      GPIO_NUM_48; // STEMMA QT I2C SCL
            static constexpr gpio_num_t SDA1 =      GPIO_NUM_47; // STEMMA QT I2C SDA

            static constexpr gpio_num_t PG =        GPIO_NUM_38; // Charger Power Good
        };

        /**
         * @brief Initialize the board power management and monitoring features.
         *
         * Initializes the charger, fuel gauge and other hardware related to power management and monitoring.
         *
         * Sets the following defaults:
         *  - \a EN: high
         *  - \a 3V3: enabled
         *  - \a VSQT: enabled
         *  - Charging: disabled
         *  - Maximum battery charging current: 100 mA
         *  - Maintain supply voltage: 4600 mV
         *  - Fuel gauge: enabled, if \p capacity is not zero
         *  - Battery temperature sense: disabled
         *  - Battery alarms (low charge, high/low voltage): disabled
         *
         * This function should be called once, before all calls to other \c Mainboard functions.
         *
         * @param[in] capacity The capacity of the connected Li-ion/LiPo battery in milliamp-hours (mAh), from 50 mAh to 6000 mAh.
         * A zero indicates that no battery is connected, and therefore some of the other \c Mainboard functions
         * will return \c Result::InvalidState. If using multiple batteries connected in parallel, specify
         * only the capacity for one cell. Non-zero value is ignored when
         * \p type is \c BatteryType::ICR18650 or \c BatteryType::UR18650ZY.
         * @param[in] type Type of Li-ion/LiPo battery; ignored when \p capacity is zero.
         * @return Result Returns \c Result::Ok if initialization succeeded; returns a value other than \c Result::Ok if not.
         */
        Result init(uint16_t capacity = 0, BatteryType type = BatteryType::Generic_3V7);

        /**
         * @brief Set \a EN pin high or low.
         *
         * This is useful for enabling or disabling connected Feather Wings to reduce power consumption.
         *
         * @param[in] high If \c true, EN is set high; if \c false, EN is set low.
         * @return Result Returns \c Result::Ok if \a EN was set high or low successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setEN(bool high);

        /**
         * @brief Enable or disable \a 3V3.
         *
         * Enables or disables \a 3V3, which is the 3.3 V header pin power output. When disabled, current flow to the
         * connected loads on \a 3V3 is cut-off, reducing power consumption.
         *
         * @param[in] enable If \c true, \a 3V3 is enabled; if \c false, \a 3V3 is disabled.
         * @return Result Returns \c Result::Ok if \a 3V3 was enabled or disabled successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result enable3V3(bool enable);

        /**
         * @brief Enable or disable \a VSQT.
         *
         * Enables or disables \a VSQT, which is the 3.3 V STEMMA QT power output. When disabled, current flow to the
         * connected STEMMA QT modules is cut-off, reducing power consumption.
         *
         * A side effect of disabling \a VSQT is that communications to the charger and fuel gauge is also disabled.
         * This means that some of the other \c Mainboard functions will return \c Result::InvalidState when
         * \a VSQT is disabled. Make sure to enable \a VSQT prior to calling these functions.
         *
         * @param[in] enable If \c true, \a VSQT is enabled; if \c false, \a VSQT is disabled.
         * @return Result Returns \c Result::Ok if \a 3V3 was enabled or disabled successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result enableVSQT(bool enable);

        /**
         * @brief Measure the supply voltage.
         *
         * Measures the \a VUSB or \a VDC voltage. \a VUSB is the power input from the USB-C connector,
         * while \a VDC is the power input from the respective header pin.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This function can block for 100 ms.
         *
         * @param[out] voltage The measured voltage in millivolts (mV).
         * @return Result Returns \c Result::Ok if the supply voltage was measured successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getSupplyVoltage(uint16_t &voltage);

        /**
         * @brief Measure the supply current.
         *
         * Measures the current drawn from \a VUSB or \a VDC. \a VUSB is the power input from the USB-C connector,
         * while \a VDC is the power input from the respective header pin.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This function can block for 100 ms.
         *
         * @param[out] voltage The measured current draw in milliamperes (mA).
         * @return Result Returns \c Result::Ok if the supply current was measured successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getSupplyCurrent(int16_t &current);

        /**
         * @brief Check if the supply is good.
         *
         * Checks if the supply, whether \a VUSB or \a VDC is good as determined by the charger. A good supply
         * means that it powers the board and connected loads, not the battery.
         *
         * @param[out] good If \c true, the charger has determined the supply to be good; \c false if not.
         * @return Result Returns \c Result::Ok if the supply was checked successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result checkSupplyGood(bool &good);

        /**
         * @brief Set the voltage at which the supply should be maintained at.
         *
         * The charger dynamically regulates the current drawn from the supply to prevent it from collapsing under
         * the set maintain voltage. This is useful for specifying the maximum power point (MPP) voltage if using a
         * solar panel; allowing the charger to extract power from the panel more effectively at near-MPPT
         * effectiveness.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * @param[in] voltage The supply voltage to maintain in millivolts (mV).
         * @return Result Returns \c Result::Ok if the supply voltage to maintain was set successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setSupplyMaintainVoltage(uint16_t voltage);

        /**
         * @brief Enter ship mode.
         *
         * Ship mode is a power state that only consumes around 1.5 μA. Only the charger and
         * the fuel gauge is powered.
         *
         * This mode can only be entered into if the battery is powering the board and connected loads;
         * that is, if \c checkSupplyGood output parameter \p good is \c false. This function returns
         * a \c Result if it fails to enter ship mode.
         *
         * Ship mode can be exited by either (1) pulling \a QON header pin low for around 800 ms or
         * (2) connecting a power supply which the charger determines to be good.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This function can block for 20 ms if it fails to enter ship mode.
         *
         * @return Result Does not return if ship mode was successfully entered into;
         * returns a value other than \c Result::Ok if not.
         */
        Result enterShipMode();

        /**
         * @brief Enter shutdown mode.
         *
         * Shutdown mode is a power state that only consumes around 1.4 μA. Only the charger and
         * the fuel gauge is powered.
         *
         * This mode can only be entered into if the battery is powering the board and connected loads;
         * that is, if \c checkSupplyGood output parameter \p good is \c false. This function returns
         * a \c Result if it fails to enter shutdown mode.
         *
         * Shutdown mode can only be exited by connecting a power supply which the charger determines to be good.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This function can block for 20 ms if it fails to enter shutdown mode.
         *
         * @return Result Does not return if shutdown mode was successfully entered into;
         * returns a value other than \c Result::Ok if not.
         */
        Result enterShutdownMode();

        /**
         * @brief Perform a power cycle.
         *
         * For all components on the board and connected loads, except the fuel gauge
         * and loads connected to \a VS (supply output header pin, whichever of \a VUSB and \a VDC),
         * the power cycle removes power reconnects it again after a short delay.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * @return Result Does not return if a power cycle has been successfully performed;
         * returns a value other than \c Result::Ok if not.
         */
        Result doPowerCycle();

        /**
         * @brief Enable or disable battery charging.
         *
         * This is useful when opting to not fully charge a battery in order to reduce wear and prolong
         * its lifespan.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called.
         *
         * @param[in] enable If \c true, battery charging is enabled; if \c false, battery charging is disabled.
         * @return Result Returns \c Result::Ok if battery charging has been enabled or disabled successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result enableBatteryCharging(bool enable);

        /**
         * @brief Set maximum battery charging current.
         *
         * Ensures that the battery is not charged with a current more than the one specified using this function.
         * This is useful for small capacity batteries, since it is not recommended charge a battery at
         * more than 1 C. For example, when charging a 550 mAh battery, a current of no more than 550 mA is
         * recommended.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called.
         *
         * @param[in] current The maximum charging current up to 2000 mA.
         * @return Result Returns \c Result::Ok if battery charging current has been set successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setBatteryChargingMaxCurrent(uint16_t current);

        /**
         * @brief Enable or disable battery temperature measurement.
         *
         * Enables or disables battery temperature measurement using a 103AT thermistor connected to the \a TS pin.
         * When enabled, the charger also implements temperature-based battery charging current reduction or cutoff.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called.
         *
         * @param[in] enable If \c true, battery temperature measurement is enabled; if \c false, battery
         * temperature measurement is disabled.
         * @return Result Returns \c Result::Ok if battery temperature measurement current has been enabled or
         * disabled successfully; returns a value other than \c Result::Ok if not.
         */
        Result enableBatteryTempSense(bool enable);

        /**
         * @brief Enable or disable the fuel guage.
         *
         * Disabling the fuel guage can save around 0.5 μA. However, once disabled, the fuel gauge
         * cannot keep track of battery information such as voltage, charge, health, cycle count, etc.
         * This is useful when trying to reduce power as much as possible, such as when going
         * into ship mode or shutdown mode for a long time.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called.
         *
         * @param[in] enable If \c true, fuel gauge enabled; if \c false, fuel gauge is disabled.
         * @return Result Returns \c Result::Ok if fuel gauge has been enabled or disabled successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result enableBatteryFuelGauge(bool enable);

        /**
         * @brief Measure battery voltage.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called.
         *
         * This function can block for 100 ms.
         *
         * @param[out] voltage Measured battery voltage in millivolts (mV).
         * @return Result Returns \c Result::Ok if battery voltage measurement was done successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryVoltage(uint16_t &voltage);

        /**
         * @brief Measure battery current.
         *
         * Measures the current to (when charging) or from (during discharge) the battery.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called.
         *
         * This function can block for 100 ms.
         *
         * @param[out] current Measured battery voltage in milliamps (mA). If battery is discharging,
         * this value is negative; positive if battery is charging.
         * @return Result Returns \c Result::Ok if battery current measurement was done successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryCurrent(int16_t &current);

        /**
         * @brief Estimate battery charge.
         *
         * Gives an estimate of battery state-of-charge from 0 to 100 percent.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called or when fuel gauge is disabled.
         *
         * @param[out] percent Estimated battery charge, from 0 to 100 percent.
         * @return Result Returns \c Result::Ok if battery charge estimation was done successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryCharge(uint8_t &percent);

        /**
         * @brief Estimate battery health.
         *
         * Gives an estimate of battery state-of-health from 0 to 100 percent. This is useful to get a
         * sense of how much the battery has degraded over time.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called or when fuel gauge is disabled.
         *
         * @param[out] percent Estimated battery health, from 0 to 100 percent.
         * @return Result Returns \c Result::Ok if battery health estimation was done successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryHealth(uint8_t &percent);

        /**
         * @brief Estimate battery cycle count.
         *
         * Gives an estimate of the number of battery cycle count. This is useful to compare against the
         * battery's rated cycle counts.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called or when fuel gauge is disabled.
         *
         * @param[out] cycles Estimated battery cycle count.
         * @return Result Returns \c Result::Ok if battery cycle count estimation was done successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryCycles(uint16_t &cycles);

        /**
         * @brief Estimate battery time left to charge or discharge.
         *
         * Gives an estimate of the battery's time-to-empty or time-to-full in minutes. This function call
         * only succeeds when the percent charge has previously dropped and/or risen by 10% for the time-to-empty or
         * time-to-full estimate, respectively; else \c Result::NotReady is returned.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called or when fuel gauge is disabled.
         *
         * @param[out] minutes Estimated battery charge or discharge time left in minutes. If battery is discharging,
         * this value is negative; positive if battery is charging.
         * @return Result Returns \c Result::Ok if estimation of battery charge or discharge time left was done
         * successfully; returns a value other than \c Result::Ok if not.
         */
        Result getBatteryTimeLeft(int &minutes);

        /**
         * @brief Measure battery temperature.
         *
         * Requires a Semitec 103AT thermistor to be connected to the \a TS pin and attached to the battery
         * for the measurement to be valid.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * Battery temperature measurement must be enabled using \c Mainboard::enableBatteryTempSense prior to calling
         * this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called.
         *
         * This function can block for 100 ms.
         *
         * @param[out] cycles Estimated battery cycle count.
         * @return Result Returns \c Result::Ok if battery cycle count estimation was done successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryTemperature(float &celsius);

        /**
         * @brief Set an alarm for battery low voltage.
         *
         * If battery voltage is less than the set voltage, \a ALARM pin is pulled low.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called or when fuel gauge is disabled.
         *
         * @param[in] voltage The voltage at which the low voltage alarm will trigger, from 2500 mV to 5000 mV
         * inclusive. If zero the alarm is disabled and existing low voltage alarms are cleared.
         * @return Result Returns \c Result::Ok if battery low voltage alarm was set successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setBatteryLowVoltageAlarm(uint16_t voltage);

        /**
         * @brief Set an alarm for battery high voltage.
         *
         * If battery voltage is more than the set voltage, \a ALARM pin is pulled low.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called or when fuel gauge is disabled.
         *
         * @param[in] voltage The voltage at which the high voltage alarm will trigger, from 2500 mV to 5000 mV
         * inclusive. If zero the alarm is disabled and existing high voltage alarms are cleared.
         * @return Result Returns \c Result::Ok if battery high voltage alarm was set successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setBatteryHighVoltageAlarm(uint16_t voltage);

        /**
         * @brief Set an alarm for battery low charge.
         *
         * If battery charge is less than the set percentage, \a ALARM pin is pulled low.
         *
         * \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * This functions returns \c Result::InvalidState if \p capacity was set to zero when \c MainBoard::init
         * was called or when fuel gauge is disabled.
         *
         * @param[in] percent The percentage at which the low charge alarm will trigger, from 1% to 100%
         * inclusive. If zero the alarm is disabled and existing low charge alarms are cleared.
         * @return Result Returns \c Result::Ok if battery low charge alarm was set successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setBatteryLowChargeAlarm(uint8_t percent);

        BQ2562x &getCharger() { return _charger; }
        LC709204F &getFuelGauge() { return _fuelGauge; }

        static Mainboard &get();

    private:
        Mainboard() {}

        static constexpr int _i2cPort = 1;
        static constexpr uint32_t _i2cFreq = 100000;
        static constexpr uint32_t _i2cTimeout = 1000;

        static constexpr uint16_t _minBatteryCapacity = 50; // charger minimum charging current is 40 mA,
                                                            // fuel guage minimum supported is 50 mAh.
        static constexpr uint16_t _defaultMaxChargingCurrent = _minBatteryCapacity; // minimum charge current at 1C

        static constexpr uint16_t _chargerADCWaitTime = 100; // 80 ms actual
        static constexpr uint16_t _batfetCtrlWaitTime = 30; // 20 ms actual

#ifdef ARDUINO
        ArduinoMasterI2C _i2c{_i2cPort, Pin::SDA1, Pin::SCL1, _i2cFreq};
#else
        MasterI2C _i2c{_i2cPort, Pin::SDA1, Pin::SCL1, _i2cFreq};
#endif

        BQ2562x _charger{_i2c};
        LC709204F _fuelGauge{_i2c};

        bool _sqtEnabled{false};
        bool _fgEnabled{false};
        bool _initDone{false};
        uint32_t _chargerADCTime{0};
        uint16_t _batteryCapacity{0};
        BatteryType _batteryType{BatteryType::Generic_3V7};
        Mutex _mutex{100};

        bool _isFirst();
        bool _initInternalDigitalPin(gpio_num_t pin, gpio_mode_t mode);
        bool _initInternalRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode);
        Result _initFuelGauge();
        bool _setRTCPin(gpio_num_t pin, bool value);
        Result _udpateChargerADC();
    };

    extern Mainboard &Board;
}
