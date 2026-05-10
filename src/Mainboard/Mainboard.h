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

#include <driver/gpio.h>
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
#include "MAX17260.h"

#ifndef POWERFEATHER_ENABLE_PF_STATE_STREAM
#define POWERFEATHER_ENABLE_PF_STATE_STREAM 0
#endif

namespace PowerFeather
{
    class Mainboard
    {
    public:
        enum class BatteryType
        {
            Generic_3V7, // Generic Li-ion/LiPo, 3.7 V nominal and 4.2 V max
            ICR18650_26H, // Samsung ICR18650-26H
            UR18650ZY, // Panasonic UR18650ZY
            Generic_LFP
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
            static constexpr gpio_num_t ALARM =     GPIO_NUM_21; // Battery fuel gauge alarm
            static constexpr gpio_num_t INT =       GPIO_NUM_5;  // Battery charge interrupt

            static constexpr gpio_num_t LED =       GPIO_NUM_46; // User LED
            static constexpr gpio_num_t BTN =       GPIO_NUM_0;  // User button
            static constexpr gpio_num_t EN =        GPIO_NUM_7;  // FeatherWings enable/disable (read)

        private:
            // SDK-Managed Fixed
            static constexpr gpio_num_t USBDP =     GPIO_NUM_20;  // USB D+
            static constexpr gpio_num_t USBDM =     GPIO_NUM_19;  // USB D-

            static constexpr gpio_num_t EN_3V3 =    GPIO_NUM_4;  // 3V3 enable/disable
            static constexpr gpio_num_t EN_SQT =    GPIO_NUM_14; // VSQT enable/disable
            static constexpr gpio_num_t EN0 =       GPIO_NUM_13; // FeatherWings enable/disable (write)

            static constexpr gpio_num_t SCL1 =      GPIO_NUM_48; // STEMMA QT I2C SCL
            static constexpr gpio_num_t SDA1 =      GPIO_NUM_47; // STEMMA QT I2C SDA

            static constexpr gpio_num_t PG =        GPIO_NUM_38; // Battery charger power good indicator
        };

        /**
         * @brief Initialize the board power management and monitoring features with no battery expected.
         *
         * Initializes the board with battery monitoring disabled. Use this when no battery is connected.
         * See \c init(uint16_t, BatteryType) for the full default list.
         *
         * @return Result Returns \c Result::Ok if the board was initialized successfully; returns a value other than \c Result::Ok if not.
         */
        Result init();

        /**
         * @brief Initialize the board power management and monitoring features.
         *
         * Initializes the battery charger, battery fuel gauge and other hardware related to power management and monitoring.
         *
         * Sets the following defaults:
         *  - \a EN: high
         *  - \a 3V3: enabled
         *  - \a VSQT: enabled
         *  - Charging: disabled
         *  - Maximum battery charging current request: 50 mA (programs 40 mA after BQ step quantization)
         *  - Maintain supply voltage: 4.6 V
         *  - Fuel gauge: enabled (use \c init() to disable)
         *  - Battery temperature sense: disabled
         *  - Battery alarms (low charge, high/low voltage): disabled
         *
         * On RTC-retaining warm boots, charger settings changed through public setters are retained only when the
         * battery or profile configuration still matches. If the configuration changes, initialization re-applies safe defaults.
         *
         * This function should be called once, before calling all other \c Mainboard functions.
         *
         * @param[in] capacity The capacity of the connected Li-ion/LiPo battery in milliamp-hours (mAh).
         * Valid range depends on board revision: V1 supports 50-6000 mAh, V2 supports 1-16383 mAh.
         * On V2, capacities below 50 mAh are supported for monitoring only: battery charging remains disabled
         * and charge-current configuration is rejected.
         * Must be non-zero; use \c init() when no battery is expected. If using multiple batteries connected
         * in parallel, specify only the capacity for one cell. Ignored when \p type is
         * \c BatteryType::ICR18650_26H or \c BatteryType::UR18650ZY.
         * @param[in] type Type of Li-ion/LiPo battery; ignored when value is \c BatteryType::ICR18650_26H or
         * \c BatteryType::UR18650ZY. Use \c init(const MAX17260::Model &profile) for MAX17260 profiles.
         *
         * @return Result Returns \c Result::Ok if the board was initialized successfully; returns a value other than \c Result::Ok if not.
         */
        Result init(uint16_t capacity, BatteryType type = BatteryType::Generic_3V7);

        /**
         * @brief Initialize the board using a MAX17260 model profile.
         *
         * The battery capacity is inferred from the profile.
         * On V2, inferred capacities below 50 mAh are supported for monitoring only: battery charging remains
         * disabled and charge-current configuration is rejected.
         *
         * The profile must provide a valid charger constant-voltage target in \c chargeVoltage.
         * This value is applied directly to the charger VREG/CV limit. Accepted range is 3.5-4.8 V.
         * The SDK does not infer a safe charge voltage from the custom model data; ensure this value
         * matches the connected cell chemistry because an incorrect value can overcharge the cell.
         * The profile's \c ichgTerm is also applied to the charger termination-current setting after
         * conversion from MAX17260 register units; it must correspond to 5-310 mA.
         *
         * @param[in] profile MAX17260 model profile.
         *
         * @return Result Returns \c Result::Ok if the board was initialized successfully; returns a value other than \c Result::Ok if not.
         */
        Result init(const MAX17260::Model &profile);

        /**
         * @brief Set \a EN pin high or low.
         *
         * This is useful for enabling or disabling connected Feather Wings to reduce power consumption.
         *
         * @param[in] high If \c true, EN is set high; if \c false, EN is set low.
         *
         * @return Result Returns \c Result::Ok if \a EN was set high or low successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setEN(bool high);

        /**
         * @brief Enable or disable \a 3V3.
         *
         * Enables or disables \a 3V3, the 3.3 V header pin power output. When disabled, power to the
         * connected loads on \a 3V3 is cut, reducing power consumption.
         *
         * @param[in] enable If \c true, \a 3V3 is enabled; if \c false, \a 3V3 is disabled.
         *
         * @return Result Returns \c Result::Ok if \a 3V3 was enabled or disabled successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result enable3V3(bool enable);

        /**
         * @brief Enable or disable \a VSQT.
         *
         * Enables or disables \a VSQT, the 3.3 V STEMMA QT power output. When disabled, power to the
         * connected STEMMA QT modules is cut, reducing power consumption.
         *
         * On V1, disabling \a VSQT also disables communications to the battery charger and fuel gauge.
         * This means that some of the other \c Mainboard functions will return \c Result::InvalidState when
         * \a VSQT is disabled. On V2, power-management I2C remains usable even with \a VSQT disabled.
         *
         * @param[in] enable If \c true, \a VSQT is enabled; if \c false, \a VSQT is disabled.
         *
         * @return Result Returns \c Result::Ok if \a VSQT was enabled or disabled successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result enableVSQT(bool enable);

        /**
         * @brief Enable or disable the \a STAT  LED.
         *
         * Normally, the STAT LED turns on when charging, or blinks when there is an error preventing
         * charging (when battery temperature exceeds the set threshold, for example).
         * This function can enable/disable this LED from turning on in these cases.
         *
         * One instance where disabling this LED is desirable is during low-sunlight charging conditions,
         * where the current extracted from the solar panel should be used to charge the battery as
         * much as possible.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * @param[in] enable If \c true, \a STAT LED is enabled; if \c false, \a STAT LED is disabled.
         *
         * @return Result Returns \c Result::Ok if \a STAT  LED was enabled or disabled successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result enableSTAT(bool enable);

        /**
         * @brief Measure the supply voltage.
         *
         * Measures the \a VUSB or \a VDC voltage. \a VUSB is the power input from the USB-C connector,
         * while \a VDC is the power input from the header pin. Resolution is approximately 0.004 V.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * This function can block for about 100 ms on a normal charger ADC refresh, plus
         * power-management I2C transfer time. I2C faults can add several 50 ms transaction
         * timeout windows before the function returns failure; tasks contending for the SDK
         * mutex are bounded by the mutex timeout while this call waits for the ADC conversion.
         *
         * @param[out] voltage The measured voltage in volts (V).
         *
         * @return Result Returns \c Result::Ok if the supply voltage was measured successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getSupplyVoltage(float &voltage);

        /**
         * @brief Measure the supply current.
         *
         * Measures the current drawn from \a VUSB or \a VDC. \a VUSB is the power input from the USB-C connector,
         * while \a VDC is the power input from the header pin. Resolution is 2 mA.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * This function can block for about 100 ms on a normal charger ADC refresh, plus
         * power-management I2C transfer time. I2C faults can add several 50 ms transaction
         * timeout windows before the function returns failure; tasks contending for the SDK
         * mutex are bounded by the mutex timeout while this call waits for the ADC conversion.
         *
         * @param[out] current The measured current draw in milliamperes (mA).
         *
         * @return Result Returns \c Result::Ok if the supply current was measured successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getSupplyCurrent(float &current);

        /**
         * @brief Check if the supply is good.
         *
         * Checks if the supply, whether \a VUSB or \a VDC is good as determined by the battery charger. A good supply
         * means that it powers the board and connected loads, not the battery.
         *
         * @param[out] good If \c true, the charger has determined the supply to be good; \c false if not.
         *
         * @return Result Returns \c Result::Ok if the supply was checked successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result checkSupplyGood(bool &good);

        /**
         * @brief Set the supply voltage to maintain.
         *
         * The battery charger dynamically regulates the current drawn from the supply to prevent it from collapsing under
         * the set voltage to maintain. This is useful for specifying the maximum power point (MPP) voltage if using a
         * solar panel; allowing the battery charger to extract power from the panel at near-MPPT effectiveness.
         * This programs the charger's VINDPM request. The charger's effective regulation point can be higher when
         * its battery-tracking behavior applies.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * @param[in] voltage The supply voltage to maintain in volts (V), from 4.6 V to 16.8 V.
         *
         * @return Result Returns \c Result::Ok if the supply voltage to maintain was set successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setSupplyMaintainVoltage(float voltage);

        /**
         * @brief Enter ship mode.
         *
         * Ship mode is a power state that only consumes around 1.5 μA. Only the battery charger and
         * the battery fuel gauge is powered.
         *
         * This mode can only be entered into if the battery is powering the board and connected loads;
         * that is, if \c checkSupplyGood output parameter \p good is \c false.
         *
         * Ship mode can be exited by either (1) pulling \a QON header pin low for around 800 ms or
         * (2) connecting a power supply which the battery charger determines to be good.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * This function can block for 30 ms if it fails to enter ship mode.
         *
         *
         * @return Result Does not return if ship mode was successfully entered into;
         * returns a value other than \c Result::Ok if not.
         */
        Result enterShipMode();

        /**
         * @brief Enter shutdown mode.
         *
         * Shutdown mode is a power state that only consumes around 1.4 μA. Only the battery charger and
         * the battery fuel gauge is powered.
         *
         * This mode can only be entered into if the battery is powering the board and connected loads;
         * that is, if \c checkSupplyGood output parameter \p good is \c false.
         *
         * Shutdown mode can only be exited by connecting a power supply which the battery charger determines to be good.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * This function can block for 30 ms if it fails to enter shutdown mode.
         *
         *
         * @return Result Does not return if shutdown mode was successfully entered into;
         * returns a value other than \c Result::Ok if not.
         */
        Result enterShutdownMode();

        /**
         * @brief Perform a power cycle.
         *
         * For all components on the board and connected loads, except the battery fuel gauge
         * and loads connected to \a VS (supply output header pin, whichever of \a VUSB and \a VDC),
         * the power cycle provides complete reset by removing power and re-applying it after a short delay.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * @return Result Does not return if a power cycle was performed successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result doPowerCycle();

        /**
         * @brief Enable or disable battery charging.
         *
         * This is useful when opting to not fully charge a battery in order to prolong its lifespan.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         * On V2, charging is not available for configured battery capacities below 50 mAh.
         *
         * @param[in] enable If \c true, battery charging is enabled; if \c false, battery charging is disabled.
         *
         * @return Result Returns \c Result::Ok if battery charging was enabled or disabled successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result enableBatteryCharging(bool enable);

        /**
         * @brief Set maximum battery charging current.
         *
         * Ensures that the battery is not charged with a current more than the amount specified using this function.
         * This is useful for batteries with small capacities, since it is not recommended to charge a battery at
         * more than 1C. For example, when charging a 550 mAh battery, a current of no more than 550 mA is
         * recommended. That current limit of 550 mA can be specified using this function.
         * The charger encodes this limit in 40 mA steps; values between steps are rounded down so the programmed
         * charger limit does not exceed the requested maximum.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         * On V2, this function is not available for configured battery capacities below 50 mAh.
         *
         * @param[in] current The maximum charging current in milliamps (mA), from 40 mA to 2000 mA.
         * Values between 40 mA register steps are rounded down.
         *
         * @return Result Returns \c Result::Ok if the maximum battery charging current was set successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setBatteryChargingMaxCurrent(float current);

        /**
         * @brief Enable or disable battery temperature measurement.
         *
         * Enables or disables battery temperature measurement using the thermistor connected to the \a TS pin.
         * If enabled, aside from measurement, the battery charger performs temperature-based battery charging current
         * reduction or cutoff.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * @param[in] enable If \c true, battery temperature measurement is enabled; if \c false, battery
         * temperature measurement is disabled.
         *
         * @return Result Returns \c Result::Ok if the battery temperature measurement was enabled or
         * disabled successfully; returns a value other than \c Result::Ok if not.
         */
        Result enableBatteryTempSense(bool enable);

        /**
         * @brief Enable or disable the battery fuel guage.
         *
         * Disabling the battery fuel guage can save around 0.5 μA. However, once disabled, it
         * cannot keep track of battery information such as voltage, charge, health, cycle count, etc.
         * Nonetheless, this is useful when trying to reduce power as much as possible, such as when going
         * into ship mode or shutdown mode for a long time.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * @param[in] enable If \c true, the battery fuel gauge is enabled; if \c false, the battery fuel gauge is disabled.
         *
         * @return Result Returns \c Result::Ok if the battery fuel gauge was enabled or disabled successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result enableBatteryFuelGauge(bool enable);

        /**
         * @brief Measure battery voltage.
         *
         * Resolution is approximately 0.002 V. If the fuel gauge is enabled and available, it is used;
         * otherwise, the charger VBAT ADC path is used as a fallback.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * This function can block for about 100 ms plus power-management I2C transfer time when
         * the charger ADC fallback path is used. I2C faults can add several 50 ms transaction
         * timeout windows before the function returns failure; tasks contending for the SDK
         * mutex are bounded by the mutex timeout while this call waits for the ADC conversion.
         *
         * @param[out] voltage Measured battery voltage in volts (V).
         *
         * @return Result Returns \c Result::Ok if the battery voltage was measured successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryVoltage(float &voltage);

        /**
         * @brief Measure battery current.
         *
         * Measures the current to or from the battery during charging and discharging, respectively.
         *
         * On V1, this function uses the charger `IBAT_ADC` register with a 4 mA LSb. BQ25628E
         * Table 8-35 specifies that `IBAT_ADC` resets to zero when charging is disabled; in
         * that case this function returns \c Result::NotReady instead of reporting a
         * misleading zero current.
         * On V2, this function uses the MAX17260 `Current` register with a 0.078125 mA LSb.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * The battery fuel gauge must be enabled on V2 prior to calling this function, else
         * \c Result::InvalidState is returned.
         *
         * This function can block for about 100 ms on a normal V1 charger ADC refresh, plus
         * power-management I2C transfer time. I2C faults can add several 50 ms transaction
         * timeout windows before the function returns failure; tasks contending for the SDK
         * mutex are bounded by the mutex timeout while this call waits for the ADC conversion.
         *
         * @param[out] current Measured battery current in milliamps (mA). If battery is discharging,
         * this value is negative; positive if battery is charging. On V1, this signed contract
         * applies only when the charger provides a valid `IBAT_ADC` reading.
         *
         * @return Result Returns \c Result::Ok if the battery current was measured successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryCurrent(float &current);

        /**
         * @brief Estimate battery charge.
         *
         * Gives an estimate of battery state-of-charge from 0% to 100%. This is useful to get a sense
         * if the battery still has much charge or is nearly empty.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * The battery fuel gauge must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * @param[out] percent Estimated battery charge in percent, from 0% to 100%.
         *
         * @return Result Returns \c Result::Ok if the battery charge was estimated successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryCharge(uint8_t &percent);

        /**
         * @brief Estimate battery health.
         *
         * Gives an estimate of battery state-of-health from 0% to 100%. This is useful to get a
         * sense of how much the battery has degraded over time.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * The battery fuel gauge must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * @param[out] percent Estimated battery health in percent, from 0% to 100%.
         *
         * @return Result Returns \c Result::Ok if the battery health was estimated successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryHealth(uint8_t &percent);

        /**
         * @brief Estimate battery cycle count.
         *
         * Gives an estimate of the battery cycle count. This is useful to compare against the number of
         * cycle counts the battery is rated for.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * The battery fuel gauge must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * @param[out] cycles Estimated battery cycle count.
         *
         * @return Result Returns \c Result::Ok if the battery cycle count was estimated successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryCycles(uint16_t &cycles);

        /**
         * @brief Estimate time left for battery to charge or discharge.
         *
         * Gives an estimate of the battery time-to-empty or time-to-full in minutes. The battery charge must have
         * previously dropped and/or risen by a certain percentage to be able to estimate time-to-empty or time-to-full, respectively.
         * If the gauge has not accumulated enough history yet, this function returns \c Result::NotReady.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * The battery fuel gauge must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * @param[out] minutes Estimated time left for battery to charge or discharge in minutes. If battery is discharging,
         * this value is negative; if battery is charging, this value is positive.
         *
         * @return Result Returns \c Result::Ok if the time left for battery to charge or discharge was estimated
         * successfully; returns a value other than \c Result::Ok if not.
         */
        Result getBatteryTimeLeft(int &minutes);

        /**
         * @brief Measure battery temperature.
         *
         * Requires a Semitec 103AT thermistor to be connected to the \a TS pin and attached to the battery
         * for the measurement to be accurate.
         *
         * Returns \c Result::Failure if the thermistor reading is outside the plausible range,
         * such as when the thermistor is missing, open, or shorted.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * Battery temperature measurement must be enabled prior calling this function, else \c Result::InvalidState
         * is returned.
         *
         * This function can block for about 100 ms on a normal charger ADC refresh, plus
         * power-management I2C transfer time. I2C faults can add several 50 ms transaction
         * timeout windows before the function returns failure; tasks contending for the SDK
         * mutex are bounded by the mutex timeout while this call waits for the ADC conversion.
         *
         * @param[out] celsius Measured battery temperature in celsius.
         *
         * @return Result Returns \c Result::Ok if the battery temperature was measured successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result getBatteryTemperature(float &celsius);

        /**
         * @brief Set an alarm for battery low voltage.
         *
         * If battery voltage is less than the set voltage, the \a ALARM pin is pulled low.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * The battery fuel gauge must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * @param[in] voltage The voltage at which the low voltage alarm will trigger in volts (V).
         * Valid non-zero range depends on board revision (2.5-5.0 V for V1, 0.02-5.1 V for V2). If zero,
         * triggering of the alarm is disabled and any existing low voltage
         * alarm is cleared.
         *
         * Alarm handling differs by board revision:
         *  - V1 (LC709204F): low-voltage status naturally clears once voltage returns above threshold.
         *  - V2 (MAX17260): low-voltage status is latched until explicitly cleared.
         *    Use \c setBatteryLowVoltageAlarm(0.0f) to clear, then set a non-zero threshold to re-arm.
         *
         * @return Result Returns \c Result::Ok if the battery low voltage alarm was set successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setBatteryLowVoltageAlarm(float voltage);

        /**
         * @brief Set an alarm for battery high voltage.
         *
         * If battery voltage is more than the set voltage, the \a ALARM pin is pulled low.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * The battery fuel gauge must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * @param[in] voltage The voltage at which the high voltage alarm will trigger in volts (V).
         * Valid non-zero range depends on board revision (2.5-5.0 V for V1, 0.02-5.1 V for V2). If zero,
         * triggering of the alarm is disabled and any existing high voltage
         * alarm is cleared.
         *
         * Alarm handling differs by board revision:
         *  - V1 (LC709204F): high-voltage status naturally clears once voltage returns below threshold.
         *  - V2 (MAX17260): high-voltage status is latched until explicitly cleared.
         *    Use \c setBatteryHighVoltageAlarm(0.0f) to clear, then set a non-zero threshold to re-arm.
         *
         * @return Result Returns \c Result::Ok if the battery high voltage alarm was set successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setBatteryHighVoltageAlarm(float voltage);

        /**
         * @brief Set an alarm for battery low charge.
         *
         * If battery charge is less than the set percentage, the \a ALARM pin is pulled low.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * The battery fuel gauge must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * @param[in] percent The percentage at which the low charge alarm will trigger in percent, from 1% to 100%.
         * If zero, triggering of the alarm is disabled and any existing low charge alarm is cleared.
         *
         * Alarm handling differs by board revision:
         *  - V1 (LC709204F): low-charge status follows gauge behavior.
         *  - V2 (MAX17260): low-charge status is latched until explicitly cleared.
         *    Use \c setBatteryLowChargeAlarm(0) to clear, then set a non-zero threshold to re-arm.
         *
         * @return Result Returns \c Result::Ok if the battery low charge alarm was set successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result setBatteryLowChargeAlarm(uint8_t percent);

        /**
         * @brief Update fuel guage with measured battery temperature
         *
         * In order to increase fuel gauge accuracy, you can update the fuel gauge with the battery
         * temperature obtained from getBatteryTemperature() or other sources.
         *
         * Temperature mode behavior depends on board revision:
         *  - V1: after initialization, fuel gauge temperature is host-updated (this API writes the value used for gauging).
         *  - V2: after initialization, fuel gauge temperature defaults to on-IC measurement until the first call to
         *        this API (or its no-arg overload), after which host-updated temperature is used.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * The battery fuel gauge must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * @param[in] temperature The temperature of the battery cell
         *
         * @return Result Returns \c Result::Ok if the fuel gauge's battery temperature has been update successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result updateBatteryFuelGaugeTemp(float temperature);

        /**
         * @brief Update fuel gauge temperature using the current battery thermistor measurement.
         *
         * Equivalent to calling \c getBatteryTemperature() then \c updateBatteryFuelGaugeTemp(float).
         * See \c updateBatteryFuelGaugeTemp(float) for V1/V2 temperature mode behavior details.
         *
         * On V1, \a VSQT must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         * On V2, power-management I2C remains usable with \a VSQT disabled.
         *
         * A non-zero \p capacity or \p type of \c BatteryType::ICR18650_26H / \c BatteryType::UR18650ZY
         * should have been specified when \c MainBoard::init was called, else \c Result::InvalidState is returned.
         *
         * Battery temperature measurement must be enabled prior calling this function, else \c Result::InvalidState
         * is returned.
         *
         * The battery fuel gauge must be enabled prior to calling this function, else \c Result::InvalidState is returned.
         *
         * @return Result Returns \c Result::Ok if the fuel gauge's battery temperature has been updated successfully;
         * returns a value other than \c Result::Ok if not.
         */
        Result updateBatteryFuelGaugeTemp();

        BQ2562x &getCharger() { return _charger; }
        /**
         * @brief Get the active fuel gauge instance.
         *
         * Returns the fuel gauge instance configured at compile time for this board revision.
         */
        FuelGauge &getFuelGauge();

#if POWERFEATHER_ENABLE_PF_STATE_STREAM
        Result testRestoreFuelGaugeLearnedStateAfterPor();
        Result testClearFuelGaugeInitSignature();
#endif

        static Mainboard &get();

    private:
        Mainboard() {}

    #if defined(CONFIG_ESP32S3_POWERFEATHER_V2) || defined(POWERFEATHER_BOARD_V2)
        using FuelGaugeImpl = MAX17260;
    #else
        using FuelGaugeImpl = LC709204F;
    #endif

        static constexpr int _i2cPort = 1;
        static constexpr uint32_t _i2cFreq = 100000;

        static constexpr float _defaultMaxChargingCurrent = 50.0f; // minimum charge current at 1C
        static constexpr uint16_t _minChargeableBatteryCapacity = 50;
        static constexpr float _minSupplyMaintainVoltage = BQ2562x::ResetVINDPMVoltage;

        static_assert(_minSupplyMaintainVoltage >= BQ2562x::MinVINDPMVoltage);
        static_assert(_defaultMaxChargingCurrent >= BQ2562x::MinChargingCurrent);

        static constexpr uint16_t _chargerADCWaitTime = 100; // 80 ms actual
        static constexpr uint16_t _batfetCtrlWaitTime = 30; // 30 ms actual

        struct FuelGaugeInitSignature
        {
            FuelGauge::InitSource source{FuelGauge::InitSource::Generic_3V7};
            uint16_t capacityMah{0};
            uint16_t terminationCurrentMa{0};
            float chargeVoltage{0.0f};
            uint32_t profileHash{0};
        };

#ifdef ARDUINO
        ArduinoMasterI2C _i2c{_i2cPort, Pin::SDA1, Pin::SCL1, _i2cFreq};
#else
        MasterI2C _i2c{_i2cPort, Pin::SDA1, Pin::SCL1, _i2cFreq};
#endif

        BQ2562x _charger{_i2c};
        FuelGaugeImpl _fuelGauge{_i2c};

#if !defined(CONFIG_ESP32S3_POWERFEATHER_V2) && !defined(POWERFEATHER_BOARD_V2)
        bool _sqtEnabled{false};
#endif
        bool _initDone{false};
        uint32_t _chargerADCTime{0};
        uint16_t _batteryCapacity{0};
        uint16_t _terminationCurrent{0};
        float _chargeVoltage{4.2f};
        float _chargingCurrentLimit{_defaultMaxChargingCurrent};
        float _vindpm{_minSupplyMaintainVoltage};
        uint32_t _profileHash{0};
        bool _tsEnabled{false};
        bool _chargingEnabled{false};
        BatteryType _batteryType{BatteryType::Generic_3V7};
        bool _usesProfile{false};
        FuelGaugeInitSignature _lastFuelGaugeInitSignature{};
        bool _hasFuelGaugeInitSignature{false};
        Mutex _mutex{100};

#if defined(CONFIG_ESP32S3_POWERFEATHER_V2) || defined(POWERFEATHER_BOARD_V2)
        bool _fuelGaugeUsingExternalTemp{false};
#endif

        bool _isFirst();
        bool _canAccessPowerI2C() const;
        bool _initInternalDigitalPin(gpio_num_t pin, gpio_mode_t mode);
        bool _initInternalRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode);
        bool _setRTCPin(gpio_num_t pin, bool value);
        uint16_t _capacityFromProfile(const MAX17260::Model &profile) const;
        Result _initInternal(uint16_t capacity, BatteryType type, const MAX17260::Model *profile);
        Result _udpateChargerADC();
        Result _applyChargerConfig();
        Result _reapplyChargerConfig(bool *applied = nullptr);
        Result _getBatteryCurrentLocked(float &current);

        bool _isFuelGaugeEnabled();
        Result _initFuelGauge();
    };

    extern Mainboard &Board; // singleton instance of Mainboard
}
