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

#include <climits>
#include <cstdint>
#include <cstring>
#include <math.h>

#include <soc/reset_reasons.h>
#include <esp_log.h>

#include "MainBoard.h"

namespace PowerFeather
{
    static_assert(CHAR_BIT == 8, "Unsupported architecture.");

    static const char* TAG = "MainBoard";

    /*extern*/ MainBoard& Board = MainBoard::get();

    #define LOG_FAIL(r)                 ESP_LOGE(TAG, "Unexpected result %d on %s:%d.", (r), __FUNCTION__, __LINE__)
    #define RET_IF_ERR(f)               { Result r = (f); if (r != Result::Ok) { LOG_FAIL(1); return r; } }
    #define RET_IF_NOK(f)               { esp_err_t r = (f); if (r != ESP_OK) { LOG_FAIL(r); return false; } }
    #define RET_IF_FALSE(f, r)          { if ((f) == false) { LOG_FAIL(false); return (r); } }
    #define TRY_LOCK(m)                 Mutex::Lock m##Lock(m); RET_IF_FALSE(m##Lock.isLocked(), Result::Busy);

    static RTC_NOINIT_ATTR uint32_t first;
    static const uint32_t firstMagic = 0xdeadbeef;

    /*static*/ MainBoard& MainBoard::get()
    {
        static MainBoard board;
        return board;
    }

    bool MainBoard::_initInternalRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode)
    {
        // Configure the RTC pin.
        RET_IF_NOK(rtc_gpio_init(pin));
        RET_IF_NOK(rtc_gpio_set_direction(pin, mode));
        RET_IF_NOK(rtc_gpio_set_direction_in_sleep(pin, mode));
        RET_IF_NOK(rtc_gpio_pullup_dis(pin));
        RET_IF_NOK(rtc_gpio_pulldown_dis(pin));
        ESP_LOGD(TAG, "Initialized RTC pin %d with mode %d.", pin, mode);
        return true;
    }

    bool MainBoard::_initInternalDigitalPin(gpio_num_t pin, gpio_mode_t mode)
    {
        // Configure the digital pin.
        gpio_config_t io_conf = {};
        memset(&io_conf, 0, sizeof(io_conf));
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = mode;
        io_conf.pin_bit_mask = (static_cast<uint64_t>(0b1) << pin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        RET_IF_NOK(gpio_config(&io_conf));
        ESP_LOGD(TAG, "Initialized digital pin %d with mode %d.", pin, mode);
        return true;
    }

    bool MainBoard::_setRTCPin(gpio_num_t pin, bool value)
    {
        // Disable pin hold, set the level, and re-enable pin hold.
        rtc_gpio_hold_dis(pin);
        rtc_gpio_set_level(pin, value);
        rtc_gpio_hold_en(pin);
        ESP_LOGD(TAG, "Set RTC pin %d to %d.", pin, value);
        return true;
    }

    bool MainBoard::_isFirst()
    {
        // If the RTC is domain is shutdown, consider next boot as first boot.
        bool isFirst = (first != firstMagic);
        ESP_LOGD(TAG, "Check if first boot: %d.", isFirst);
        return isFirst;
    }

    Result MainBoard::_initFuelGauge()
    {
        bool op = false;
        RET_IF_FALSE(getFuelGauge().getOperation(op), Result::Failure); // check first mode of operation

        if (!op) // if op is false, fuel gauge in sleep mode
        {
            // Reinitialize even if fuel gauge has been previously initialized, and was just
            // put into sleep mode using enableFuelGauge(false). TODO: verify
            LC709204F::ChangeOfParameter param = _batteryType == BatteryType::ICR18650 ? LC709204F::ChangeOfParameter::ICR18650_26H :
                                                 _batteryType == BatteryType::UR18650ZY ? LC709204F::ChangeOfParameter::UR18650ZY :
                                                 LC709204F::ChangeOfParameter::Nominal_3V7_Charging_4V2;
            RET_IF_FALSE(getFuelGauge().setAPA(_batteryCapacity, param), Result::Failure);
            RET_IF_FALSE(getFuelGauge().setChangeOfParameter(param), Result::Failure);
            RET_IF_FALSE(getFuelGauge().enableTSENSE(false, false), Result::Failure);
            RET_IF_FALSE(getFuelGauge().enableOperation(true), Result::Failure);
            ESP_LOGD(TAG, "Fuel gauge initialized.");
        }
        else
        {
            ESP_LOGD(TAG, "Fuel gauge already initialized.");
        }

        return Result::Ok;
    }

    Result MainBoard::init(uint16_t capacity, BatteryType type)
    {
        _mutex.init();

        TRY_LOCK(_mutex);

        // Capacity should either be 0, in which case it indicates to the SDK that there is no battery expected
        // to be connected to the system; or within _minBatteryCapacity and _maxBatteryCapacity inclusive.
        RET_IF_FALSE(!capacity || (capacity >= _minBatteryCapacity && capacity <= _maxBatteryCapacity), Result::InvalidArg);

        _initDone = false;

        _batteryCapacity = capacity;
        _batteryType = type;
        ESP_LOGD(TAG, "Battery capacity and type set to %d mAh, %d.", static_cast<int>(_batteryCapacity), static_cast<int>(_batteryType));

        // On first boot VSQT, through EN_SQT, is always enabled. On wake from deep sleep try and maintain held state.
        RET_IF_FALSE(_initInternalRTCPin(Pin::EN_SQT, RTC_GPIO_MODE_INPUT_OUTPUT), Result::Failure);
        _sqtEnabled = _isFirst() ? true : rtc_gpio_get_level(Pin::EN_SQT);
        RET_IF_FALSE(_setRTCPin(Pin::EN_SQT, _sqtEnabled), Result::Failure)
        ESP_LOGD(TAG, "VSQT set to %d during initialization", _sqtEnabled);

        RET_IF_FALSE(_i2c.init(_i2cPort, Pin::SDA0, Pin::SCL0, _i2cFreq), Result::Failure); // always initialize STEMMA QT, charger, fuel gauge I2C; TODO: check disabling

        if (_sqtEnabled)
        {
            bool wdOn = true;
            RET_IF_FALSE(getCharger().getWD(wdOn), Result::Failure);

            if (wdOn) // watchdog disabled means that the initiatialization was done previously
            {
                RET_IF_FALSE(getCharger().enableCharging(false), Result::Failure);
                RET_IF_FALSE(getCharger().enableTS(false), Result::Failure);
                RET_IF_FALSE(getCharger().setChargeCurrent(_defaultMaxChargingCurrent), Result::Failure);
                RET_IF_FALSE(getCharger().setBATFETDelay(BQ2562x::BATFETDelay::Delay20ms), Result::Failure);
                RET_IF_FALSE(getCharger().enableWVBUS(true), Result::Failure);
                RET_IF_FALSE(getCharger().enableInterrupts(false), Result::Failure);
                // Disable the charger watchdog to keep the charger in host mode and to
                // keep some registers from resetting to their POR values.
                RET_IF_FALSE(getCharger().enableWD(false), Result::Failure);
                ESP_LOGD(TAG, "Charger IC initialized.");
            }
            else
            {
                ESP_LOGD(TAG, "Charger IC already initialized.");
            }

            // If battery capacity is not 0, initialize the fuel gauge. This can fail if during
            // startup no battery is connected, therefore failures are not checked here. Fuel guage
            // initialization attempts will be made later, during calls to member functions that
            // talk to the fuel gauge.
            _fgEnabled = false;
            if (_batteryCapacity)
            {
                _initFuelGauge();
                _fgEnabled = true;
            }
        }

        // Initialize the rest of the RTC/digital pins managed by the SDK.
        RET_IF_FALSE(_initInternalRTCPin(Pin::EN0, RTC_GPIO_MODE_OUTPUT_OD), Result::Failure);
        RET_IF_FALSE(_initInternalRTCPin(Pin::EN_3V3, RTC_GPIO_MODE_OUTPUT_ONLY), Result::Failure);
        RET_IF_FALSE(_initInternalDigitalPin(Pin::PG, GPIO_MODE_INPUT), Result::Failure);

        if (_isFirst())
        {
            RET_IF_FALSE(_setRTCPin(Pin::EN0, true), Result::Failure);
            RET_IF_FALSE(_setRTCPin(Pin::EN_3V3, true), Result::Failure);
        }

        first = firstMagic;
        _initDone = true;

        ESP_LOGD(TAG, "Initialization done.");

        return Result::Ok;
    }

    Result MainBoard::setEN(bool value)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_setRTCPin(Pin::EN0, value), Result::Failure);
        ESP_LOGD(TAG, "EN set to: %d.", value);
        return Result::Ok;
    }

    Result MainBoard::enable3V3(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_setRTCPin(Pin::EN_3V3, enable), Result::Failure);
        ESP_LOGD(TAG, "3V3 set to: %d.", enable);
        return Result::Ok;
    }

    Result MainBoard::enableVSQT(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_setRTCPin(Pin::EN_SQT, enable), Result::Failure)
        _sqtEnabled = enable;
        ESP_LOGD(TAG, "VSQT set to: %d.", _sqtEnabled);
        return Result::Ok;
    }

    Result MainBoard::setSupplyMaintainVoltage(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(getCharger().setVINDPM(voltage), Result::Failure);
        ESP_LOGD(TAG, "Maintain supply voltage set to: %d mV.", voltage);
        return Result::Ok;
    }

    Result MainBoard::getSupplyCurrent(int16_t& current)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getIBUS(current), Result::Failure);
        ESP_LOGD(TAG, "Measured supply current: %d mA.", current);
        return Result::Ok;
    }

    Result MainBoard::getSupplyVoltage(uint16_t& voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getVBUS(voltage), Result::Failure);
        ESP_LOGD(TAG, "Measured supply voltage: %d mV.", voltage);
        return Result::Ok;
    }

    Result MainBoard::checkSupplyGood(bool& good)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        good = (gpio_get_level(Pin::PG) == 0);
        ESP_LOGD(TAG, "Check power supply good: %d.", good);
        return Result::Ok;
    }

    Result MainBoard::enterShipMode()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::ShipMode), Result::Failure);
        // If this executes, then charger did not enter ship mode. Return to normal operation
        // and return failure status. TODO: verify
        vTaskDelay(pdMS_TO_TICKS(_batfetCtrlWaitTime));
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::Normal), Result::Failure);
        ESP_LOGD(TAG, "Failed to enter ship mode.");
        return Result::Failure;
    }

    Result MainBoard::enterShutdownMode()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::ShutdownMode), Result::Failure);
        // If this executes, then charger did not enter shutdown mode. According to the datasheet,
        // charger automatically returns to normal mode, so just return failure. TODO: verify
        vTaskDelay(pdMS_TO_TICKS(_batfetCtrlWaitTime));
        ESP_LOGD(TAG, "Failed to enter shutdown mode.");
        return Result::Failure;
    }

    Result MainBoard::doPowerCycle()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::SystemPowerReset), Result::Failure);
        // If this executes, then charger did not perform power cycle. TODO: verify
        vTaskDelay(pdMS_TO_TICKS(_batfetCtrlWaitTime));
        ESP_LOGD(TAG, "Failed to do power cycle.");
        return Result::Failure;
    }

    Result MainBoard::enableTempSense(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(getCharger().enableTS(enable), Result::Failure);
        ESP_LOGD(TAG, "Temperature sense set to: %d.", enable);
        return Result::Ok;
    }

    Result MainBoard::enableCharging(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(getCharger().enableCharging(enable), Result::Failure);
        ESP_LOGD(TAG, "Charging set to: %d.", enable);
        return Result::Ok;
    }

    Result MainBoard::setChargingMaxCurrent(uint16_t current)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(current <= _maxChargingCurrent, Result::InvalidArg);
        RET_IF_FALSE(getCharger().setChargeCurrent(current), Result::Failure);
        ESP_LOGD(TAG, "Max charging current set to: %d mA.", current);
        return Result::Ok;
    }

    Result MainBoard::getBatteryTemperature(float& celsius)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);

        // Temperature sensing must be enabled using enableTempSense(true), only
        // then can the battery temperature be measured.
        bool enabled = false;
        RET_IF_FALSE(getCharger().getTS(enabled) && enabled, Result::InvalidState);

        float voltage = 0;
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getTS_ADC(voltage), Result::Failure);
        // Map percent to temperature given 103AT thermistor with fitted curve.
        celsius = (-1866.96172 * powf(voltage, 4)) + (3169.31754 * powf(voltage, 3)) - (1849.96775 * powf(voltage, 2)) + (276.6656 * voltage) + 81.98758;
        ESP_LOGD(TAG, "Measured battery temperature: %f °C.", celsius);
        return Result::Ok;
    }

    Result MainBoard::getBatteryCurrent(int16_t& current)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getIBAT(current), Result::Failure);
        ESP_LOGD(TAG, "Measured battery current: %d mA.", current);
        return Result::Ok;
    }

    Result MainBoard::enableFuelGauge(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);

        if (enable)
        {
            // Do also reinitialization here, since it is possible that the system initialized
            // with battery present, then battery is disconected, then battery is reconnected,
            // the finally this function is called.
            RET_IF_ERR(_initFuelGauge());
        }
        else
        {
            RET_IF_FALSE(getFuelGauge().enableOperation(false), Result::Failure);
        }
        _fgEnabled = enable;
        ESP_LOGD(TAG, "Fuel gauge set to: %d.", _fgEnabled);
        return Result::Ok;
    }

    Result MainBoard::getBatteryVoltage(uint16_t& voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        if (!(_batteryCapacity && _fgEnabled && _initFuelGauge() == Result::Ok && getFuelGauge().getCellVoltage(voltage))) // if fuel gauge is available, use the reading from it
        {
            RET_IF_ERR(_udpateChargerADC());
            RET_IF_FALSE(getCharger().getVBAT(voltage), Result::Failure);
        }
        return Result::Ok;
    }

    Result MainBoard::getBatteryCharge(uint8_t& percent)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgEnabled, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getRSOC(percent), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryHealth(uint8_t& percent)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgEnabled, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getSOH(percent), Result::Failure);
        ESP_LOGD(TAG, "Estimated battery charge health: %d %%.", percent);
        return Result::Ok;
    }

    Result MainBoard::getBatteryCycles(uint16_t& cycles)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgEnabled, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getCycles(cycles), Result::Failure);
        ESP_LOGD(TAG, "Estimated battery cycles: %d.", cycles);
        return Result::Ok;
    }

    Result MainBoard::getBatteryTimeLeft(int& minutes)
    {
        TRY_LOCK(_mutex);

        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgEnabled, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());

        // Check first the current direction, whether to or from the battery.
        // Negative means from battery to the board (discharge), positive
        // means from board to battery (charge).
        int16_t ibat = 0;
        RET_IF_ERR(getBatteryCurrent(ibat));
        bool discharging = ibat < 0;

        // Get the time-to-empty or time-to-full depending on battery is charging
        // or discharging.
        uint16_t mins = 0;
        RET_IF_FALSE(discharging ? getFuelGauge().getTimeToEmpty(mins) : getFuelGauge().getTimeToFull(mins), Result::Failure);

        if (mins != 0xFFFF) // check if already the required 10 % rise/drop in charge
        {
            minutes = mins * (discharging ? -1 : 1); // return negative amount of time for discharging
            return Result::Ok;
        }

        ESP_LOGD(TAG, "Estimated battery time left (%s): %d mins.", discharging ? "discharging" : "charging", abs(minutes));
        return Result::NotReady;
    }

    Result MainBoard::setBatteryLowVoltageAlarm(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgEnabled, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE((voltage >= 2500 && voltage <= 5000) || voltage == 0, Result::InvalidArg);
        RET_IF_FALSE(getFuelGauge().setLowVoltageAlarm(voltage), Result::Failure);
        ESP_LOGD(TAG, "Low battery voltage alarm set to: %d mV.", voltage);
        return Result::Ok;
    };

    Result MainBoard::setBatteryHighVoltageAlarm(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgEnabled, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE((voltage >= 2500 && voltage <= 5000) || voltage == 0, Result::InvalidArg); // TODO: clear alarm when 0
        bool oper = 0;
        RET_IF_FALSE(getFuelGauge().getOperation(oper), Result::Failure);
        RET_IF_FALSE(getFuelGauge().setHighVoltageAlarm(voltage), Result::Failure);
        ESP_LOGD(TAG, "High battery voltage alarm set to: %d mV.", voltage);
        return Result::Ok;
    };

    Result MainBoard::setBatteryLowChargeAlarm(uint8_t percent)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgEnabled, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE((percent >= 1 && percent <= 100) || percent == 0, Result::InvalidArg); // TODO: clear alarm when 0
        RET_IF_FALSE(getFuelGauge().setLowRSOCAlarm(percent), Result::Failure);
        ESP_LOGD(TAG, "Low charge alarm set to: %d %%.", (int)percent);
        return Result::Ok;
    };

    Result MainBoard::_udpateChargerADC()
    {
        uint32_t now = 0;
        // Since updating ADC values take a long time, only update it again after _chargerADCWaitTime has elapsed.
        if (_chargerADCTime == 0 || (now - _chargerADCTime) >= _chargerADCWaitTime)
        {
            bool done = false;
            RET_IF_FALSE(getCharger().setupADC(true, BQ2562x::ADCRate::Oneshot, BQ2562x::ADCSampling::Bits_10), Result::Failure);
            vTaskDelay(pdMS_TO_TICKS(_chargerADCWaitTime));
            RET_IF_FALSE(getCharger().getADCDone(done) && done, Result::Failure);
            _chargerADCTime = now;
            ESP_LOGD(TAG, "Updated charger ADC.");
        }
        return Result::Ok;
    }
}
