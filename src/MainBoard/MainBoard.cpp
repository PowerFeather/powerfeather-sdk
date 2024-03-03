/**
 *                           POWERFEATHER 4-CLAUSE LICENSE
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

#include "MainBoard.h"

namespace PowerFeather
{
    MainBoard& Board = MainBoard::get();

    static_assert(CHAR_BIT == 8, "Unsupported architecture");

    #define RET_IF_ERR(f)        { Result r = (f); if (r != Result::Ok) { return r; } }
    #define RET_IF_NOK(f, r)     { if ((f) != ESP_OK) { return (r); } }
    #define RET_IF_FALSE(f, r)   { if ((f) == false) { return (r); } }
    #define TRY_LOCK(m)          Mutex::Lock m##Lock(m); RET_IF_FALSE(m##Lock.isLocked(), Result::Busy);

    static RTC_NOINIT_ATTR uint32_t first;
    static const uint32_t firstMagic = 0xdeadbeef;

    /*static*/ MainBoard& MainBoard::get()
    {
        static MainBoard board;
        return board;
    }

    bool MainBoard::_initInternalRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode)
    {
        RET_IF_NOK(rtc_gpio_init(pin), false);
        RET_IF_NOK(rtc_gpio_set_direction(pin, mode), false);
        RET_IF_NOK(rtc_gpio_set_direction_in_sleep(pin, mode), false);
        RET_IF_NOK(rtc_gpio_pullup_dis(pin), false);
        RET_IF_NOK(rtc_gpio_pulldown_dis(pin), false);
        return true;
    }

    bool MainBoard::_initInternalDigitalPin(gpio_num_t pin, gpio_mode_t mode)
    {
        gpio_config_t io_conf = {};
        memset(&io_conf, 0, sizeof(io_conf));
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = mode;
        io_conf.pin_bit_mask = (static_cast<uint64_t>(0b1) << pin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        RET_IF_NOK(gpio_config(&io_conf), false);
        return true;
    }

    bool MainBoard::_setRTCPin(gpio_num_t pin, bool value)
    {
        // Disable pin hold during deep sleep
        rtc_gpio_hold_dis(pin);
        // Set the pin high.
        rtc_gpio_set_level(pin, value);
        rtc_gpio_hold_en(pin);
        return true;
    }

    bool MainBoard::_isFirst()
    {
        return first != firstMagic;
    }

    Result MainBoard::_initFuelGauge()
    {
        bool op = false;
        RET_IF_FALSE(getFuelGauge().getOperation(op), Result::Failure);

        if (!op)
        {
            LC709204F::ChangeOfParameter param = _batteryType == BatteryType::ICR18650 ? LC709204F::ChangeOfParameter::ICR18650_26H :
                                                 _batteryType == BatteryType::UR18650ZY ? LC709204F::ChangeOfParameter::UR18650ZY :
                                                 LC709204F::ChangeOfParameter::Nominal_3V7_Charging_4V2;
            RET_IF_FALSE(getFuelGauge().setAPA(_batteryCapacity, param), Result::Failure);
            RET_IF_FALSE(getFuelGauge().setChangeOfParameter(param), Result::Failure);
            RET_IF_FALSE(getFuelGauge().enableTSENSE(false, false), Result::Failure);
            RET_IF_FALSE(getFuelGauge().enableOperation(true), Result::Failure);
        }
        return Result::Ok;
    }

    Result MainBoard::init(uint16_t capacity, BatteryType type)
    {
        _mutex.init();
        TRY_LOCK(_mutex);

        RET_IF_FALSE(!capacity || (capacity >= _minBatteryCapacity && capacity <= _maxBatteryCapacity), Result::InvalidArg);

        _initDone = false;
        _batteryCapacity = capacity;
        _batteryType = type;

        RET_IF_FALSE(_initInternalRTCPin(Pin::EN_SQT, RTC_GPIO_MODE_INPUT_OUTPUT), Result::Failure);
        _sqtOn = _isFirst() ? true : rtc_gpio_get_level(Pin::EN_SQT); // try to maintain across deep sleep
        RET_IF_FALSE(_setRTCPin(Pin::EN_SQT, _sqtOn), Result::Failure)

        RET_IF_FALSE(_i2c.init(_i2cPort, Pin::SDA0, Pin::SCL0, _i2cFreq), Result::Failure); // initialize i2c always

        if (_sqtOn)
        {
            bool wdOn = false;
            RET_IF_FALSE(getCharger().getWD(wdOn), Result::Failure);

            if (wdOn) // watchdog disabled means that the initiatialization was done previously
            {
                RET_IF_ERR(enableCharging(false));
                RET_IF_ERR(enableTempSense(false));
                RET_IF_ERR(setChargingMaxCurrent(MainBoard::_defaultChargingMaxCurrent));
                RET_IF_FALSE(getCharger().setBATFETDelay(BQ2562x::BATFETDelay::Delay20ms), Result::Failure);
                RET_IF_FALSE(getCharger().enableWVBUS(true), Result::Failure);
                RET_IF_FALSE(getCharger().enableInterrupts(false), Result::Failure);
                // Disable the charger watchdog to keep the charger in host mode and to
                // keep some registers from resetting to their POR values.
                RET_IF_FALSE(getCharger().enableWD(false), Result::Failure);
            }

            _fgOn = false;
            if (_batteryCapacity)
            {
                _initFuelGauge();
                _fgOn = true;
            }
        }

        RET_IF_FALSE(_initInternalRTCPin(Pin::EN0, RTC_GPIO_MODE_OUTPUT_OD), Result::Failure);
        RET_IF_FALSE(_initInternalRTCPin(Pin::EN_3V3, RTC_GPIO_MODE_OUTPUT_ONLY), Result::Failure);

        if (_isFirst())
        {
            RET_IF_ERR(setEN(true));
            RET_IF_ERR(enable3V3(true));
        }

        RET_IF_FALSE(_initInternalDigitalPin(Pin::PG, GPIO_MODE_INPUT), Result::Failure);
        first = firstMagic;
        _initDone = true;
        return Result::Ok;
    }

    Result MainBoard::enableFuelGauge(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(!enable || _batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(getFuelGauge().enableOperation(enable), Result::Failure);
        _fgOn = enable;
        return Result::Ok;
    }

    Result MainBoard::setEN(bool value)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone || _isFirst(), Result::InvalidState);
        RET_IF_FALSE(_setRTCPin(Pin::EN0, value), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enable3V3(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone || _isFirst(), Result::InvalidState);
        RET_IF_FALSE(_setRTCPin(Pin::EN_3V3, enable), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enableVSQT(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_setRTCPin(Pin::EN_SQT, enable), Result::Failure)
        _sqtOn = enable;
        return Result::Ok;
    }

    Result MainBoard::setSupplyMaintainVoltage(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(getCharger().setVINDPM(voltage), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getSupplyCurrent(int16_t& current)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getIBUS(current), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getSupplyVoltage(uint16_t& voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getVBUS(voltage), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getSupplyStatus(bool& connected)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        connected = gpio_get_level(Pin::PG) == 0;
        return Result::Ok;
    }

    Result MainBoard::enterShipMode()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::ShipMode), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enterShutdownMode()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::ShutdownMode), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::doPowerCycle()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::SystemPowerReset), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enableTempSense(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone || _isFirst(), Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(getCharger().enableTS(enable), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enableCharging(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone || _isFirst(), Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(getCharger().enableCharging(enable), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::setChargingMaxCurrent(uint16_t current)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone || _isFirst(), Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(current <= 2000, Result::InvalidArg);
        RET_IF_FALSE(getCharger().setChargeCurrent(current), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryTemperature(float& celsius)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);

        bool enabled = false;
        RET_IF_FALSE(getCharger().getTS(enabled) && enabled, Result::InvalidState);

        RET_IF_ERR(_udpateChargerADC());

        float voltage = 0;
        RET_IF_FALSE(getCharger().getTS_ADC(voltage), Result::Failure);
        // Map percent to temperature given 103AT thermistor with fitted curve (see ts_calc.fods).
        celsius = (-1866.96172 * powf(voltage, 4)) + (3169.31754 * powf(voltage, 3)) - (1849.96775 * powf(voltage, 2)) + (276.6656 * voltage) + 81.98758;

        return Result::Ok;
    }

    Result MainBoard::getBatteryCurrent(int16_t& current)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getIBAT(current), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryVoltage(uint16_t& voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        if (!(_batteryCapacity && _fgOn && _initFuelGauge() == Result::Ok && getFuelGauge().getCellVoltage(voltage))) // if fuel gauge is available, use the reading from it
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
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgOn, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getRSOC(percent), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryHealth(uint8_t& percent)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgOn, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getSOH(percent), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryCycles(uint16_t& cycles)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgOn, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getCycles(cycles), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryTimeLeft(int& minutes)
    {
        TRY_LOCK(_mutex);

        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgOn, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());

        int16_t ibat = 0;
        RET_IF_ERR(getBatteryCurrent(ibat));

        uint16_t mins = 0;
        bool discharging = ibat < 0;

        RET_IF_FALSE(discharging ? getFuelGauge().getTimeToEmpty(mins) : getFuelGauge().getTimeToFull(mins), Result::Failure);

        if (mins != 0xFFFF)
        {
            minutes = mins * (discharging ? -1 : 1);
            return Result::Ok;
        }

        return Result::NotReady;
    }

    Result MainBoard::setBatteryLowVoltageAlarm(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgOn, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE((voltage >= 2500 && voltage <= 5000) || voltage == 0, Result::InvalidArg);
        RET_IF_FALSE(getFuelGauge().setLowVoltageAlarm(voltage), Result::Failure);
        return Result::Ok;
    };

    Result MainBoard::setBatteryHighVoltageAlarm(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgOn, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE((voltage >= 2500 && voltage <= 5000) || voltage == 0, Result::InvalidArg);
        bool oper = 0;
        RET_IF_FALSE(getFuelGauge().getOperation(oper), Result::Failure);
        RET_IF_FALSE(getFuelGauge().setHighVoltageAlarm(voltage), Result::Failure);
        return Result::Ok;
    };

    Result MainBoard::setBatteryLowChargeAlarm(uint8_t percent)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _fgOn, Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE((percent >= 1 && percent <= 100) || percent == 0, Result::InvalidArg);
        RET_IF_FALSE(getFuelGauge().setLowRSOCAlarm(percent), Result::Failure);
        return Result::Ok;
    };

    Result MainBoard::_udpateChargerADC()
    {
        uint32_t now = 0;
        // Since updating ADC values take a long time, only update it again after _chargerADCWaitTime has elapsed.
        if (_chargerADCTime == 0 || now - _chargerADCTime >= _chargerADCWaitTime)
        {
            bool done = false;
            RET_IF_FALSE(getCharger().setupADC(true, BQ2562x::ADCRate::Oneshot, BQ2562x::ADCSampling::Bits_10), Result::Failure);
            vTaskDelay(pdMS_TO_TICKS(_chargerADCWaitTime));
            RET_IF_FALSE(getCharger().getADCDone(done) && done, Result::Failure);
            _chargerADCTime = now;
        }
        return Result::Ok;
    }
}
