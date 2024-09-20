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

#include <limits.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <Utils/Logging.h>
#include <Utils/SoC.h>

#include "Mainboard.h"

namespace PowerFeather
{
    static_assert(CHAR_BIT == 8, "Unsupported architecture.");

    static const char *TAG = "PowerFeather::Mainboard";

    /*extern*/ Mainboard &Board = Mainboard::get();

    #define LOG_FAIL(r)                 Log.Debug(TAG, "Unexpected result %d on %s:%d.", (r), __FUNCTION__, __LINE__)
    #define RET_IF_ERR(f)               { Result r = (f); if (r != Result::Ok) { LOG_FAIL(1); return r; } }
    #define RET_IF_FALSE(f, r)          { if ((f) == false) { LOG_FAIL(false); return (r); } }
    #define TRY_LOCK(m)                 Mutex::Lock m##Lock(m); RET_IF_FALSE(m##Lock.isLocked(), Result::LockFailed);

    bool Mainboard::_isFuelGaugeEnabled()
    {
        bool enabled = false;
        getFuelGauge().getOperationMode(enabled); // failure here means false is returned
        return enabled;
    }

    Result Mainboard::_initFuelGauge()
    {
        bool inited = false;
        RET_IF_FALSE(getFuelGauge().getInitialized(inited), Result::Failure); // check if already initialized

        if (!inited)
        {
            LC709204F::ChangeOfParameter param = _batteryType == BatteryType::ICR18650_26H ? LC709204F::ChangeOfParameter::ICR18650_26H :
                                                 _batteryType == BatteryType::UR18650ZY ? LC709204F::ChangeOfParameter::UR18650ZY :
                                                 LC709204F::ChangeOfParameter::Nominal_3V7_Charging_4V2;
            RET_IF_FALSE(getFuelGauge().setAPA(_batteryCapacity, param), Result::Failure);
            RET_IF_FALSE(getFuelGauge().setChangeOfParameter(param), Result::Failure);

            const float minFactor = LC709204F::MinTerminationFactor;
            const float maxFactor = LC709204F::MaxTerminationFactor;
            float terminationFactor = _terminationCurrent/static_cast<float>(_batteryCapacity);
            terminationFactor = std::min(std::max(terminationFactor, minFactor), maxFactor);
            RET_IF_FALSE(getFuelGauge().setTerminationFactor(terminationFactor), Result::Failure);

            RET_IF_FALSE(getFuelGauge().enableTSENSE(false, false), Result::Failure);
            RET_IF_FALSE(getFuelGauge().setOperationMode(true), Result::Failure);
            RET_IF_FALSE(getFuelGauge().setInitialized(), Result::Failure);
            Log.Debug(TAG, "Fuel gauge initialized.");
        }
        else
        {
            Log.Debug(TAG, "Fuel gauge already initialized.");
        }

        return Result::Ok;
    }

    Result Mainboard::_udpateChargerADC()
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
            Log.Debug(TAG, "Updated charger ADC.");
        }
        return Result::Ok;
    }

    Result Mainboard::init(uint16_t capacity, BatteryType type)
    {
        _mutex.init();

        TRY_LOCK(_mutex);

        if (type == BatteryType::ICR18650_26H || type == BatteryType::UR18650ZY)
        {
            // Ignore set capacity and set 2600 mAh for both ICR18650_26H and UR18650ZY.
            capacity = 2600;
        }
        else
        {
            // Capacity should either be 0, in which case it indicates to the SDK that there is no battery expected
            // to be connected to the system; or within _minBatteryCapacity and _maxBatteryCapacity inclusive.
            RET_IF_FALSE(!capacity || (capacity >= _minBatteryCapacity && capacity <= LC709204F::MaxBatteryCapacity), Result::InvalidArg);
        }

        _initDone = false;

        _batteryCapacity = capacity;
        _batteryType = type;
        Log.Debug(TAG, "Battery capacity and type set to %d mAh, %d.", static_cast<int>(_batteryCapacity), static_cast<int>(_batteryType));
        // Set termination current to C / 10, or within limits of the IC.
        uint16_t minCurrent = BQ2562x::MinITERMCurrent;
        uint16_t maxCurrent = BQ2562x::MaxITERMCurrent;
        _terminationCurrent = static_cast<uint16_t>(_batteryCapacity / 10);
        _terminationCurrent = std::min(std::max(_terminationCurrent, minCurrent), maxCurrent);
        Log.Info(TAG, "Termination current set to %d mA.", _terminationCurrent);

        // On first boot VSQT, through EN_SQT, is always enabled. On wake from deep sleep try and maintain held state.
        RET_IF_FALSE(Chip.configureRTCPin(Pins::EN_SQT, SoC::PinMode::InputOutput), Result::Failure);
        _sqtEnabled = Chip.isFirstBoot() ? true : Chip.readRTCPin(Pins::EN_SQT);
        RET_IF_FALSE(Chip.setRTCPin(Pins::EN_SQT, _sqtEnabled), Result::Failure)
        Log.Debug(TAG, "VSQT detected as %d during initialization", _sqtEnabled);

        if (_sqtEnabled)
        {
            RET_IF_FALSE(_i2c.start(), Result::Failure);

            bool wdOn = true;
            RET_IF_FALSE(getCharger().getWD(wdOn), Result::Failure);

            if (wdOn) // watchdog enabled means that the initiatialization was not done previously
            {
                RET_IF_FALSE(getCharger().enableCharging(false), Result::Failure);
                RET_IF_FALSE(getCharger().setIINDPM(BQ2562x::MaxChargingCurrent), Result::Failure);
                RET_IF_FALSE(getCharger().enableTS(false), Result::Failure);
                RET_IF_FALSE(getCharger().setChargeCurrent(_defaultMaxChargingCurrent), Result::Failure);
                RET_IF_FALSE(getCharger().setBATFETDelay(BQ2562x::BATFETDelay::Delay20ms), Result::Failure);
                RET_IF_FALSE(getCharger().enableWVBUS(true), Result::Failure);
                RET_IF_FALSE(getCharger().setTopOff(BQ2562x::TopOffTimer::Timer17Min), Result::Failure);
                RET_IF_FALSE(getCharger().setIbatPk(BQ2562x::IbatPkLimit::Limit3A), Result::Failure);
                RET_IF_FALSE(getCharger().setTH456(BQ2562x::TH456Setting::TH4_35_TH5_40_TH6_50), Result::Failure);
                RET_IF_FALSE(getCharger().setTempIset(BQ2562x::TempPoint::Precool, BQ2562x::TempIset::Ichg40), Result::Failure);
                RET_IF_FALSE(getCharger().setTempIset(BQ2562x::TempPoint::Prewarm, BQ2562x::TempIset::Ichg40), Result::Failure);
                RET_IF_FALSE(getCharger().setTempIset(BQ2562x::TempPoint::Cool, BQ2562x::TempIset::Ichg20), Result::Failure);
                RET_IF_FALSE(getCharger().setTempIset(BQ2562x::TempPoint::Warm, BQ2562x::TempIset::Ichg20), Result::Failure);
                RET_IF_FALSE(getCharger().enableInterrupts(false), Result::Failure);
                if (_batteryCapacity)
                {
                    RET_IF_FALSE(getCharger().setITERM(_terminationCurrent), Result::Failure);
                }
                // Disable the charger watchdog to keep the charger in host mode and to
                // keep some registers from resetting to their POR values.
                RET_IF_FALSE(getCharger().setWD(BQ2562x::WatchdogTimer::Disabled), Result::Failure);
                Log.Debug(TAG, "Charger IC initialized.");
            }
            else
            {
                Log.Debug(TAG, "Charger IC already initialized.");
            }

            // If battery capacity is not 0, initialize the fuel gauge. This can fail if during
            // startup no battery is connected, therefore failures are not checked here. Fuel guage
            // initialization attempts will be made later, during calls to member functions that
            // talk to the fuel gauge.
            if (_batteryCapacity)
            {
                _initFuelGauge();
            }
        }

        // Initialize the rest of the RTC/digital pins managed by the SDK.
        RET_IF_FALSE(Chip.configureRTCPin(Pins::EN0, SoC::PinMode::InputOutputOpenDrain), Result::Failure);
        bool _enHigh = Chip.isFirstBoot() ? true : Chip.readRTCPin(Pins::EN0);
        RET_IF_FALSE(Chip.setRTCPin(Pins::EN0, _enHigh), Result::Failure)
        Log.Debug(TAG, "EN detected as %d during initialization", _enHigh);

        RET_IF_FALSE(Chip.configureRTCPin(Pins::EN_3V3, SoC::PinMode::InputOutput), Result::Failure);
        bool _3V3Enabled = Chip.isFirstBoot() ? true :  Chip.readRTCPin(Pins::EN_3V3);
        RET_IF_FALSE(Chip.setRTCPin(Pins::EN_3V3, _3V3Enabled), Result::Failure)
        Log.Debug(TAG, "3V3 detected as %d during initialization.", _3V3Enabled);

        RET_IF_FALSE(Chip.configureDigitalPin(Pins::PG, SoC::PinMode::Input), Result::Failure);

        Chip.init();

        _initDone = true;

        Log.Debug(TAG, "Initialization done.");

        return Result::Ok;
    }

    Result Mainboard::setEN(bool value)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(Chip.setRTCPin(Pins::EN0, value), Result::Failure);
        Log.Debug(TAG, "EN set to: %d.", value);
        return Result::Ok;
    }

    Result Mainboard::enable3V3(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(Chip.setRTCPin(Pins::EN_3V3, enable), Result::Failure);
        Log.Debug(TAG, "3V3 set to: %d.", enable);
        return Result::Ok;
    }

    Result Mainboard::enableVSQT(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(Chip.setRTCPin(Pins::EN_SQT, enable), Result::Failure);
        RET_IF_FALSE(enable ? (_sqtEnabled || _i2c.start()) : !_sqtEnabled || _i2c.end(), Result::Failure);
        _sqtEnabled = enable;
        Log.Debug(TAG, "VSQT set to: %d.", _sqtEnabled);
        return Result::Ok;
    }

    Result Mainboard::getSupplyVoltage(uint16_t &voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getVBUS(voltage), Result::Failure);
        Log.Debug(TAG, "Measured supply voltage: %d mV.", voltage);
        return Result::Ok;
    }

    Result Mainboard::getSupplyCurrent(int16_t &current)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getIBUS(current), Result::Failure);
        Log.Debug(TAG, "Measured supply current: %d mA.", current);
        return Result::Ok;
    }

    Result Mainboard::checkSupplyGood(bool &good)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        good = (Chip.readDigitalPin(Pins::PG) == 0);
        Log.Debug(TAG, "Check power supply good: %d.", good);
        return Result::Ok;
    }

    Result Mainboard::setSupplyMaintainVoltage(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(voltage >= _minSupplyMaintainVoltage && voltage <= BQ2562x::MaxVINDPMVoltage, Result::InvalidArg);
        RET_IF_FALSE(getCharger().setVINDPM(voltage), Result::Failure);
        Log.Debug(TAG, "Maintain supply voltage set to: %d mV.", voltage);
        return Result::Ok;
    }

    Result Mainboard::enterShipMode()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::ShipMode), Result::Failure);
        // If this executes, then charger did not enter ship mode. Return to normal operation
        // and return failure status.
        vTaskDelay(pdMS_TO_TICKS(_batfetCtrlWaitTime));
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::Normal), Result::Failure);
        Log.Debug(TAG, "Failed to enter ship mode.");
        return Result::Failure;
    }

    Result Mainboard::enterShutdownMode()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::ShutdownMode), Result::Failure);
        // If this executes, then charger did not enter shutdown mode. According to the datasheet,
        // charger automatically returns to normal mode, so just return failure.
        vTaskDelay(pdMS_TO_TICKS(_batfetCtrlWaitTime));
        Log.Debug(TAG, "Failed to enter shutdown mode.");
        return Result::Failure;
    }

    Result Mainboard::doPowerCycle()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::SystemPowerReset), Result::Failure);
        // If this executes, then charger did not perform power cycle.
        vTaskDelay(pdMS_TO_TICKS(_batfetCtrlWaitTime));
        Log.Debug(TAG, "Failed to do power cycle.");
        return Result::Failure;
    }

    Result Mainboard::enableBatteryCharging(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(getCharger().enableCharging(enable), Result::Failure);
        Log.Debug(TAG, "Charging set to: %d.", enable);
        return Result::Ok;
    }

    Result Mainboard::setBatteryChargingMaxCurrent(uint16_t current)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(current >= _minBatteryCapacity && current <= BQ2562x::MaxChargingCurrent, Result::InvalidArg);
        RET_IF_FALSE(getCharger().setChargeCurrent(current), Result::Failure);
        Log.Debug(TAG, "Max charging current set to: %d mA.", current);
        return Result::Ok;
    }

    Result Mainboard::enableBatteryTempSense(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(getCharger().enableTS(enable), Result::Failure);
        RET_IF_FALSE(getCharger().enableInterrupt(BQ2562x::Interrupt::TS, enable), Result::Failure);
        Log.Debug(TAG, "Temperature sense set to: %d.", enable);
        return Result::Ok;
    }

    Result Mainboard::enableBatteryFuelGauge(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        if (enable)
        {
            // Do also initialization here, since it is possible that the system initialized
            // with battery present, then battery is disconected, then battery is reconnected,
            // the finally this function is called.
            RET_IF_ERR(_initFuelGauge());
        }
        RET_IF_FALSE(getFuelGauge().setOperationMode(enable), Result::Failure);
        Log.Debug(TAG, "Fuel gauge set to: %d.", enable);
        return Result::Ok;
    }

    Result Mainboard::getBatteryVoltage(uint16_t &voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        // If fuel gauge is available, use the reading from it.
        if (!(_isFuelGaugeEnabled() && _initFuelGauge() == Result::Ok && getFuelGauge().getCellVoltage(voltage)))
        {
            RET_IF_ERR(_udpateChargerADC());
            RET_IF_FALSE(getCharger().getVBAT(voltage), Result::Failure);
        }
        return Result::Ok;
    }

    Result Mainboard::getBatteryCurrent(int16_t &current)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getIBAT(current), Result::Failure);
        Log.Debug(TAG, "Measured battery current: %d mA.", current);
        return Result::Ok;
    }

    Result Mainboard::getBatteryCharge(uint8_t &percent)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getRSOC(percent), Result::Failure);
        Log.Debug(TAG, "Estimated battery charge: %d %%.", percent);
        return Result::Ok;
    }

    Result Mainboard::getBatteryHealth(uint8_t &percent)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getSOH(percent), Result::Failure);
        Log.Debug(TAG, "Estimated battery health: %d %%.", percent);
        return Result::Ok;
    }

    Result Mainboard::getBatteryCycles(uint16_t &cycles)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getCycles(cycles), Result::Failure);
        Log.Debug(TAG, "Estimated battery cycles: %d.", cycles);
        return Result::Ok;
    }

    Result Mainboard::getBatteryTimeLeft(int &minutes)
    {
        TRY_LOCK(_mutex);

        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _isFuelGaugeEnabled(), Result::InvalidState);
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
            Log.Debug(TAG, "Estimated battery time left (%s): %d mins.", discharging ? "discharging" : "charging", abs(minutes));
            return Result::Ok;
        }

        Log.Debug(TAG, "No estimate for %s can be provided yet.", discharging ? "time-to-empty" : "time-to-full");
        return Result::NotReady;
    }

    Result Mainboard::getBatteryTemperature(float &celsius)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);

        // Temperature sensing must be enabled using enableBatteryTempSense(true), only
        // then can the battery temperature be measured.
        bool enabled = false;
        RET_IF_FALSE(getCharger().getTSEnabled(enabled) && enabled, Result::InvalidState);

        float bias = 0;
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getTSBias(bias), Result::Failure);
        // Map bias to temperature given 103AT thermistor with fitted curve.
        celsius = (-1866.96172 * powf(bias, 4)) + (3169.31754 * powf(bias, 3)) - (1849.96775 * powf(bias, 2)) + (276.6656 * bias) + 81.98758;
        Log.Debug(TAG, "Measured battery temperature: %f °C.", celsius);
        return Result::Ok;
    }

    Result Mainboard::setBatteryLowVoltageAlarm(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE((voltage >= LC709204F::MinVoltageAlarm && voltage <= LC709204F::MaxVoltageAlarm) || voltage == 0, Result::InvalidArg);
        RET_IF_FALSE(getFuelGauge().setLowVoltageAlarm(voltage), Result::Failure);
        if (voltage == 0)
        {
            RET_IF_FALSE(getFuelGauge().clearLowVoltageAlarm(), Result::Failure);
        }
        Log.Debug(TAG, "Low battery voltage alarm set to: %d mV.", voltage);
        return Result::Ok;
    };

    Result Mainboard::setBatteryHighVoltageAlarm(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE((voltage >= LC709204F::MinVoltageAlarm && voltage <= LC709204F::MaxVoltageAlarm) || voltage == 0, Result::InvalidArg);
        RET_IF_FALSE(getFuelGauge().setHighVoltageAlarm(voltage), Result::Failure);
        if (voltage == 0)
        {
            RET_IF_FALSE(getFuelGauge().clearHighVoltageAlarm(), Result::Failure);
        }
        Log.Debug(TAG, "High battery voltage alarm set to: %d mV.", voltage);
        return Result::Ok;
    };

    Result Mainboard::setBatteryLowChargeAlarm(uint8_t percent)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_sqtEnabled, Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity && _isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(percent <= 100, Result::InvalidArg);
        RET_IF_FALSE(getFuelGauge().setLowRSOCAlarm(percent), Result::Failure);
        if (percent == 0)
        {
            RET_IF_FALSE(getFuelGauge().clearLowRSOCAlarm(), Result::Failure);
        }
        Log.Debug(TAG, "Low charge alarm set to: %d %%.", (int)percent);
        return Result::Ok;
    };

    /*static*/ Mainboard &Mainboard::get()
    {
        static Mainboard board;
        return board;
    }
}
