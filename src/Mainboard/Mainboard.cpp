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

#include <algorithm>
#include <climits>
#include <cstdint>
#include <cstring>
#include <math.h>

#include <soc/reset_reasons.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "Mainboard.h"

namespace PowerFeather
{
    static_assert(CHAR_BIT == 8, "Unsupported architecture.");

    static const char *TAG = "PowerFeather::Mainboard";

    /*extern*/ Mainboard &Board = Mainboard::get();

    namespace
    {
        static uint32_t fnv1aHash(const void *data, size_t size)
        {
            const uint8_t *bytes = static_cast<const uint8_t *>(data);
            uint32_t hash = 2166136261u;
            for (size_t i = 0; i < size; ++i)
            {
                hash ^= bytes[i];
                hash *= 16777619u;
            }
            return hash;
        }

        template <typename Gauge>
        struct FuelGaugeProfileHelper
        {
            static void setProfile(Gauge &, const MAX17260::Model *) {}
        };

        template <>
        struct FuelGaugeProfileHelper<MAX17260>
        {
            static void setProfile(MAX17260 &gauge, const MAX17260::Model *profile)
            {
                if (profile)
                {
                    gauge.setProfile(*profile);
                }
            }
        };

        static uint16_t designCapToMah(uint16_t designCapRaw, uint16_t maxMah)
        {
            // MAX17260 profile DesignCap is a normalized 16-bit value.
            // getBatteryCapacityRange() already reflects the active gauge's mAh limits, including
            // hardware parameters (for example, RSENSE). Scale the normalized value to mAh.
            uint32_t scaled = (static_cast<uint32_t>(designCapRaw) * maxMah + 0x7FFFu) / 0xFFFFu;
            return static_cast<uint16_t>(std::min<uint32_t>(scaled, maxMah));
        }
    }

    #define LOG_FAIL(r)                 ESP_LOGD(TAG, "Unexpected result %d on %s:%d.", (r), __FUNCTION__, __LINE__)
    #define RET_IF_ERR(f)               { Result r = (f); if (r != Result::Ok) { LOG_FAIL(1); return r; } }
    #define RET_IF_NOK(f)               { esp_err_t r = (f); if (r != ESP_OK) { LOG_FAIL(r); return false; } }
    #define RET_IF_FALSE(f, r)          { if ((f) == false) { LOG_FAIL(false); return (r); } }
    #define TRY_LOCK(m)                 Mutex::Lock m##Lock(m); RET_IF_FALSE(m##Lock.isLocked(), Result::LockFailed);

    static RTC_NOINIT_ATTR uint32_t first;
    static const uint32_t firstMagic = 0xdeadbeef;

    // Charger state the user has committed via public setters. Placed in RTC
    // slow memory so it survives deep sleep and can be re-applied to the chip
    // in _reapplyChargerConfig. Valid iff first == firstMagic (warm boot).
    struct PersistedChargerState
    {
        uint16_t chargingCurrentLimit;
        uint16_t vindpm;
        bool chargingEnabled;
        bool tsEnabled;
    };
    static RTC_NOINIT_ATTR PersistedChargerState persistedChargerState;

    struct PersistedFuelGaugeInitSignature
    {
        uint8_t source;
        uint16_t capacityMah;
        uint16_t terminationCurrentMa;
        uint16_t chargeVoltageMv;
        uint32_t profileHash;
        bool hasSignature;
    };
    static RTC_NOINIT_ATTR PersistedFuelGaugeInitSignature persistedFuelGaugeInitSignature;

    FuelGauge &Mainboard::getFuelGauge()
    {
        return _fuelGauge;
    }

    bool Mainboard::_isFuelGaugeEnabled()
    {
        bool enabled = false;
        getFuelGauge().getEnabled(enabled); // failure here means false is returned
        return enabled;
    }

    Result Mainboard::_initFuelGauge()
    {
        FuelGauge &gauge = getFuelGauge();
        FuelGauge::InitConfig config;
        FuelGaugeInitSignature signature;

        if (_usesProfile)
        {
            config.source = FuelGauge::InitSource::Profile_Max17260;
            config.data.profile.model = nullptr;
            signature.source = config.source;
            signature.profileHash = _profileHash;
        }
        else
        {
            config.data.capacity.capacityMah = _batteryCapacity;
            config.data.capacity.terminationCurrentMa = _terminationCurrent;
            config.data.capacity.chargeVoltageMv = _chargeVoltageMv;
            signature.capacityMah = _batteryCapacity;
            signature.terminationCurrentMa = _terminationCurrent;
            signature.chargeVoltageMv = _chargeVoltageMv;
            switch (_batteryType)
            {
                case BatteryType::Generic_3V7:
                    config.source = FuelGauge::InitSource::Generic_3V7;
                    break;
                case BatteryType::ICR18650_26H:
                    config.source = FuelGauge::InitSource::ICR18650_26H;
                    break;
                case BatteryType::UR18650ZY:
                    config.source = FuelGauge::InitSource::UR18650ZY;
                    break;
                case BatteryType::Generic_LFP:
                    config.source = FuelGauge::InitSource::Generic_LFP;
                    break;
            }
            signature.source = config.source;
        }

#if defined(CONFIG_ESP32S3_POWERFEATHER_V2) || defined(POWERFEATHER_BOARD_V2)
        config.tsenseEnabled = !_fuelGaugeUsingExternalTemp;
#endif

        bool forceReinit = _hasFuelGaugeInitSignature &&
                           (signature.source != _lastFuelGaugeInitSignature.source ||
                            signature.capacityMah != _lastFuelGaugeInitSignature.capacityMah ||
                            signature.terminationCurrentMa != _lastFuelGaugeInitSignature.terminationCurrentMa ||
                            signature.chargeVoltageMv != _lastFuelGaugeInitSignature.chargeVoltageMv ||
                            signature.profileHash != _lastFuelGaugeInitSignature.profileHash);
        RET_IF_FALSE(gauge.init(config, forceReinit), Result::Failure);
        _lastFuelGaugeInitSignature = signature;
        _hasFuelGaugeInitSignature = true;
        persistedFuelGaugeInitSignature.source = static_cast<uint8_t>(signature.source);
        persistedFuelGaugeInitSignature.capacityMah = signature.capacityMah;
        persistedFuelGaugeInitSignature.terminationCurrentMa = signature.terminationCurrentMa;
        persistedFuelGaugeInitSignature.chargeVoltageMv = signature.chargeVoltageMv;
        persistedFuelGaugeInitSignature.profileHash = signature.profileHash;
        persistedFuelGaugeInitSignature.hasSignature = true;
        ESP_LOGD(TAG, "Fuel gauge init complete (%s).", gauge.getName());

        return Result::Ok;
    }

    bool Mainboard::_isFirst()
    {
        // If the RTC is domain is shutdown, consider next boot as first boot.
        bool isFirst = (first != firstMagic);
        ESP_LOGD(TAG, "Check if first boot: %d.", isFirst);
        return isFirst;
    }

    bool Mainboard::_canAccessPowerI2C() const
    {
#if defined(CONFIG_ESP32S3_POWERFEATHER_V2) || defined(POWERFEATHER_BOARD_V2)
        return true;
#else
        return _sqtEnabled;
#endif
    }


    bool Mainboard::_initInternalDigitalPin(gpio_num_t pin, gpio_mode_t mode)
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

    bool Mainboard::_initInternalRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode)
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


    bool Mainboard::_setRTCPin(gpio_num_t pin, bool value)
    {
        // Disable pin hold, set the level, and re-enable pin hold.
        rtc_gpio_hold_dis(pin);
        rtc_gpio_set_level(pin, value);
        rtc_gpio_hold_en(pin);
        ESP_LOGD(TAG, "Set RTC pin %d to %d.", pin, value);
        return true;
    }

    Result Mainboard::_udpateChargerADC()
    {
        RET_IF_ERR(_reapplyChargerConfig());

        uint32_t now = esp_timer_get_time() / 1000;
        // Since updating ADC values take a long time, only update it again after _chargerADCWaitTime has elapsed.
        if (_chargerADCTime == 0 || (now - _chargerADCTime) >= _chargerADCWaitTime)
        {
            bool done = false;
            RET_IF_FALSE(getCharger().setupADC(true, BQ2562x::ADCRate::Oneshot, BQ2562x::ADCSampling::Bits_10), Result::Failure);

            // Release lock during long ADC wait to preserve parallel-task responsiveness,
            // then block indefinitely on relock to guarantee the caller's RAII contract is honored.
            _mutex.unlock();
            vTaskDelay(pdMS_TO_TICKS(_chargerADCWaitTime));
            _mutex.lockBlocking();

            RET_IF_FALSE(getCharger().getADCDone(done) && done, Result::Failure);
            _chargerADCTime = now;
            ESP_LOGD(TAG, "Updated charger ADC.");
        }
        return Result::Ok;
    }

    Result Mainboard::_reapplyChargerConfig()
    {
        bool wdOn = true;
        RET_IF_FALSE(getCharger().getWD(wdOn), Result::Failure);

        if (wdOn)
        {
            ESP_LOGW(TAG, "Charger watchdog enabled, re-applying configuration.");
            RET_IF_FALSE(getCharger().enableCharging(false), Result::Failure);
            RET_IF_FALSE(getCharger().setIINDPM(BQ2562x::MaxChargingCurrent), Result::Failure);
            RET_IF_FALSE(getCharger().enableTS(_tsEnabled), Result::Failure);
            RET_IF_FALSE(getCharger().setChargeCurrentLimit(_chargingCurrentLimit), Result::Failure);
            RET_IF_FALSE(getCharger().setBATFETDelay(BQ2562x::BATFETDelay::Delay20ms), Result::Failure);
            RET_IF_FALSE(getCharger().enableWVBUS(true), Result::Failure);
            RET_IF_FALSE(getCharger().setTopOff(BQ2562x::TopOffTimer::Timer17Min), Result::Failure);
            RET_IF_FALSE(getCharger().setIbatPk(BQ2562x::IbatPkLimit::Limit3A), Result::Failure);
            RET_IF_FALSE(getCharger().setTH456(BQ2562x::TH456Setting::TH4_35_TH5_40_TH6_50), Result::Failure);
            RET_IF_FALSE(getCharger().setTempIset(BQ2562x::TempPoint::Precool, BQ2562x::TempIset::Ichg40), Result::Failure);
            RET_IF_FALSE(getCharger().setTempIset(BQ2562x::TempPoint::Prewarm, BQ2562x::TempIset::Ichg40), Result::Failure);
            RET_IF_FALSE(getCharger().setTempIset(BQ2562x::TempPoint::Cool, BQ2562x::TempIset::Ichg20), Result::Failure);
            RET_IF_FALSE(getCharger().setTempIset(BQ2562x::TempPoint::Warm, BQ2562x::TempIset::Ichg20), Result::Failure);
            // Mask all interrupts first so POR-default unmasks don't leak through,
            // then selectively re-enable only the ones the SDK actually uses.
            RET_IF_FALSE(getCharger().enableInterrupts(false), Result::Failure);
            if (_tsEnabled)
            {
                RET_IF_FALSE(getCharger().enableInterrupt(BQ2562x::Interrupt::TS, true), Result::Failure);
            }
            if (_batteryCapacity)
            {
                RET_IF_FALSE(getCharger().setIINDPM(BQ2562x::MaxIINDPMCurrent), Result::Failure);
                RET_IF_FALSE(getCharger().setITERM(_terminationCurrent), Result::Failure);
            }
            RET_IF_FALSE(getCharger().setChargeVoltageLimit(_chargeVoltageMv), Result::Failure);
            RET_IF_FALSE(getCharger().setVINDPM(_vindpm), Result::Failure);
            RET_IF_FALSE(getCharger().setWD(BQ2562x::WatchdogTimer::Disabled), Result::Failure);
            RET_IF_FALSE(getCharger().enableCharging(_chargingEnabled), Result::Failure);
        }

        return Result::Ok;
    }

    Result Mainboard::init(uint16_t capacity, BatteryType type)
    {
        RET_IF_FALSE(capacity, Result::InvalidArg);
        return _initInternal(capacity, type, nullptr);
    }

    Result Mainboard::init()
    {
        return _initInternal(0, BatteryType::Generic_3V7, nullptr);
    }

    Result Mainboard::init(const MAX17260::Model &profile)
    {
        uint16_t capacity = _capacityFromProfile(profile);
        RET_IF_FALSE(capacity, Result::InvalidArg);
        // Profile init must explicitly provide the charger CV target.
        static constexpr uint16_t minProfileChargeVoltageMv = 3500;
        static constexpr uint16_t maxProfileChargeVoltageMv = 4800;
        RET_IF_FALSE(profile.chargeVoltageMv >= minProfileChargeVoltageMv &&
                     profile.chargeVoltageMv <= maxProfileChargeVoltageMv, Result::InvalidArg);
        return _initInternal(capacity, BatteryType::Generic_3V7, &profile);
    }

    uint16_t Mainboard::_capacityFromProfile(const MAX17260::Model &profile) const
    {
        if (!FuelGaugeImpl::SupportsProfile)
        {
            return 0;
        }

        if (profile.designCap == 0)
        {
            return 0;
        }

        uint16_t minMah = 0;
        uint16_t maxMah = 0;
        _fuelGauge.getBatteryCapacityRange(minMah, maxMah);
        if (maxMah == 0)
        {
            return 0;
        }

        return designCapToMah(profile.designCap, maxMah);
    }

    Result Mainboard::_initInternal(uint16_t capacity, BatteryType type, const MAX17260::Model *profile)
    {
        _mutex.init();

        TRY_LOCK(_mutex);

        if (type == BatteryType::ICR18650_26H || type == BatteryType::UR18650ZY)
        {
            // Ignore set capacity and set 2600 mAh for both ICR18650_26H and UR18650ZY.
            capacity = 2600;
        }

        _initDone = false;

        const bool useProfile = (profile != nullptr);
        if (type == BatteryType::Generic_LFP && !FuelGaugeImpl::SupportsLfp)
        {
            return Result::InvalidArg;
        }
        if (useProfile && !FuelGaugeImpl::SupportsProfile)
        {
            return Result::InvalidArg;
        }
        RET_IF_FALSE(!useProfile || capacity, Result::InvalidArg);

        uint16_t minCapacity = 0;
        uint16_t maxCapacity = 0;
        _fuelGauge.getBatteryCapacityRange(minCapacity, maxCapacity);
        // Capacity should either be 0 (from init()) to indicate no battery is expected,
        // or within range inclusive.
        RET_IF_FALSE(!capacity || (capacity >= minCapacity && capacity <= maxCapacity), Result::InvalidArg);

        _batteryCapacity = capacity;
        _batteryType = type;
        _usesProfile = useProfile;
        _profileHash = useProfile ? fnv1aHash(profile, sizeof(*profile)) : 0;
        _chargingEnabled = false;
        _chargingCurrentLimit = _defaultMaxChargingCurrent;
        _tsEnabled = false;
        _vindpm = _minSupplyMaintainVoltage;
        if (!_isFirst())
        {
            // Warm boot: the chip may have retained the previous session's config
            // (wdOn == false) or may have POR'd (wdOn == true). Either way,
            // _reapplyChargerConfig should restore what the user last set, not
            // reset to defaults — so rehydrate from RTC-persisted state.
            _chargingEnabled = persistedChargerState.chargingEnabled;
            _chargingCurrentLimit = persistedChargerState.chargingCurrentLimit;
            _tsEnabled = persistedChargerState.tsEnabled;
            _vindpm = persistedChargerState.vindpm;
            _lastFuelGaugeInitSignature.source = static_cast<FuelGauge::InitSource>(persistedFuelGaugeInitSignature.source);
            _lastFuelGaugeInitSignature.capacityMah = persistedFuelGaugeInitSignature.capacityMah;
            _lastFuelGaugeInitSignature.terminationCurrentMa = persistedFuelGaugeInitSignature.terminationCurrentMa;
            _lastFuelGaugeInitSignature.chargeVoltageMv = persistedFuelGaugeInitSignature.chargeVoltageMv;
            _lastFuelGaugeInitSignature.profileHash = persistedFuelGaugeInitSignature.profileHash;
            _hasFuelGaugeInitSignature = persistedFuelGaugeInitSignature.hasSignature;
        }
        else
        {
            // Cold boot: RTC_NOINIT_ATTR leaves persistedChargerState undefined.
            // Seed it with the defaults above so that a warm boot prior to any
            // setter call rehydrates known-good values instead of RTC garbage.
            persistedChargerState.chargingEnabled = _chargingEnabled;
            persistedChargerState.chargingCurrentLimit = _chargingCurrentLimit;
            persistedChargerState.tsEnabled = _tsEnabled;
            persistedChargerState.vindpm = _vindpm;
            persistedFuelGaugeInitSignature.source = static_cast<uint8_t>(FuelGauge::InitSource::Generic_3V7);
            persistedFuelGaugeInitSignature.capacityMah = 0;
            persistedFuelGaugeInitSignature.terminationCurrentMa = 0;
            persistedFuelGaugeInitSignature.chargeVoltageMv = 0;
            persistedFuelGaugeInitSignature.profileHash = 0;
            persistedFuelGaugeInitSignature.hasSignature = false;
            _lastFuelGaugeInitSignature = {};
            _hasFuelGaugeInitSignature = false;
        }
#if defined(CONFIG_ESP32S3_POWERFEATHER_V2) || defined(POWERFEATHER_BOARD_V2)
        _fuelGaugeUsingExternalTemp = false;
#endif
        ESP_LOGD(TAG, "Battery capacity and type set to %d mAh, %d.", static_cast<int>(_batteryCapacity), static_cast<int>(_batteryType));
        if (useProfile)
        {
            FuelGaugeProfileHelper<FuelGaugeImpl>::setProfile(_fuelGauge, profile);
        }
        // Set termination current to C / 10, or within limits of the IC.
        uint16_t minCurrent = BQ2562x::MinITERMCurrent;
        uint16_t maxCurrent = BQ2562x::MaxITERMCurrent;
        _terminationCurrent = static_cast<uint16_t>(_batteryCapacity / 10);
        _terminationCurrent = std::min(std::max(_terminationCurrent, minCurrent), maxCurrent);
        ESP_LOGI(TAG, "Termination current set to %d mA.", _terminationCurrent);

        uint16_t chargeVoltageMv = 4200;
        if (useProfile && profile->chargeVoltageMv)
        {
            chargeVoltageMv = profile->chargeVoltageMv;
        }
        if (_batteryType == BatteryType::Generic_LFP)
        {
            chargeVoltageMv = 3600;
        }
        _chargeVoltageMv = chargeVoltageMv;

        // On first boot VSQT, through EN_SQT, is always enabled. On wake from deep sleep try and maintain held state.
        RET_IF_FALSE(_initInternalRTCPin(Pin::EN_SQT, RTC_GPIO_MODE_INPUT_OUTPUT), Result::Failure);
        bool sqtEnabled = _isFirst() ? true : rtc_gpio_get_level(Pin::EN_SQT);
        RET_IF_FALSE(_setRTCPin(Pin::EN_SQT, sqtEnabled), Result::Failure)
        ESP_LOGD(TAG, "VSQT detected as %d during initialization", sqtEnabled);
#if !defined(CONFIG_ESP32S3_POWERFEATHER_V2) && !defined(POWERFEATHER_BOARD_V2)
        _sqtEnabled = sqtEnabled;
#endif

        if (_canAccessPowerI2C())
        {
            RET_IF_FALSE(_i2c.start(), Result::Failure);

            uint8_t pi = 0;
            RET_IF_FALSE(getCharger().getPartInformation(pi), Result::Failure);
            uint8_t pn = (pi >> 3) & 0x07;
            if (pn != BQ2562x::Charger_PN_BQ25622 && pn != BQ2562x::Charger_PN_BQ25628)
            {
                ESP_LOGE(TAG, "Unsupported charger part ID: 0x%02x (expected PN: 0x%02x or 0x%02x)", pi, BQ2562x::Charger_PN_BQ25622, BQ2562x::Charger_PN_BQ25628);
                return Result::Failure;
            }

            // _reapplyChargerConfig is gated on the charger watchdog (POR indicator).
            // On wdOn == true (first init, or chip POR since last init), it applies the
            // full configuration using the software state set above — which on a warm
            // boot is the user's previous-session values rehydrated from RTC, not the
            // hardcoded defaults. On wdOn == false (chip retained state), it's a no-op
            // and the redundant setChargeVoltageLimit below is the belt-and-braces write.
            RET_IF_ERR(_reapplyChargerConfig());

            // Enforce charge voltage target on every init path, including the wdOn==false
            // path (chip retained state across sleep but we still want to rewrite VREG
            // in case the BatteryType chemistry selection changed since last session).
            RET_IF_FALSE(getCharger().setChargeVoltageLimit(_chargeVoltageMv), Result::Failure);

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
        RET_IF_FALSE(_initInternalRTCPin(Pin::EN0, RTC_GPIO_MODE_INPUT_OUTPUT_OD), Result::Failure);
        bool _enHigh = _isFirst() ? true : rtc_gpio_get_level(Pin::EN0);
        RET_IF_FALSE(_setRTCPin(Pin::EN0, _enHigh), Result::Failure)
        ESP_LOGD(TAG, "EN detected as %d during initialization", _enHigh);

        RET_IF_FALSE(_initInternalRTCPin(Pin::EN_3V3, RTC_GPIO_MODE_INPUT_OUTPUT), Result::Failure);
        bool _3V3Enabled = _isFirst() ? true :  rtc_gpio_get_level(Pin::EN_3V3);
        RET_IF_FALSE(_setRTCPin(Pin::EN_3V3, _3V3Enabled), Result::Failure)
        ESP_LOGD(TAG, "3V3 detected as %d during initialization.", _3V3Enabled);

        RET_IF_FALSE(_initInternalDigitalPin(Pin::PG, GPIO_MODE_INPUT), Result::Failure);

        first = firstMagic;
        _initDone = true;

        ESP_LOGD(TAG, "Initialization done.");

        return Result::Ok;
    }

    Result Mainboard::setEN(bool value)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_setRTCPin(Pin::EN0, value), Result::Failure);
        ESP_LOGD(TAG, "EN set to: %d.", value);
        return Result::Ok;
    }

    Result Mainboard::enable3V3(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_setRTCPin(Pin::EN_3V3, enable), Result::Failure);
        ESP_LOGD(TAG, "3V3 set to: %d.", enable);
        return Result::Ok;
    }

    Result Mainboard::enableVSQT(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_setRTCPin(Pin::EN_SQT, enable), Result::Failure);
#if defined(CONFIG_ESP32S3_POWERFEATHER_V2) || defined(POWERFEATHER_BOARD_V2)
        // On V2, power-management I2C remains usable even with VSQT disabled.
        ESP_LOGD(TAG, "VSQT set to: %d.", enable);
#else
        if (enable && !_sqtEnabled)
        {
            RET_IF_FALSE(_i2c.start(), Result::Failure);
            RET_IF_ERR(_reapplyChargerConfig());
            RET_IF_FALSE(getCharger().setChargeVoltageLimit(_chargeVoltageMv), Result::Failure);
            _chargerADCTime = 0;
            if (_batteryCapacity)
            {
                _initFuelGauge();
            }
        }
        else if (!enable && _sqtEnabled)
        {
            RET_IF_FALSE(_i2c.end(), Result::Failure);
        }
        _sqtEnabled = enable;
        ESP_LOGD(TAG, "VSQT set to: %d.", _sqtEnabled);
#endif
        return Result::Ok;
    }

    Result Mainboard::enableSTAT(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(getCharger().enableSTAT(enable), Result::Failure);
        ESP_LOGD(TAG, "STAT LED %s", enable ? "enabled" : "disabled");
        return Result::Ok;
    }

    Result Mainboard::getSupplyVoltage(uint16_t &voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getVBUS(voltage), Result::Failure);
        ESP_LOGD(TAG, "Measured supply voltage: %d mV.", voltage);
        return Result::Ok;
    }

    Result Mainboard::getSupplyCurrent(int16_t &current)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getIBUS(current), Result::Failure);
        ESP_LOGD(TAG, "Measured supply current: %d mA.", current);
        return Result::Ok;
    }

    Result Mainboard::checkSupplyGood(bool &good)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        good = (gpio_get_level(Pin::PG) == 0);
        ESP_LOGD(TAG, "Check power supply good: %d.", good);
        return Result::Ok;
    }

    Result Mainboard::setSupplyMaintainVoltage(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(voltage >= _minSupplyMaintainVoltage && voltage <= BQ2562x::MaxVINDPMVoltage, Result::InvalidArg);
        RET_IF_ERR(_reapplyChargerConfig());
        RET_IF_FALSE(getCharger().setVINDPM(voltage), Result::Failure);
        _vindpm = voltage;
        persistedChargerState.vindpm = voltage;
        ESP_LOGD(TAG, "Maintain supply voltage set to: %d mV.", voltage);
        return Result::Ok;
    }

    Result Mainboard::enterShipMode()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::ShipMode), Result::Failure);
        // If this executes, then charger did not enter ship mode. Return to normal operation
        // and return failure status.
        vTaskDelay(pdMS_TO_TICKS(_batfetCtrlWaitTime));
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::Normal), Result::Failure);
        ESP_LOGD(TAG, "Failed to enter ship mode.");
        return Result::Failure;
    }

    Result Mainboard::enterShutdownMode()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::ShutdownMode), Result::Failure);
        // If this executes, then charger did not enter shutdown mode. According to the datasheet,
        // charger automatically returns to normal mode, so just return failure.
        vTaskDelay(pdMS_TO_TICKS(_batfetCtrlWaitTime));
        ESP_LOGD(TAG, "Failed to enter shutdown mode.");
        return Result::Failure;
    }

    Result Mainboard::doPowerCycle()
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(getCharger().setBATFETControl(BQ2562x::BATFETControl::SystemPowerReset), Result::Failure);
        // If this executes, then charger did not perform power cycle.
        vTaskDelay(pdMS_TO_TICKS(_batfetCtrlWaitTime));
        ESP_LOGD(TAG, "Failed to do power cycle.");
        return Result::Failure;
    }

    Result Mainboard::enableBatteryCharging(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_ERR(_reapplyChargerConfig());
        RET_IF_FALSE(getCharger().enableCharging(enable), Result::Failure);
        _chargingEnabled = enable;
        persistedChargerState.chargingEnabled = enable;
        ESP_LOGD(TAG, "Charging set to: %d.", enable);
        return Result::Ok;
    }

    Result Mainboard::setBatteryChargingMaxCurrent(uint16_t current)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_ERR(_reapplyChargerConfig());
        RET_IF_FALSE(current >= BQ2562x::MinChargingCurrent && current <= BQ2562x::MaxChargingCurrent, Result::InvalidArg);
        RET_IF_FALSE(getCharger().setChargeCurrentLimit(current), Result::Failure);
        _chargingCurrentLimit = current;
        persistedChargerState.chargingCurrentLimit = current;
        ESP_LOGD(TAG, "Max charging current set to: %d mA.", current);
        return Result::Ok;
    }

    Result Mainboard::enableBatteryTempSense(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_ERR(_reapplyChargerConfig());
        RET_IF_FALSE(getCharger().enableTS(enable), Result::Failure);
        RET_IF_FALSE(getCharger().enableInterrupt(BQ2562x::Interrupt::TS, enable), Result::Failure);
        _tsEnabled = enable;
        persistedChargerState.tsEnabled = enable;
        ESP_LOGD(TAG, "Temperature sense set to: %d.", enable);
        return Result::Ok;
    }

    Result Mainboard::enableBatteryFuelGauge(bool enable)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        if (enable)
        {
            // Do also initialization here, since it is possible that the system initialized
            // with battery present, then battery is disconected, then battery is reconnected,
            // the finally this function is called.
            RET_IF_ERR(_initFuelGauge());
        }
        RET_IF_FALSE(getFuelGauge().setEnabled(enable), Result::Failure);
        ESP_LOGD(TAG, "Fuel gauge set to: %d.", enable);
        return Result::Ok;
    }

    Result Mainboard::getBatteryVoltage(uint16_t &voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        // If fuel gauge is available, use the reading from it.
        bool usedFuelGauge = false;
        if (_isFuelGaugeEnabled() && _initFuelGauge() == Result::Ok)
        {
            usedFuelGauge = getFuelGauge().getCellVoltage(voltage);
        }

        if (!usedFuelGauge)
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
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getIBAT(current), Result::Failure);
        ESP_LOGD(TAG, "Measured battery current from charger: %d mA.", current);
        return Result::Ok;
    }

    Result Mainboard::getBatteryCharge(uint8_t &percent)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(_isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getRSOC(percent), Result::Failure);
        ESP_LOGD(TAG, "Estimated battery charge: %d %%.", percent);
        return Result::Ok;
    }

    Result Mainboard::getBatteryHealth(uint8_t &percent)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(_isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getSOH(percent), Result::Failure);
        ESP_LOGD(TAG, "Estimated battery health: %d %%.", percent);
        return Result::Ok;
    }

    Result Mainboard::getBatteryCycles(uint16_t &cycles)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(_isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(getFuelGauge().getCycles(cycles), Result::Failure);
        ESP_LOGD(TAG, "Estimated battery cycles: %d.", cycles);
        return Result::Ok;
    }

    Result Mainboard::getBatteryTimeLeft(int &minutes)
    {
        TRY_LOCK(_mutex);

        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(_isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());

        int16_t ibat = 0;
        RET_IF_ERR(getBatteryCurrent(ibat));
        bool discharging = ibat <= 0;

        // Get the time-to-empty or time-to-full depending on battery is charging
        // or discharging.
        uint16_t mins = 0;
        RET_IF_FALSE(discharging ? getFuelGauge().getTimeToEmpty(mins) : getFuelGauge().getTimeToFull(mins), Result::Failure);

        if (mins != 0xFFFF) // check if already the required 10 % rise/drop in charge
        {
            minutes = mins * (discharging ? -1 : 1); // return negative amount of time for discharging
            ESP_LOGD(TAG, "Estimated battery time left (%s): %d mins.", discharging ? "discharging" : "charging", abs(minutes));
            return Result::Ok;
        }

        ESP_LOGD(TAG, "No estimate for %s can be provided yet.", discharging ? "time-to-empty" : "time-to-full");
        return Result::NotReady;
    }

    Result Mainboard::getBatteryTemperature(float &celsius)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);

        // Temperature sensing must be enabled using enableBatteryTempSense(true), only
        // then can the battery temperature be measured.
        bool enabled = false;
        RET_IF_FALSE(getCharger().getTSEnabled(enabled) && enabled, Result::InvalidState);

        float bias = 0;
        RET_IF_ERR(_udpateChargerADC());
        RET_IF_FALSE(getCharger().getTSBias(bias), Result::Failure);

        // Check if bias is within plausible range for 103AT thermistor.
        // Values outside [0.1, 0.8] likely indicate open/short circuit or missing thermistor.
        if (bias < 0.10f || bias > 0.80f)
        {
            ESP_LOGD(TAG, "Battery thermistor bias %f is out of plausible range.", bias);
            return Result::Failure;
        }

        // Map bias to temperature given 103AT thermistor with fitted curve.
        celsius = (-1866.96172 * powf(bias, 4)) + (3169.31754 * powf(bias, 3)) - (1849.96775 * powf(bias, 2)) + (276.6656 * bias) + 81.98758;
        ESP_LOGD(TAG, "Measured battery temperature: %f °C.", celsius);
        return Result::Ok;
    }

    Result Mainboard::setBatteryLowVoltageAlarm(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(_isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        uint16_t minAlarm = 0;
        uint16_t maxAlarm = 0;
        getFuelGauge().getVoltageAlarmRange(minAlarm, maxAlarm);
        RET_IF_FALSE((voltage == 0) || (voltage >= minAlarm && voltage <= maxAlarm), Result::InvalidArg);
        RET_IF_FALSE(getFuelGauge().setLowVoltageAlarm(voltage), Result::Failure);
        if (voltage == 0)
        {
            RET_IF_FALSE(getFuelGauge().clearLowVoltageAlarm(), Result::Failure);
        }
        ESP_LOGD(TAG, "Low battery voltage alarm set to: %d mV.", voltage);
        return Result::Ok;
    };

    Result Mainboard::setBatteryHighVoltageAlarm(uint16_t voltage)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(_isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        uint16_t minAlarm = 0;
        uint16_t maxAlarm = 0;
        getFuelGauge().getVoltageAlarmRange(minAlarm, maxAlarm);
        RET_IF_FALSE((voltage == 0) || (voltage >= minAlarm && voltage <= maxAlarm), Result::InvalidArg);
        RET_IF_FALSE(getFuelGauge().setHighVoltageAlarm(voltage), Result::Failure);
        if (voltage == 0)
        {
            RET_IF_FALSE(getFuelGauge().clearHighVoltageAlarm(), Result::Failure);
        }
        ESP_LOGD(TAG, "High battery voltage alarm set to: %d mV.", voltage);
        return Result::Ok;
    };

    Result Mainboard::setBatteryLowChargeAlarm(uint8_t percent)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(_isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        RET_IF_FALSE(percent <= 100, Result::InvalidArg);
        RET_IF_FALSE(getFuelGauge().setLowRSOCAlarm(percent), Result::Failure);
        if (percent == 0)
        {
            RET_IF_FALSE(getFuelGauge().clearLowRSOCAlarm(), Result::Failure);
        }
        ESP_LOGD(TAG, "Low charge alarm set to: %d %%.", (int)percent);
        return Result::Ok;
    };

    Result Mainboard::updateBatteryFuelGaugeTemp(float temperature)
    {
        TRY_LOCK(_mutex);
        RET_IF_FALSE(_initDone, Result::InvalidState);
        RET_IF_FALSE(_canAccessPowerI2C(), Result::InvalidState);
        RET_IF_FALSE(_batteryCapacity, Result::InvalidState);
        RET_IF_FALSE(_isFuelGaugeEnabled(), Result::InvalidState);
        RET_IF_ERR(_initFuelGauge());
        float minTemp = 0;
        float maxTemp = 0;
        getFuelGauge().getTemperatureRange(minTemp, maxTemp);
        RET_IF_FALSE(temperature >= minTemp && temperature <= maxTemp, Result::InvalidArg);
#if defined(CONFIG_ESP32S3_POWERFEATHER_V2) || defined(POWERFEATHER_BOARD_V2)
        if (!_fuelGaugeUsingExternalTemp)
        {
            RET_IF_FALSE(getFuelGauge().enableTSENSE(false, false), Result::Failure);
            _fuelGaugeUsingExternalTemp = true;
        }
#endif
        RET_IF_FALSE(getFuelGauge().setCellTemperature(temperature), Result::Failure);
        ESP_LOGD(TAG, "Fuel guage temperature updated to: %f °C.", temperature);
        return Result::Ok;
    }

    Result Mainboard::updateBatteryFuelGaugeTemp()
    {
        float temperature = 0.0f;
        RET_IF_ERR(getBatteryTemperature(temperature));
        return updateBatteryFuelGaugeTemp(temperature);
    }

    /*static*/ Mainboard &Mainboard::get()
    {
        static Mainboard board;
        return board;
    }
}
