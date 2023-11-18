#include <climits>
#include <cstdint>
#include <cstring>

#include <soc/reset_reasons.h>

#include <MainBoard.h>

namespace PowerFeather
{
    MainBoard& Board = MainBoard::get();

    static_assert(CHAR_BIT == 8, "Unsupported architecture");

    #define RET_IF_ERR(f)        { Result r = (f); if (r != Result::Ok) { abort(); return r; } }
    #define RET_IF_NOK(f, r)     { if ((f) != ESP_OK) { abort(); return (r); } }
    #define RET_IF_FALSE(f, r)   { if ((f) == false) { abort(); return (r); } }

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

    Result MainBoard::_initChargerAndFuelGauge(uint16_t capacity)
    {
        RET_IF_FALSE(_initInternalRTCPin(Pin::FFI::EN_SQT, RTC_GPIO_MODE_INPUT_OUTPUT), Result::Failure);
        _sqtOn = rtc_gpio_get_level(Pin::FFI::EN_SQT);

        if (_sqtOn || _isFirst())
        {
            RET_IF_FALSE(_i2c.init(_i2cPort, Pin::FFI::SDA0, Pin::FFI::SCL0, _i2cFreq), Result::Failure);
        }

        if (_isFirst())
        {
            RET_IF_ERR(enableVSQT(true));

            RET_IF_ERR(enableCharging(false));
            RET_IF_ERR(enableTempSense(false));
            RET_IF_ERR(setSupplyMaxCurrent(MainBoard::_defaultVSMaxCurrent));
            RET_IF_ERR(setChargingMaxCurrent(MainBoard::_defaultChargingMaxCurrent));
            RET_IF_FALSE(_charger.setBATFETDelay(BQ2562x::BATFETDelay::Delay20ms), Result::Failure);
            RET_IF_FALSE(_charger.enableWVBUS(true), Result::Failure);
            // Disable the charger watchdog to keep the charger in host mode and to
            // keep some registers from resetting to their POR values.
            RET_IF_FALSE(_sqtOn && _charger.enableWD(false), Result::Failure);

            if (capacity > 0)
            {
                RET_IF_FALSE(_fuelGauge.setAPA(capacity), Result::Failure);
                RET_IF_FALSE(_fuelGauge.setChangeOfParameter(LC709204F::ChangeOfParameter::Nominal_3V7_Charging_4V2), Result::Failure);
                RET_IF_FALSE(_fuelGauge.enableTSENSE(false, false), Result::Failure);
                RET_IF_ERR(enableFuelGauge(true));
            }
        }

        return Result::Ok;
    }

    Result MainBoard::init(uint16_t mAh)
    {
        _initDone = false;
        RET_IF_ERR(_initChargerAndFuelGauge(mAh));
        RET_IF_FALSE(_initInternalRTCPin(Pin::FFI::EN0, RTC_GPIO_MODE_OUTPUT_OD), Result::Failure);
        RET_IF_FALSE(_initInternalRTCPin(Pin::FFI::EN_3V3, RTC_GPIO_MODE_OUTPUT_ONLY), Result::Failure);

        if (_isFirst())
        {
            RET_IF_ERR(setEN(true));
            RET_IF_ERR(enable3V3(true));
        }

        RET_IF_FALSE(_initInternalDigitalPin(Pin::FFI::PG, GPIO_MODE_INPUT), Result::Failure);

        first = firstMagic;
        _initDone = true;
        return Result::Ok;
    }

    Result MainBoard::enableFuelGauge(bool enable)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_fuelGauge.enableOperation(true), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enableSupply(bool enable)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.enableHIZ(!enable), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::setEN(bool value)
    {
        RET_IF_FALSE(_initDone || _isFirst(), Result::InvalidState );
        RET_IF_FALSE(_setRTCPin(Pin::FFI::EN0, value), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enable3V3(bool enable)
    {
        RET_IF_FALSE(_initDone || _isFirst(), Result::InvalidState );
        RET_IF_FALSE(_setRTCPin(Pin::FFI::EN_3V3, enable), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enableVSQT(bool enable)
    {
        RET_IF_FALSE(_initDone || _isFirst(), Result::InvalidState );
        RET_IF_FALSE(_setRTCPin(Pin::FFI::EN_SQT, enable), Result::Failure)
        _sqtOn = enable;
        return Result::Ok;
    }

    Result MainBoard::setSupplyMinVoltage(uint16_t mV)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.setVINDPM(mV), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::setSupplyMaxCurrent(uint16_t mA)
    {
        RET_IF_FALSE(_initDone || _isFirst(), Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.setIINDPM(mA), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getSupplyCurrent(int16_t& mA)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.getIBUS(mA), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getSupplyVoltage(uint16_t& mV)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.getVBUS(mV), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getSupplyStatus(bool& connected)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        connected = gpio_get_level(Pin::FFI::PG) == 0;
        return Result::Ok;
    }

    Result MainBoard::setVBATMinVoltage(uint16_t mV)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.setVINDPM(mV), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enterShipMode()
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.setBATFETControl(BQ2562x::BATFETControl::ShipMode), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enterShutdownMode()
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.setBATFETControl(BQ2562x::BATFETControl::ShutdownMode), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::doPowerCycle()
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.setBATFETControl(BQ2562x::BATFETControl::SystemPowerReset), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enableTempSense(bool enable)
    {
        RET_IF_FALSE(_initDone || _isFirst(), Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.enableTS(enable), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enableCharging(bool enable)
    {
        RET_IF_FALSE(_initDone || _isFirst(), Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.enableCharging(enable), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::setChargingMaxCurrent(uint16_t mA)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.setChargeCurrent(mA), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryTemperature(float& celsius)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        float x = 0;
        RET_IF_FALSE(_charger.getTS_ADC(x), Result::Failure);
        // Map percent to temperature given 103AT thermistor with fitted curve (see ts_calc.fods).
        celsius = (-1866.96172 * powf(x, 4)) + (3169.31754 * powf(x, 3)) - (1849.96775 * powf(x, 2)) + (276.6656 * x) + 81.98758;
        return Result::Ok;
    }

    Result MainBoard::getBatteryCurrent(int16_t& mA)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.getIBAT(mA), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryVoltage(uint16_t& mV)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.getVBAT(mV), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryCharge(uint8_t& percent)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_fuelGauge.getRSOC(percent), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryHealth(uint8_t& percent)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        percent = _fuelGauge.getSOH(percent) / 100.0f;
        return Result::Ok;
    }

    Result MainBoard::getBatteryTimeLeft(int& minutes)
    {
        RET_IF_FALSE(_initDone, Result::InvalidState );
        RET_IF_FALSE(_sqtOn, Result::InvalidState);

        int16_t ibat = 0;
        RET_IF_ERR(getBatteryCurrent(ibat));

        uint16_t mins = 0;
        bool discharging = ibat < 0;

        if (discharging)
        {
            _fuelGauge.getTimeToEmpty(mins);
        }
        else
        {
            _fuelGauge.getTimeToFull(mins);
        }

        if (mins == 0xFFFF)
        {
            return Result::NotReady;
        }

        minutes = mins * (discharging ? -1 : 1);
        return Result::Ok;
    }
}
