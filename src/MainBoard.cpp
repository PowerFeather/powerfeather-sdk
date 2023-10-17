#include <climits>
#include <cstdint>
#include <cstring>

#include <soc/reset_reasons.h>

#include <MainBoard.h>

namespace PowerFeather
{
    MainBoard& Board = MainBoard::get();

    static_assert(CHAR_BIT == 8, "Unsupported architecture");

    #define RET_IF_ERR(f)        { Result r = (f); if (r != Result::Ok) { return r; } }
    #define RET_IF_NOK(f, r)     { if ((f) != ESP_OK) { return (r); } }
    #define RET_IF_FALSE(f, r)   { if ((f) == false) { return (r); } }

    static RTC_NOINIT_ATTR uint32_t inited;
    static const uint32_t initedMagic = 0xdeadbeef;

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
        return inited != initedMagic;
    }

    Result MainBoard::_initChargerAndFuelGauge(uint16_t mAh)
    {
        RET_IF_FALSE(_i2c.init(_i2cPort, Pin::FFI::SDA0, Pin::FFI::SCL0, _i2cFreq), Result::Failure);

        if (_isFirst())
        {
            RET_IF_FALSE(_initInternalRTCPin(Pin::FFI::EN_SQT, RTC_GPIO_MODE_INPUT_OUTPUT), Result::Failure);
            RET_IF_ERR(enableSQT(true));

            RET_IF_ERR(enableCharging(false));
            RET_IF_ERR(enableChargingTemperatureMonitor(false));
            RET_IF_ERR(setSupplyMaxCurrent(MainBoard::_defaultVSMaxCurrent));
            RET_IF_ERR(setChargingMaxCurrent(MainBoard::_defaultChargingMaxCurrent));
            RET_IF_FALSE(_charger.setBATFETDelay(BQ2562x::BATFETDelay::Delay20ms), Result::Failure);
            RET_IF_FALSE(_charger.enableWVBUS(true), Result::Failure);
            // Disable the charger watchdog to keep the charger in host mode and to
            // keep some registers from resetting to their POR values.
            RET_IF_FALSE(_sqtOn && _charger.enableWD(false), Result::Failure);

            if (mAh > 0)
            {
                RET_IF_FALSE(_fuelGauge.setAPA(mAh), Result::Failure);
                RET_IF_FALSE(_fuelGauge.setChangeOfParameter(LC709204F::ChangeOfParameter::Nominal_3V7_Charging_4V2), Result::Failure);
                RET_IF_FALSE(_fuelGauge.enableTSENSE(false, false), Result::Failure);
                RET_IF_FALSE(_fuelGauge.enableOperation(true), Result::Failure);
            }
        }

        _sqtOn = rtc_gpio_get_level(Pin::FFI::EN_SQT);

        return Result::Ok;
    }

    Result MainBoard::init(uint16_t mAh)
    {
        RET_IF_ERR(_initChargerAndFuelGauge(mAh));

        if (_isFirst())
        {
            RET_IF_FALSE(_initInternalRTCPin(Pin::FFI::EN, RTC_GPIO_MODE_OUTPUT_OD), Result::Failure);
            RET_IF_ERR(setEN(true));
            RET_IF_FALSE(_initInternalRTCPin(Pin::FFI::EN_3V3, RTC_GPIO_MODE_OUTPUT_ONLY), Result::Failure);
            RET_IF_ERR(enable3V3(true));
        }

        RET_IF_FALSE(_initInternalDigitalPin(Pin::FFI::PG, GPIO_MODE_INPUT), Result::Failure);

        inited = initedMagic;
        return Result::Ok;
    }

    Result MainBoard::setEN(bool value)
    {
        RET_IF_FALSE(_setRTCPin(Pin::FFI::EN, value), Result::Failure);
        return Result::Ok;
    }

    bool MainBoard::getEN()
    {
        return rtc_gpio_get_level(Pin::FFI::EN);
    }

    Result MainBoard::enable3V3(bool enable)
    {
        RET_IF_FALSE(_setRTCPin(Pin::FFI::EN_3V3, enable), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enableSQT(bool enable)
    {
        RET_IF_FALSE(_setRTCPin(Pin::FFI::EN_SQT, enable), Result::Failure)
        _sqtOn = enable;
        return Result::Ok;
    }

    void MainBoard::setSupplyMinVoltage(float voltage)
    {
        if (_sqtOn)
        {
            _charger.setVINDPM(voltage * 1000);
        }
    }

    Result MainBoard::setSupplyMaxCurrent(uint32_t mA)
    {
        RET_IF_FALSE(_sqtOn && _charger.setIINDPM(mA), Result::Failure);
        return Result::Ok;
    }

    bool MainBoard::checkSupplyConnected()
    {
        return gpio_get_level(Pin::FFI::PG) == 0;
    }

    void MainBoard::setVBATMinVoltage(float voltage)
    {
        if (_sqtOn)
        {
            _charger.setVINDPM(voltage * 1000);
        }
    }

    Result MainBoard::enterShipMode()
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.setBATFETControl(BQ2562x::BATFETControl::ShipMode), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enterShutdownMode()
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.setBATFETControl(BQ2562x::BATFETControl::ShutdownMode), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::doPowerCycle()
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.setBATFETControl(BQ2562x::BATFETControl::SystemPowerReset), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enableChargingTemperatureMonitor(bool enable)
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.enableTS(enable), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::enableCharging(bool enable)
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.enableCharging(enable), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::setChargingMaxCurrent(float current)
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.setChargeCurrent(current), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryTemperature(float& temperature)
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.getBatteryTemperature(temperature), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryCurrent(float& current)
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.getIBAT(current), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryVoltage(float& voltage)
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        RET_IF_FALSE(_charger.getVBAT(voltage), Result::Failure);
        return Result::Ok;
    }

    Result MainBoard::getBatteryCharge(float& percent)
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        percent = _fuelGauge.getRSOC() / 100.0f;
        return Result::Ok;
    }

    Result MainBoard::getBatteryHealth(float& percent)
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        percent = _fuelGauge.getSOH() / 100.0f;
        return Result::Ok;
    }

    Result MainBoard::getBatteryTimeLeft(int& minutes)
    {
        RET_IF_FALSE(_sqtOn, Result::InvalidState);
        minutes = 0;
        return Result::Ok;
    }
}
