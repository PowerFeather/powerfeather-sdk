#include <climits>
#include <cstdint>
#include <cstring>

#include <soc/reset_reasons.h>

#include <MainBoard.h>


namespace PowerFeather
{
    MainBoard& Board = MainBoard::get();

    static_assert(CHAR_BIT == 8, "Unsupported architecture");

    /*static*/ MainBoard& MainBoard::get()
    {
        static MainBoard board;
        return board;
    }

    bool MainBoard::_initInternalRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode)
    {
        rtc_gpio_init(pin);
        rtc_gpio_set_direction(pin, mode);
        rtc_gpio_set_direction_in_sleep(pin, mode);
        rtc_gpio_pullup_dis(pin);
        rtc_gpio_pulldown_dis(pin);
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
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        return true;
    }

    void MainBoard::_setRTCPin(gpio_num_t pin, bool value)
    {
        if (value)
        {
            // Set the pin high.
            rtc_gpio_set_level(pin, value);
            // Hold the pin high, even in deep sleep.
            rtc_gpio_hold_en(pin);
        }
        else
        {
            // Disable pin hold during deep sleep
            rtc_gpio_hold_dis(pin);

            if (pin == MainBoard::Pin::FFI::EN)
            {
                // The enable pin is in open-drain configuration with external pull-up
                // resistor. Setting the pin to 0 means pulling it down.
                rtc_gpio_set_level(pin, 0);
            }
            else
            {
                // The rest of the output RTC pins is by default pulled down by an
                // external resistor, this means setting them to 0 entails just
                // just disconnecting the pin from the chip.
                rtc_gpio_isolate(pin);
            }
        }
    }

    bool MainBoard::_isInited()
    {
        return _inited == MainBoard::_initMagic;
    }

    Error MainBoard::_initChargerAndFuelGauge()
    {
        Error res = Error::None;

        if (_i2c.init(_i2cPort, Pin::FFI::SDA0, Pin::FFI::SCL0, _i2cFreq))
        {
            // Only initialize RTC pins if the RTC core has been reset - this
            // happens on system and chip-level resets.
            if (!_isInited())
            {
                _initInternalRTCPin(MainBoard::Pin::FFI::EN_SQT, RTC_GPIO_MODE_OUTPUT_ONLY);

                enableSQT(true);

                enableCharging(false);
                enableTSPin(false);
                setVSMaxCurrent(1000);
                setChargingMaxCurrent(250);

                // Disable the charger watchdog to keep the charger in host mode.
                _charger.enableWD(false);
            }
        }
        else
        {
            res = Error::Failure;
        }

        return res;
    }

    Error MainBoard::init(uint16_t)
    {
        Error res = Error::None;

        if ((res = _initChargerAndFuelGauge()) == Error::None)
        {
            _initInternalRTCPin(MainBoard::Pin::FFI::EN, RTC_GPIO_MODE_OUTPUT_OD);
            _initInternalRTCPin(MainBoard::Pin::FFI::EN_3V3, RTC_GPIO_MODE_OUTPUT_ONLY);
            _initInternalDigitalPin(MainBoard::Pin::FFI::PG, GPIO_MODE_INPUT);

            // By default, enable both the 3V3 power outputs.
            if (_isInited())
            {
                // Initialize digital pins always.

                enable3V3(true);

                setEN(true);
            }

            _inited = MainBoard::_initMagic;
        }
 
        return res;
    }

    void MainBoard::setEN(bool value)
    {
        _setRTCPin(MainBoard::Pin::FFI::EN, value);
    }

    bool MainBoard::getEN()
    {
        return rtc_gpio_get_level(MainBoard::Pin::FFI::EN);
    }

    void MainBoard::enable3V3(bool enable)
    {
        _setRTCPin(MainBoard::Pin::FFI::EN_3V3, enable);
    }

    void MainBoard::enableSQT(bool enable)
    {
        _setRTCPin(MainBoard::Pin::FFI::EN_SQT, enable);
        _sqtOn = enable;
    }

    void MainBoard::setVSMinVoltage(float voltage)
    {
        if (_sqtOn)
        {
            _charger.setVINDPM(voltage * 1000);
        }
    }

    void MainBoard::setVSMaxCurrent(uint32_t mA)
    {
        if (_sqtOn)
        {
            _charger.setIINDPM(mA);
        }
    }

    bool MainBoard::checkVSGood()
    {
        return gpio_get_level(MainBoard::Pin::FFI::PG) == 0;
    }

    void MainBoard::setVBATMinVoltage(float voltage)
    {
        if (_sqtOn)
        {
            _charger.setVINDPM(voltage * 1000);
        }
    }

    void MainBoard::enterShipMode()
    {
        if (_sqtOn)
        {
            _charger.setBATFETControl(BQ2562x::BATFETControl::ShipMode);
        }
    }

    void MainBoard::enterShutdownMode()
    {
        if (_sqtOn)
        {
            _charger.setBATFETControl(BQ2562x::BATFETControl::ShutdownMode);
        }
    }

    void MainBoard::doPowerCycle()
    {
        if (_sqtOn)
        {
            _charger.setBATFETControl(BQ2562x::BATFETControl::SystemPowerReset);
        }
    }

    void MainBoard::enableTSPin(bool enable)
    {
        if (_sqtOn)
        {
            _charger.enableTS(enable);
        }
    }

    void MainBoard::enableCharging(bool enable)
    {
        if (_sqtOn)
        {
            _charger.enableCharging(enable);
        }
    }

    void MainBoard::setChargingMaxCurrent(float current)
    {
        if (_sqtOn)
        {
            _charger.setChargeCurrent(current * 1000);
        }
    }

    float MainBoard::getBatteryVoltage()
    {
        if (_sqtOn)
        {
            return _fuelGauge.getCellVoltage();
        }

        return 0.0f;
    }

    float MainBoard::getBatteryCharge()
    {
        if (_sqtOn)
        {
            return _fuelGauge.getRSOC() / 100.0f;
        }

        return 0.0f;
    }

    float MainBoard::getBatteryHealth()
    {
        if (_sqtOn)
        {
            return _fuelGauge.getSOH() / 100.0f;
        }

        return 0.0f;
    }

    int32_t MainBoard::getBatteryTimeLeft()
    {
        if (_sqtOn)
        {
            if (_charger.isCharging())
            {
                return _fuelGauge.getTimeToFull();
            }

            return _fuelGauge.getTimeToEmpty() * -1;
        }

        return 0;
    }

    /*static*/ RTC_NOINIT_ATTR uint32_t MainBoard::_inited;
}
