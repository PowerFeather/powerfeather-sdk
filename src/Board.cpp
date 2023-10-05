#include <climits>
#include <cstdint>
#include <cstring>

#include <soc/reset_reasons.h>

#include <Board.h>

namespace PowerFeather
{
    static_assert(CHAR_BIT == 8, "Unsupported architecture");

    bool Board::_initInternalRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode)
    {
        rtc_gpio_init(pin);
        rtc_gpio_set_direction(pin, mode);
        rtc_gpio_set_direction_in_sleep(pin, mode);
        rtc_gpio_pullup_dis(pin);
        rtc_gpio_pulldown_dis(pin);
        return true;
    }

    bool Board::_initInternalDigitalPin(gpio_num_t pin, gpio_mode_t mode)
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

    void Board::_setRTCPin(gpio_num_t pin, bool value)
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

            if (pin == Board::Pin::FFI::EN)
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

    bool Board::init()
    {
        static RTC_NOINIT_ATTR uint32_t init;
        static constexpr uint32_t magic = 0xDEADBEEF;

        _i2c.init(_i2cPort, Pin::FFI::SDA0, Pin::FFI::SCL0, _i2cFreq);

        // Initialize digital pins always.
        _initInternalRTCPin(Board::Pin::FFI::EN, RTC_GPIO_MODE_OUTPUT_OD);
        _initInternalRTCPin(Board::Pin::FFI::EN_3V3, RTC_GPIO_MODE_OUTPUT_ONLY);
        _initInternalRTCPin(Board::Pin::FFI::EN_SQT, RTC_GPIO_MODE_OUTPUT_ONLY);
        _initInternalDigitalPin(Board::Pin::FFI::PG, GPIO_MODE_INPUT);

        // Only initialize RTC pins if the RTC core has been reset - this
        // happens on system and chip-level resets.
        if (init != magic)
        {
            // By default, enable both the 3V3 power outputs.
            enable3V3(true);
            enableSQT(true);
            setEN(true);
        }

        enableTSPin(false);
        enableCharging(false);
        setVSMaxCurrent(1000);
        setChargingMaxCurrent(250);

        // Disable the charger watchdog to keep the charger in host mode.
        _charger.enableWD(false);

        init = magic;

        return true;
    }

    void Board::setEN(bool value)
    {
        _setRTCPin(Board::Pin::FFI::EN, value);
    }

    bool Board::getEN()
    {
        return rtc_gpio_get_level(Board::Pin::FFI::EN);
    }

    void Board::enable3V3(bool enable)
    {
        _setRTCPin(Board::Pin::FFI::EN_3V3, enable);
    }

    void Board::enableSQT(bool enable)
    {
        _setRTCPin(Board::Pin::FFI::EN_SQT, enable);
    }

    void Board::setVSMinVoltage(float voltage)
    {
        _charger.setVINDPM(voltage * 1000);
    }

    void Board::setVSMaxCurrent(uint32_t mA)
    {
        _charger.setIINDPM(mA);
    }

    bool Board::checkVSGood()
    {
        return gpio_get_level(Board::Pin::FFI::PG) == 0;
    }

    void Board::setVBATMinVoltage(float voltage)
    {
        _charger.setVINDPM(voltage * 1000);
    }

    void Board::enterShipMode()
    {
        _charger.setBATFETControl(BQ2562x::BATFETControl::ShipMode);
    }

    void Board::enterShutdownMode()
    {
        _charger.setBATFETControl(BQ2562x::BATFETControl::ShutdownMode);
    }

    void Board::doPowerCycle()
    {
        _charger.setBATFETControl(BQ2562x::BATFETControl::SystemPowerReset);
    }

    void Board::enableTSPin(bool enable)
    {
        _charger.enableTS(enable);
    }

    void Board::enableCharging(bool enable)
    {
        _charger.enableCharging(enable);
    }

    void Board::setChargingMaxCurrent(float current)
    {
        _charger.setChargeCurrent(current * 1000);
    }
}
