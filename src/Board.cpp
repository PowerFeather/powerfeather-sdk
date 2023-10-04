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

        // Disable charging.
        _charger.enableCharging(false);

        // Disable charger TS pin; since it registers as a TS_COLD/TS_OTG_COLD
        // (TS_STAT = 0x1) if no thermistor is connected.
        _charger.enableTS(false);
        // Disable the charger watchdog to keep the charger in host mode.
        _charger.enableWD(false);

        init = magic;

        return true;
    }

    void Board::_setRTCPin(gpio_num_t pin, bool value)
    {
        rtc_gpio_hold_dis(pin);
        rtc_gpio_set_level(pin, value);
        rtc_gpio_hold_en(pin);
    }

    void Board::enable3V3(bool enable)
    {
        _setRTCPin(Board::Pin::FFI::EN_3V3, enable);
    }

    void Board::enableSQT(bool enable)
    {
        _setRTCPin(Board::Pin::FFI::EN_SQT, enable);
    }

    void Board::setEN(bool value)
    {
        _setRTCPin(Board::Pin::FFI::EN, value);
    }

    bool Board::getEN()
    {
        return rtc_gpio_get_level(Board::Pin::FFI::EN);
    }

    bool Board::checkVSGood()
    {
        return gpio_get_level(Board::Pin::FFI::PG) == 0;
    }

    void Board::enterShipMode()
    {
        _charger.setBATFETControl(BQ2562x::BATFETControl::ShipMode);
    }

    void Board::enterShutdownMode()
    {
        _charger.setBATFETControl(BQ2562x::BATFETControl::ShutdownMode);
    }

    void Board::setVBATMin(float voltage)
    {
        _charger.setVINDPM(voltage);
    }

    void Board::doPowerCycle()
    {
        _charger.setBATFETControl(BQ2562x::BATFETControl::SystemPowerReset);
    }
}
