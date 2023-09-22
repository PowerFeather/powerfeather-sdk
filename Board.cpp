#include <climits>
#include <cstdint>
#include <cstring>

#include <soc/reset_reasons.h>

#include <Board.h>

namespace PowerFeather
{
    static_assert(CHAR_BIT == 8, "Unsupported architecture");

    bool Board::_initRTCPin(gpio_num_t pin, rtc_gpio_mode_t mode)
    {
        rtc_gpio_init(pin);
        rtc_gpio_set_direction(pin, mode);
        rtc_gpio_set_direction_in_sleep(pin, mode);
        rtc_gpio_pullup_dis(pin);
        rtc_gpio_pulldown_dis(pin);
        return true;
    }

    bool Board::_initDigitalPin(gpio_num_t pin, gpio_mode_t mode)
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

        _masterI2C.init(_i2c_port, _Signal::SDA0, _Signal::SCL0, _i2c_freq);

        // Disable charging.
        _charger.enableCharging(false);
        // Disable charger TS pin; since it registers as a TS_COLD/TS_OTG_COLD
        // (TS_STAT = 0x1) if no thermistor is connected.
        _charger.enableTS(false);
        // Disable the charger watchdog to keep the charger in host mode.
        _charger.enableWD(false);

        // Initialize digital pins always.
        // _initDigitalPin(Board::Signal::EN, GPIO_MODE_INPUT);
        _initDigitalPin(Board::Signal::ALARM, GPIO_MODE_INPUT);
        _initDigitalPin(Board::Signal::INT, GPIO_MODE_INPUT);
        _initDigitalPin(Board::Signal::BTN, GPIO_MODE_INPUT);

        _initDigitalPin(Board::Signal::LED, GPIO_MODE_OUTPUT);

        _initRTCPin(Board::_Signal::EN, RTC_GPIO_MODE_OUTPUT_OD);

        _initRTCPin(Board::_Signal::EN_3V3_HEADER, RTC_GPIO_MODE_OUTPUT_ONLY);
        _initRTCPin(Board::_Signal::EN_3V3_STEMMAQT, RTC_GPIO_MODE_OUTPUT_ONLY);

        // Only initialize RTC pins if the RTC core has been reset - this
        // happens on system and chip-level resets.
        if (init != magic)
        {
            // By default, enable both the 3V3 power outputs.
            enable3V3(true);
            enableSQT(true);
            setEN(true);
        }

        _initDigitalPin(Board::_Signal::REGN, GPIO_MODE_INPUT);
        _initDigitalPin(Board::_Signal::VDD_TYPE, GPIO_MODE_INPUT);

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
        _setRTCPin(Board::_Signal::EN_3V3_HEADER, enable);
    }

    void Board::enableSQT(bool enable)
    {
        _setRTCPin(Board::_Signal::EN_3V3_STEMMAQT, enable);
    }

    Board::PowerInput Board::getPowerInput()
    {
        BQ2562x::VBUSStat vbusStat = _charger.getVBUSStat();

        if (vbusStat != BQ2562x::VBUSStat::None)
        {
            if (gpio_get_level(Board::_Signal::VDD_TYPE))
            {
                return PowerInput::DC;
            }
            else
            {
                return PowerInput::USB;
            }
        }

        return PowerInput::Battery;
    }

    void Board::setEN(bool value)
    {
        _setRTCPin(Board::_Signal::EN, value);
    }

    bool Board::isBatteryConnected()
    {
        return gpio_get_level(Board::_Signal::REGN);
    }
}
