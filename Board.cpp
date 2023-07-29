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
        return true;
    }

    bool Board::_initDigitalPin(gpio_num_t pin, gpio_mode_t mode)
    {
        gpio_config_t io_conf = {};
        memset(&io_conf, 0, sizeof(io_conf));
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = mode;
        io_conf.pin_bit_mask = (0b1 << pin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
        return true;
    }

    bool Board::init()
    {
        soc_reset_reason_t reset_reason = esp_rom_get_reset_reason(0);

        _masterI2C.init(_i2c_port, _Signal::SDA0, _Signal::SCL0, _i2c_freq);

        if (reset_reason == RESET_REASON_CHIP_POWER_ON)
        {
            // Disable charging.
            _charger.enableCharging(false);
            // Disable charger TS pin; since it registers as a TS_COLD/TS_OTG_COLD
            // (TS_STAT = 0x1) if no thermistor is connected.
            _charger.enableTS(false);
            // Disable the charger watchdog to keep the charger in host mode.
            _charger.enableWD(false);
        }

        // Only initialize RTC pins if the RTC core has been reset - this
        // happens on system and chip-level resets.
        if (reset_reason == RESET_REASON_CHIP_POWER_ON ||
            reset_reason == RESET_REASON_CHIP_BROWN_OUT ||
            reset_reason == RESET_REASON_CHIP_SUPER_WDT ||
            reset_reason == RESET_REASON_SYS_RTC_WDT ||
            reset_reason == RESET_REASON_SYS_SUPER_WDT ||
            reset_reason == RESET_REASON_SYS_CLK_GLITCH)
        {
            _initRTCPin(Board::_Signal::Header3V3, RTC_GPIO_MODE_OUTPUT_ONLY);
            _initRTCPin(Board::_Signal::StemmaQt3V3, RTC_GPIO_MODE_OUTPUT_ONLY);
            _initRTCPin(Board::_Signal::EN, RTC_GPIO_MODE_INPUT_OUTPUT_OD);

            // By default, enable both the 3V3 power outputs.
            enableHeader3V3(true);
            enableStemmaQT3V3(true);
        }

        // Initialize digital pins always.
        _initDigitalPin(Board::Signal::EN, GPIO_MODE_INPUT);
        _initDigitalPin(Board::Signal::REGN, GPIO_MODE_INPUT);
        _initDigitalPin(Board::Signal::ALARM, GPIO_MODE_INPUT);
        _initDigitalPin(Board::Signal::INT, GPIO_MODE_INPUT);
        _initDigitalPin(Board::Signal::VDDTYPE, GPIO_MODE_INPUT);
        _initDigitalPin(Board::Signal::LED, GPIO_MODE_INPUT);
        _initDigitalPin(Board::Signal::BTN, GPIO_MODE_INPUT);

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

            if (pin == Board::_Signal::EN)
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

    void Board::enableHeader3V3(bool enable)
    {
        _setRTCPin(Board::_Signal::Header3V3, enable);
    }

    void Board::enableStemmaQT3V3(bool enable)
    {
        _setRTCPin(Board::_Signal::Header3V3, enable);
    }

    Board::PowerInput Board::getPowerInput()
    {
        BQ2562x::VBUSStat vbusStat = _charger.getVBUSStat();

        if (vbusStat != BQ2562x::VBUSStat::None)
        {
            if (gpio_get_level(Board::Signal::VDDTYPE))
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
}
