#include <climits>
#include <cstdint>
#include <cstring>

#include <soc/reset_reasons.h>

#include <Board.h>

namespace PowerFeather
{
    static_assert(CHAR_BIT == 8, "Unsupported architecture");

    bool Board::_initRTCPin(int pin, rtc_gpio_mode_t mode)
    {
        rtc_gpio_init(static_cast<gpio_num_t>(pin));
        rtc_gpio_set_direction(static_cast<gpio_num_t>(pin), mode);
        rtc_gpio_set_direction_in_sleep(static_cast<gpio_num_t>(pin), mode);
        return true;
    }

    bool Board::_initDigitalPin(int pin, gpio_mode_t mode)
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

        _masterI2C.init(_i2c_port, Board::SDA0Pin, Board::SCL0Pin, _i2c_freq);

        if (reset_reason == RESET_REASON_CHIP_POWER_ON)
        {
            _charger.enableCharging(false);
            _charger.enableTS(false);
            _charger.enableWD(false);
        }

        if (reset_reason == RESET_REASON_CHIP_POWER_ON || 
            reset_reason == RESET_REASON_CHIP_BROWN_OUT ||
            reset_reason == RESET_REASON_CHIP_SUPER_WDT ||
            reset_reason == RESET_REASON_SYS_RTC_WDT || 
            reset_reason == RESET_REASON_SYS_SUPER_WDT ||
            reset_reason == RESET_REASON_SYS_CLK_GLITCH)
        {
            _initRTCPin(Board::EnableHeader3V3Pin, RTC_GPIO_MODE_OUTPUT_ONLY);
            _initRTCPin(Board::EnableStemma3V3Pin, RTC_GPIO_MODE_OUTPUT_ONLY);
            _initRTCPin(Board::EnablePin, RTC_GPIO_MODE_INPUT_OUTPUT_OD);

            enableHeader3V3(true);
            enableStemma3V3(true);
        }

        // Initialize IO pins
        _initDigitalPin(Board::ChargerIntPin, GPIO_MODE_INPUT);
        _initDigitalPin(Board::GaugeAlarmPin, GPIO_MODE_INPUT);
        _initDigitalPin(Board::GaugeRegPin, GPIO_MODE_INPUT);
        _initDigitalPin(Board::VDDTypePin, GPIO_MODE_INPUT);

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

            if (pin == Board::EnablePin)
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
        _setRTCPin(Board::EnableHeader3V3Pin, enable);
    }

    void Board::enableStemma3V3(bool enable)
    {
        _setRTCPin(Board::EnableStemma3V3Pin, enable);
    }

    Board::PowerInput Board::getPowerInput()
    {
        BQ2562x::VBUSStat vbusStat = _charger.getVBUSStat();

        if (vbusStat != BQ2562x::VBUSStat::None)
        {
            if (gpio_get_level(Board::VDDTypePin))
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

    bool Board::getEnablePin()
    {
        return rtc_gpio_get_level(Board::EnablePin);
    }

    void Board::setEnablePin(bool value)
    {
        _setRTCPin(Board::EnablePin, value);
    }
}
