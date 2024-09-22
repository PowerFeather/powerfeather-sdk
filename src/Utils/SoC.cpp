#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/rtc_io.h>

#include "Logging.h"
#include "SoC.h"

namespace PowerFeather
{
    static RTC_NOINIT_ATTR uint32_t first;
    static const uint32_t firstMagic = 0xdeadbeef;

    #define LOG_FAIL(r)                 Log.Debug(TAG, "Unexpected result %d on %s:%d.", (r), __FUNCTION__, __LINE__)
    #define RET_IF_NOK(f)               { esp_err_t r = (f); if (r != ESP_OK) { LOG_FAIL(r); return false; } }

    static const char *TAG = "PowerFeather::Chip";

    SoC &Chip;

    void SoC::init()
    {
        first = firstMagic;
    }

    bool SoC::isFirstBoot()
    {
        // If the RTC is domain is shutdown, consider next boot as first boot.
        bool isFirst = (first != firstMagic);
        Log.Debug(TAG, "Check if first boot: %d.", isFirst);
        return isFirst;
    }

    bool SoC::configureDigitalPin(Pin pin, PinMode mode)
    {
        gpio_mode_t _mode = GPIO_MODE_INPUT;
        switch (mode)
        {
        case PinMode::Output:
            _mode = GPIO_MODE_OUTPUT;
            break;

        case PinMode::InputOutput:
            _mode = GPIO_MODE_INPUT_OUTPUT;
            break;

        case PinMode::InputOutputOpenDrain:
            _mode = GPIO_MODE_INPUT_OUTPUT_OD;
            break;

        case PinMode::Input:
        default:
            break;
        }

        // Configure the digital pin.
        gpio_config_t io_conf = {};
        memset(&io_conf, 0, sizeof(io_conf));
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = _mode;
        io_conf.pin_bit_mask = (static_cast<uint64_t>(0b1) << pin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        RET_IF_NOK(gpio_config(&io_conf));
        Log.Debug(TAG, "Initialized digital pin %d with mode %d.", pin, mode);
        return true;
    }

    bool SoC::configureRTCPin(Pin pin, PinMode mode)
    {
        rtc_gpio_mode_t _mode = RTC_GPIO_MODE_INPUT_ONLY;
        switch (mode)
        {
        case PinMode::Output:
            _mode = RTC_GPIO_MODE_OUTPUT_ONLY;
            break;

        case PinMode::InputOutput:
            _mode = RTC_GPIO_MODE_INPUT_OUTPUT;
            break;

        case PinMode::InputOutputOpenDrain:
            _mode = RTC_GPIO_MODE_INPUT_OUTPUT_OD;
            break;

        case PinMode::Input:
        default:
            break;
        }

        // Configure the RTC pin.
        RET_IF_NOK(rtc_gpio_init(pin));
        RET_IF_NOK(rtc_gpio_set_direction(pin, _mode));
        RET_IF_NOK(rtc_gpio_set_direction_in_sleep(pin, _mode));
        RET_IF_NOK(rtc_gpio_pullup_dis(pin));
        RET_IF_NOK(rtc_gpio_pulldown_dis(pin));
        Log.Debug(TAG, "Initialized RTC pin %d with mode %d.", pin, _mode);
        return true;
    }

    bool SoC::setDigitalPin(Pin pin, bool value)
    {
        // Disable pin hold, set the level, and re-enable pin hold.
        rtc_gpio_hold_dis(pin);
        rtc_gpio_set_level(pin, value);
        rtc_gpio_hold_en(pin);
        Log.Debug(TAG, "Set RTC pin %d to %d.", pin, value);
        return true;
    }

    bool SoC::setRTCPin(Pin pin, bool value)
    {
        // Disable pin hold, set the level, and re-enable pin hold.
        rtc_gpio_hold_dis(pin);
        rtc_gpio_set_level(pin, value);
        rtc_gpio_hold_en(pin);
        Log.Debug(TAG, "Set RTC pin %d to %d.", pin, value);
        return true;
    }

    bool SoC::readDigitalPin(Pin pin)
    {
        return gpio_get_level(pin);
    }

    bool SoC::readRTCPin(Pin pin)
    {
        return rtc_gpio_get_level(pin);
    }

    void SoC::delay(size_t ms)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    /*static*/ SoC &SoC::get()
    {
        static SoC chip;
        return chip;
    }
}