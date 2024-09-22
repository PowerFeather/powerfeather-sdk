#pragma once

#if ESP_PLATFORM
#include <driver/gpio.h>
#endif

namespace PowerFeather
{
    class SoC
    {
    public:
    #if ESP_PLATFORM
        typedef gpio_num_t Pin;
    #else
        typedef uint8_t Pin;
    #endif

        enum class PinMode
        {
            Input,
            Output,
            InputOutput,
            InputOutputOpenDrain
        };

        void init();
        bool configureDigitalPin(Pin pin, PinMode mode);
        bool configureRTCPin(Pin pin, PinMode mode);
        bool setRTCPin(Pin pin, bool value);
        bool setDigitalPin(Pin pin, bool value);
        bool readDigitalPin(Pin pin);
        bool readRTCPin(Pin pin);
        bool isFirstBoot();

        static SoC &get();

        void delay(size_t ms);
    };

    extern SoC &Chip;
}
