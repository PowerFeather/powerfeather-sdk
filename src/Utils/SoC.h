#pragma once

#include <driver/gpio.h>

class SoC
{
public:
    typedef gpio_num_t Pin;

    enum class PinMode
    {
        Input,
        Output,
        InputOutput,
        InputOutputOpenDrain
    };

    void Init();
    bool ConfigureDigitalPin(Pin pin, PinMode mode);
    bool ConfigureRTCPin(Pin pin, PinMode mode);
    bool SetRTCPin(Pin pin, bool value);
    bool SetDigitalPin(Pin pin, bool value);
    bool ReadDigitalPin(Pin pin);
    bool ReadRTCPin(Pin pin);
    bool IsFirstBoot();

    static SoC &get();
};

extern SoC &Chip;