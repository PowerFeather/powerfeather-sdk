#pragma once

#include <cstring>
#include <driver/i2c.h>

#include <Wire.h>

namespace PowerFeather
{
    class MasterI2C
    {
    public:
        bool init(uint8_t port, uint8_t sdaPin, uint8_t sclPin, uint32_t freq);
        bool read(uint8_t address, uint8_t *buffer, size_t len);
        bool write(uint8_t address, const uint8_t *buffer, size_t len);
    private:
        bool _read(uint8_t address, uint8_t *buffer, size_t len);
        TwoWire *_wire;
    };
}