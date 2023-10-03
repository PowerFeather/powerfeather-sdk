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
        bool write(uint8_t address, uint8_t reg, const uint8_t *buf, size_t len);
        bool read(uint8_t address, uint8_t reg, uint8_t *buf, size_t len);
    private:
        bool read(uint8_t address, uint8_t *buffer, size_t len);
        TwoWire *_wire;
    };
}