#pragma once

#include <cstring>
#include <driver/i2c.h>

namespace PowerFeather
{
    class MasterI2C
    {
    public:
        virtual bool init(uint8_t port, uint8_t sdaPin, uint8_t sclPin, uint32_t freq);
        virtual bool write(uint8_t address, uint8_t reg, const uint8_t *buf, size_t len);
        virtual bool read(uint8_t address, uint8_t reg, uint8_t *buf, size_t len);
    private:
        uint8_t _port;
    };
}