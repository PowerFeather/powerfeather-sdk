#pragma once

#include <driver/i2c.h>


namespace PowerFeather
{
    class MasterI2C
    {
    public:
        bool init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq);
        template <typename T>
        bool read(uint8_t address, uint8_t reg, T &data);
        template <typename T>
        bool write(uint8_t address, uint8_t reg, T data);
    private:
        uint32_t _port;
    };
}