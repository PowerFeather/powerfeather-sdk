#pragma once

#include <cstring>
#include <driver/i2c.h>

namespace PowerFeather
{
    static constexpr uint16_t timeout = 2000;

    class MasterI2C
    {
    public:
        bool init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq);

        template <typename T>
        bool read(uint8_t address, uint8_t reg, T &data)
        {
            uint8_t *data2 = reinterpret_cast<uint8_t*>(&data);
            return i2c_master_write_read_device(_port, address, &reg, sizeof(reg), data2, sizeof(data), pdMS_TO_TICKS(timeout)) == ESP_OK;
        }

        template <typename T>
        bool write(uint8_t address, uint8_t reg, T data)
        {
            uint8_t data2[sizeof(reg) + sizeof(data)] = { reg };
            memcpy(&data2[sizeof(reg)], &data, sizeof(data));
            return i2c_master_write_to_device(_port, address, data2, sizeof(data2), pdMS_TO_TICKS(timeout)) == ESP_OK;
        }
    private:
        uint32_t _port;
    };
}