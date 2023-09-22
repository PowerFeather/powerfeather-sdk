#include <MasterI2C.h>

namespace PowerFeather
{

    bool MasterI2C::init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq)
    {
        _port = port;

        // Initialize I2C bus
        i2c_config_t i2c_conf;
        memset(&i2c_conf, 0, sizeof(i2c_conf));
        i2c_conf.mode = I2C_MODE_MASTER;
        i2c_conf.sda_io_num = sda;
        i2c_conf.scl_io_num = scl;
        i2c_conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
        i2c_conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
        i2c_conf.master.clk_speed = freq;

        esp_err_t res;
        if ((res = i2c_param_config(_port, &i2c_conf)) != ESP_OK)
        {
            return false;
        }

        if ((res = i2c_driver_install(_port, i2c_conf.mode, 0, 0, 0)) != ESP_OK)
        {
            return false;
        }

        return true;
    }
}