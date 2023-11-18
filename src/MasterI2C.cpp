#include <MasterI2C.h>

namespace PowerFeather
{
    bool MasterI2C::init(uint8_t port, uint8_t sdaPin, uint8_t sclPin, uint32_t freq)
    {
        i2c_config_t conf;
        memset(&conf, 0, sizeof(conf));
        conf.mode = I2C_MODE_MASTER,
        conf.sda_io_num = sdaPin,
        conf.scl_io_num = sclPin,
        conf.sda_pullup_en = GPIO_PULLUP_DISABLE,
        conf.scl_pullup_en = GPIO_PULLUP_DISABLE,
        conf.master.clk_speed = freq,

        i2c_param_config(port, &conf);

        _port = port;

        return i2c_driver_install(port, conf.mode, 0, 0, 0) == ESP_OK;
    }

    bool MasterI2C::write(uint8_t address, uint8_t reg, const uint8_t *buf, size_t len)
    {
        return i2c_master_write_read_device(_port, address, &reg, 1, const_cast<uint8_t*>(buf), len, pdMS_TO_TICKS(1000)) == ESP_OK;
    }

    bool MasterI2C::read(uint8_t address, uint8_t reg, uint8_t *buf, size_t len)
    {
        return i2c_master_write_read_device(_port, address, &reg, 1, buf, len, pdMS_TO_TICKS(1000)) == ESP_OK;
    }
}
