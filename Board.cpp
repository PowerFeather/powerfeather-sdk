#include <driver/i2c.h>
#include <SparkFunBQ27441.h>

#include "BQ2562x.hpp"
#include "Board.hpp"


static constexpr int I2C_NUM = 0;
static constexpr uint32_t I2C_SPEED = 400000;
static constexpr uint32_t I2C_TIMEOUT = 1000;

static constexpr uint8_t BQ2562x_ADDR = 0x6a;

namespace PowerFeather
{
    bool Board::_readI2C(uint8_t address, uint8_t reg, uint8_t *data)
    {
        return i2c_master_write_read_device(I2C_NUM, address, &reg, 1, data, 1, pdMS_TO_TICKS(I2C_TIMEOUT)) == ESP_OK;
    }

    bool Board::_writeI2C(uint8_t address, uint8_t reg, uint8_t data)
    {
        uint8_t to_write[2] = {reg, data};
        return i2c_master_write_to_device(I2C_NUM, address, to_write, sizeof(to_write), pdMS_TO_TICKS(I2C_TIMEOUT)) == ESP_OK;
    }

    Board::Board(uint16_t batteryCapacity, bool useTSPin)
    {
        this->_batteryCapacity = batteryCapacity;
        this->_useTSPin = useTSPin;
    }

    bool Board::setChargeFactor(float factor)
    {
        return true;
    }

    bool Board::setVoltageHeader5V(float voltage)
    {
        // Sets the voltage header voltage for 
        return true;
    }

    bool Board::_setChargerRegister(uint8_t address, uint8_t bit, bool value)
    {
        uint8_t data = 0;
        bool res = this->_readI2C(BQ2562x_ADDR, address, &data);
        if (res)
        {
            if (value)
            {
                data |= (0b1 << bit);
            }
            else
            {
                data &= ~(0b1 << bit);
            }
            res = this->_writeI2C(BQ2562x_ADDR, address, data);
        }
        return res;
    }

    bool Board::_enableChargerStatLed(bool enable)
    {
        return _setChargerRegister(0x15, 7, !enable);
    }

    bool Board::_enableChargerTS(bool enable)
    {
        return _setChargerRegister(0x1a, 7, !enable);
    }

    bool Board::_initRTCPin(int pin, rtc_gpio_mode_t mode)
    {
        rtc_gpio_init(static_cast<gpio_num_t>(pin));
        rtc_gpio_set_direction(static_cast<gpio_num_t>(pin), mode);
        rtc_gpio_set_direction_in_sleep(static_cast<gpio_num_t>(pin), mode);
        return true;
    }

    bool Board::_enableRTCPin(bool enable)
    {
        return true;
    }

    uint8_t Board::_getChargerFault()
    {
        uint8_t data;
        this->_readI2C(BQ2562x_ADDR, 0x1f, &data);
        return data;
    }

    bool Board::init()
    {
        // Initialize I2C bus
        int i2c_master_port = I2C_NUM;

        i2c_config_t i2c_conf;
        memset(&i2c_conf, 0, sizeof(i2c_conf));
        i2c_conf.mode = I2C_MODE_MASTER;
        i2c_conf.sda_io_num = Board::SDA0Pin;
        i2c_conf.scl_io_num = Board::SCL0Pin;
        i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.master.clk_speed = I2C_SPEED;

        esp_err_t res;
        if ((res = i2c_param_config(i2c_master_port, &i2c_conf)) != ESP_OK)
        {
            return false;
        }
        
        if ((res = i2c_driver_install(i2c_master_port, i2c_conf.mode, 0, 0, 0)) != ESP_OK)
        {
            printf("res: %d\n", res);
            return false;
        }

        _enableChargerTS(_useTSPin);


        // _setChargeFactor(0.5f);
        // _setVoltageHeader5V(5.0);

        // Initialize pins


        /**
         * Initialize pins
         * 
         * EnableHeader3V3Pin - output pp, hold sleep
         * EnableStemma3V3Pin - output pp, hold sleep
         * EnablePin - input/output od, hold sleep, wake source, interrupt source
         * 
         * ChargerIntPin - input, wake source, interrupt source
         * GaugeAlarmPin - input, wake source, interrupt source
         * VDDTypePin - input, wake source, interrupt source
         * 
         * ChargerEnPin - output, hold sleep 
         */
        // Output pins

        // Input pins

        // RTC output pins
        _initRTCPin(Board::EnableHeader3V3Pin, RTC_GPIO_MODE_OUTPUT_ONLY);
        _initRTCPin(Board::EnableStemma3V3Pin, RTC_GPIO_MODE_OUTPUT_ONLY);

        // RTC input pins

        // RTC input output
        _initRTCPin(Board::EnablePin, RTC_GPIO_MODE_INPUT_OUTPUT_OD);

        // wake source
        // interrupt

        enableHeader3V3(true);
        enableStemma3V3(true);

        return true;
    }

    void Board::_setRTCPin(gpio_num_t pin, bool value)
    {
        if (value)
        {
            // Disable pin hold during deep sleep
            rtc_gpio_hold_dis(pin);
            // Disconnect from internal circuity to reduce leakage current
            rtc_gpio_isolate(pin);
        }
        else
        {
            // Disable pin hold during deep sleep
            rtc_gpio_hold_dis(pin);
            // Disconnect from internal circuity to reduce leakage current
            rtc_gpio_isolate(pin);
        }
    }

    void Board::enableHeader3V3(bool enable)
    {
        _setRTCPin(Board::EnableHeader3V3Pin, enable);
    }

    void Board::enableStemma3V3(bool enable)
    {
        _setRTCPin(Board::EnableStemma3V3Pin, enable);
    }

    void Board::enableHeader5V(bool enabVle)
    {
    }

    bool Board::getEnablePin()
    {
        return true;
    }

    void Board::setEnablePin(bool value)
    {
    }

    void Board::enableCharging(bool state)
    {

    }

    void Board::enableGauge(bool enable)
    {

    }
}
