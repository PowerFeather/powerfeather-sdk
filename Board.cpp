#include <driver/rtc_io.h>
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
    BQ2562x charger;

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

    bool Board::_enableStatLed(bool enable)
    {
        uint8_t address = 0x15;
        uint8_t data = 0;
        bool res = this->_readI2C(BQ2562x_ADDR, address, &data);
        if (res)
        {
            data |= 0b10000000;
            res = this->_writeI2C(BQ2562x_ADDR, address, data);
        }
        return res;
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

        if (i2c_param_config(i2c_master_port, &i2c_conf) != ESP_OK)
        {
            return false;
        }
        
        if (i2c_driver_install(i2c_master_port, i2c_conf.mode, 0, 0, 0) != ESP_OK)
        {
            return false;
        }

        this->_enableStatLed(false);




        // this->setChargeFactor(0.5f);
        // this->setVoltageHeader5V(5.0);
        return true;

        // /**
        //  * Initialize pins
        // */
        // // Input
        // pinMode(VDD_TYPE, INPUT);
        // pinMode(INT, INPUT);

        // // Output
        // // pinMode(static_cast<gpio_num_t>(ENABLE_3V3), OUTPUT);
        // rtc_gpio_init(static_cast<gpio_num_t>(ENABLE_3V3));
        // rtc_gpio_set_direction(static_cast<gpio_num_t>(ENABLE_3V3), RTC_GPIO_MODE_OUTPUT_ONLY);
        // rtc_gpio_set_direction_in_sleep(static_cast<gpio_num_t>(ENABLE_3V3), RTC_GPIO_MODE_OUTPUT_ONLY);

        // pinMode(CE, OPEN_DRAIN);

        // // Input-Output
        // pinMode(EN, INPUT | OUTPUT | OPEN_DRAIN);
        // pinMode(GPOUT, INPUT | OUTPUT | OPEN_DRAIN);

        // lipo.begin();
        // lipo.setCapacity(batteryCapacity);

        // charger.Initialize();

        // /**
        //  * Set initial state
        // */
        // this->Enable3V3(true);
        // this->Enable5V(true);
        // this->SetEN(true);
        // this->EnableCharging(true);
        // this->EnableGauge(true);
    }

    void Board::EnablePowerOutput(Board::PowerOutput output, bool state)
    {
        int pin = 0;

        if (pin)
        {
            if (state)
            {
                // // Set the pin high.
                // rtc_gpio_set_level(static_cast<gpio_num_t>(ENABLE_3V3), 1);
                // // Hold the pin high, even in deep sleep.
                // rtc_gpio_hold_en(static_cast<gpio_num_t>(ENABLE_3V3));
            }
            else
            {
                // // Disable pin hold during deep sleep
                // rtc_gpio_hold_dis(static_cast<gpio_num_t>(ENABLE_3V3));
                // // Disconnect from internal circuity to reduce leakage current
                // rtc_gpio_isolate(static_cast<gpio_num_t>(ENABLE_3V3));
            }
        }
    }

    bool Board::GetEN()
    {
        return digitalRead(EN);
    }

    void Board::SetEN(bool enable)
    {
        digitalWrite(EN, enable);
    }

    void Board::SetChargingEnabled(bool state)
    {
        digitalWrite(static_cast<gpio_num_t>(CE), state);
    }

    void Board::SetGaugeEnabled(bool enable)
    {

    }
}
