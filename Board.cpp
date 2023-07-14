#include <driver/rtc_io.h>

#include <driver/i2c.h>

#include <esp32-hal-i2c.h>
#include <esp32-hal-gpio.h>
#include <SparkFunBQ27441.h>

#include "BQ2562x.hpp"
#include "Board.hpp"


static constexpr int I2C_NUM = 0;
static constexpr uint32_t I2C_SPEED = 400000;

namespace PowerFeather
{
    BQ2562x charger;

    void Board::Initialize(uint16_t batteryCapacity, uint16_t maxInputCurrent, uint16_t maxChargeCurrent)
    {
        // Initialize I2C bus
        int i2c_master_port = I2C_NUM;

        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = SDA0;
        conf.scl_io_num = SCL0;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_SPEED;

        i2c_param_config(i2c_master_port, &conf);
        i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
        


        // this->_batteryCapacity = batteryCapacity;

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

    void Board::SetPowerOutputEnabled(Board::PowerInput input, bool state)
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
