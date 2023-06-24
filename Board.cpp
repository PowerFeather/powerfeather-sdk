#include <driver/rtc_io.h>

#include <esp32-hal-i2c.h>
#include <esp32-hal-gpio.h>
#include <SparkFunBQ27441.h>

#include "BQ2562x.hpp"
#include "Board.hpp"

namespace PowerFeather
{
    BQ2562x charger;

    void Board::Initialize(uint16_t batteryCapacity, uint16_t maxInputCurrent, uint16_t maxChargeCurrent)
    {
        this->_batteryCapacity = batteryCapacity;

        /**
         * Initialize pins
        */
        // Input
        pinMode(VDD_TYPE, INPUT);
        pinMode(INT, INPUT);

        // Output
        // pinMode(static_cast<gpio_num_t>(ENABLE_3V3), OUTPUT);
        rtc_gpio_init(static_cast<gpio_num_t>(ENABLE_3V3));
        rtc_gpio_set_direction(static_cast<gpio_num_t>(ENABLE_3V3), RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_set_direction_in_sleep(static_cast<gpio_num_t>(ENABLE_3V3), RTC_GPIO_MODE_OUTPUT_ONLY);

        pinMode(CE, OPEN_DRAIN);

        // Input-Output
        pinMode(EN, INPUT | OUTPUT | OPEN_DRAIN);
        pinMode(GPOUT, INPUT | OUTPUT | OPEN_DRAIN);

        lipo.begin();
        lipo.setCapacity(batteryCapacity);

        charger.Initialize();

        /**
         * Set initial state
        */
        this->Enable3V3(true);
        this->Enable5V(true);
        this->SetEN(true);
        this->EnableCharging(true);
        this->EnableGauge(true);
    }

    void Board::Enable3V3(bool enable)
    {
        if (enable)
        {
            // Set the pin high.
            rtc_gpio_set_level(static_cast<gpio_num_t>(ENABLE_3V3), 1);
            // Hold the pin high, even in deep sleep.
            rtc_gpio_hold_en(static_cast<gpio_num_t>(ENABLE_3V3));
        }
        else
        {
            // Disable pin hold during deep sleep
            rtc_gpio_hold_dis(static_cast<gpio_num_t>(ENABLE_3V3));
            // Disconnect from internal circuity to reduce leakage current
            rtc_gpio_isolate(static_cast<gpio_num_t>(ENABLE_3V3));
        }
    }

    void Board::Enable5V(bool enable)
    {

    }

    bool Board::GetEN()
    {
        return digitalRead(EN);
    }

    void Board::SetEN(bool enable)
    {
        digitalWrite(EN, enable);
    }

    void Board::EnableCharging(bool enable)
    {
        digitalWrite(static_cast<gpio_num_t>(ENABLE_3V3), enable);
    }

    void Board::EnableGauge(bool enable)
    {

    }
}
