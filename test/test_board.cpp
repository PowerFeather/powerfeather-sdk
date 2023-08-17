#include <esp_private/system_internal.h>
#include <esp_sleep.h>
#include <driver/ledc.h>

#include "unity.h"

#include <Board.h>

PowerFeather::Board board;

static constexpr char MODULE_NAME[] = "[PowerFeather::Board]";
static inline size_t MS_TO_US(size_t ms) { return ms * 1000; }


TEST_CASE("Basic I2C communicaton", MODULE_NAME)
{
    board.init();

    board.getCharger().getPartInformation();
}

TEST_CASE("Charger interrupt", MODULE_NAME)
{
    board.init();
    board.getCharger().getPartInformation();
}

TEST_CASE("FuelGauge interrupt", MODULE_NAME)
{
    board.init();
}

TEST_CASE("Charge enable and disable", MODULE_NAME)
{
    board.init();
}

TEST_CASE("Ship mode", MODULE_NAME)
{
    // Test ship mode can be entered
    // Measure ship mode current
    // Tie QON to reset, check that ship mode can be exited
    board.init();
}

TEST_CASE("Shutdown mode current", MODULE_NAME)
{
    // Test shutdown mode can be entered
    // Measure shutdown mode current
    // Tie QON to reset, check that ship mode can be exited
    board.init();
}

TEST_CASE("Temperature sense", MODULE_NAME)
{
    // Tie potentiometer to temperature sense
    // Check interrupt, may be combined with another test
    board.init();
}

TEST_CASE("Pin connections", MODULE_NAME)
{
    board.init();
}

TEST_CASE("Current loading", MODULE_NAME)
{
    // Perform iperf test
    // 5V is loaded up to 2.5A
    // 3.3V is loaded up to 500mA 
    board.init();
}

TEST_CASE("3.3V rails off, no glitch on deep sleep and wake", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(false);
    board.enableSTEMMAQT3V3(false);
    esp_deep_sleep(MS_TO_US(100));
}

TEST_CASE("3.3V rails on, no glitch on deep sleep and wake", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(true);
    board.enableSTEMMAQT3V3(true);
    esp_deep_sleep(MS_TO_US(100));
}

TEST_CASE("3.3V rails on, deep sleep current draw", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(true);
    board.enableSTEMMAQT3V3(true);
    esp_deep_sleep(MS_TO_US(7000));
}


TEST_CASE("3.3V rails off, deep sleep current draw", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(false);
    board.enableSTEMMAQT3V3(false);
    esp_deep_sleep(MS_TO_US(7000));
}

RTC_DATA_ATTR bool high = false;
TEST_CASE("3.3V rails on/off after deep sleep wake", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(high);
    board.enableSTEMMAQT3V3(high);
    printf("%d\n", high);
    high = !high;
    printf("%d\n", high);
    esp_deep_sleep(MS_TO_US(100));
}

TEST_CASE("3.3V rails off, no glitch on digital reset", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(false);
    board.enableSTEMMAQT3V3(false);
    esp_restart_noos_dig();
}

TEST_CASE("3.3V rails on, no glitch on digital reset", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(true);
    board.enableSTEMMAQT3V3(true);
    esp_restart_noos_dig();
}

TEST_CASE("Determining power source", MODULE_NAME)
{
    #define LEDC_TIMER              LEDC_TIMER_0
    #define LEDC_MODE               LEDC_LOW_SPEED_MODE
    #define LEDC_OUTPUT_IO          (PowerFeather::Board::Signal::LED) // Define the output GPIO
    #define LEDC_CHANNEL            LEDC_CHANNEL_0
    #define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
    #define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz
    #define LEDC_DUTY(x)            (((1 << 13 ) - 1) * x) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095

    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode       = LEDC_MODE;
    ledc_timer.timer_num        = LEDC_TIMER;
    ledc_timer.duty_resolution  = LEDC_DUTY_RES;
    ledc_timer.freq_hz          = LEDC_FREQUENCY;
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode     = LEDC_MODE;
    ledc_channel.channel        = LEDC_CHANNEL;
    ledc_channel.timer_sel      = LEDC_TIMER;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num       = LEDC_OUTPUT_IO;
    ledc_channel.duty           = 0;
    ledc_channel.hpoint         = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.01f)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    while (true)
    {
        switch (board.getPowerInput())
        {
        case PowerFeather::Board::PowerInput::Battery:
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.999f)));
            break;

        case PowerFeather::Board::PowerInput::USB:
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.95f)));
            break;

        case PowerFeather::Board::PowerInput::DC:
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.01f)));
            break;
        
        default:
            break;
        }

        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
}

// printf("voltage: %02x\n", board.getCharger().getPartInformation());

// uint8_t adc_reg = 0;
// board.getCharger().readReg((uint8_t)0x26, adc_reg);
// printf("adc_reg: 0x%02x ----\n", adc_reg);

// board.getCharger().enableADC(true);

// while (true)
// {
//     adc_reg = 0;
//     board.getCharger().readReg((uint8_t)0x26, adc_reg);
//     printf("adc_reg: 0x%02x ---- ", adc_reg);


//     printf("voltage: %f\n", board.getCharger().getBatteryVoltage());
//     vTaskDelay(pdMS_TO_TICKS(100));
// }
// test6();