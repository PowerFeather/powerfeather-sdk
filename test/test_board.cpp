#include <esp_private/system_internal.h>
#include <esp_sleep.h>
#include <driver/ledc.h>

#include "unity.h"

#include <Board.h>
#include <BQ2562x.h>

using namespace PowerFeather;

Board board;

static constexpr char MODULE_NAME[] = "[Board]";
static inline size_t MS_TO_US(size_t ms) { return ms * 1000; }

TEST_CASE("rtc outputs off, no glitch on deep sleep and wake", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(false);
    board.enableSTEMMAQT3V3(false);
    board.setEN(false);
    esp_deep_sleep(MS_TO_US(100));
}

TEST_CASE("rtc outputs on, no glitch on deep sleep and wake", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(true);
    board.enableSTEMMAQT3V3(true);
    board.setEN(true);
    esp_deep_sleep(MS_TO_US(100));
}

TEST_CASE("3.3v power outputs on, deep sleep current draw", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(true);
    board.enableSTEMMAQT3V3(true);
    esp_deep_sleep(MS_TO_US(10000));
}

TEST_CASE("3.3V power outputs off, deep sleep current draw", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(false);
    board.enableSTEMMAQT3V3(false);
    esp_deep_sleep(MS_TO_US(10000));
}

TEST_CASE("rtc outputs off, no glitch on digital reset", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(false);
    board.enableSTEMMAQT3V3(false);
    board.setEN(false);
    esp_restart_noos_dig();
}

TEST_CASE("rtc outputs on, no glitch on digital reset", MODULE_NAME)
{
    board.init();
    board.enableHeader3V3(true);
    board.enableSTEMMAQT3V3(true);
    board.setEN(true);
    esp_restart_noos_dig();
}

extern "C" void determine_power_source()
{
    // No reset when removing external supply (usb/dc) with battery connected.
    board.init();
    board.enableHeader3V3(true);

    #define LEDC_TIMER              LEDC_TIMER_0
    #define LEDC_MODE               LEDC_LOW_SPEED_MODE
    #define LEDC_OUTPUT_IO          (Board::Signal::LED) // Define the output GPIO
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

    Board::PowerInput last = board.getPowerInput();
    bool first = true;

    while (true)
    {
        Board::PowerInput current = board.getPowerInput();
        if (current != last || first)
        {
            switch (current)
            {
            case Board::PowerInput::Battery:
                // Barely lit
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.001f));
                printf("battery\n");
                break;

            case Board::PowerInput::USB:
                // Faint
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.01));
                printf("usb\n");
                break;

            case Board::PowerInput::DC:
                // Bright
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(1.0f));
                printf("dc\n");
                break;

            default:
                break;
            }

            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            last = current;
        }

        first = false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

TEST_CASE("determine power source", MODULE_NAME)
{
    determine_power_source();
}


static void periodic_timer_callback(void* arg)
{
    gpio_num_t pin = static_cast<gpio_num_t>((int)arg);
    gpio_set_level(pin, !gpio_get_level(pin));
}

void setup_pin(int pin)
{
    gpio_config_t io_conf = {};
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = (static_cast<uint64_t>(0b1) << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    esp_timer_create_args_t periodic_timer_args;
    memset(&periodic_timer_args, 0, sizeof(periodic_timer_args));
    periodic_timer_args.callback = &periodic_timer_callback;
    periodic_timer_args.arg = reinterpret_cast<void*>(pin);

    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, (1000000 / (pin)) / 2);
}

TEST_CASE("digital pin connections", MODULE_NAME)
{
    // Output frequency same as pin number, i.e. GPIO1 output 1Hz,
    // GPIO2 output 2Hz and so on.
    gpio_num_t exclude[] = { GPIO_NUM_0, GPIO_NUM_3, GPIO_NUM_45, GPIO_NUM_46};
    int min = 36;
    int max = 40;

    for (int i = min; i < max + 1; i++)
    {
        bool excluded = false;
        for (int e : exclude)
        {
            if (i == e)
            {
                excluded = true;
                break;
            }
        }

        if (!excluded)
        {
            setup_pin(i);
        }
    }

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

TEST_CASE("3.3V regulator response to VSYS voltage", MODULE_NAME)
{
    // Check the response of the 3.3V switching regulator as VSYS is adjusted.
}

// TEST_CASE("set VINDPM", MODULE_NAME)
// {
//     // Check the response of the 3.3V switching regulator as VSYS is adjusted.
//     board.init();
//     board.getCharger().setVINDPM(4.0f);
// }

// TEST_CASE("temperature sense", MODULE_NAME)
// {
//     // Tie potentiometer to temperature sense
//     // Check interrupt, may be combined with another test
//     board.init();
// }

TEST_CASE("charger i2c communicaton", MODULE_NAME)
{
    board.init();
    uint8_t partNum = 0, rev = 0;
    board.getCharger().readReg(BYTE(0x38), 0, 2, rev);
    board.getCharger().readReg(BYTE(0x38), 3, 5, partNum);
    TEST_ASSERT_EQUAL(0x02, partNum);
    TEST_ASSERT_EQUAL(0x02, rev);
}

// TEST_CASE("fuel guage interrupt", MODULE_NAME)
// {
//     board.init();
//     // Write a high temperature to register
// }

// TEST_CASE("charge enable and disable", MODULE_NAME)
// {
//     board.init();
// }

TEST_CASE("ship mode", MODULE_NAME)
{
    // Test ship mode can be entered
    // Measure ship mode current
    // Tie QON to reset, check that ship mode can be exited
    board.init();
    board.getCharger().setBATFETControl(BQ2562x::BATFETControl::ShipMode);
}

TEST_CASE("shutdown mode", MODULE_NAME)
{
    // Test shutdown mode can be entered
    // Measure shutdown mode current
    // Tie QON to reset, check that ship mode can be exited
    board.init();
    board.getCharger().setBATFETControl(BQ2562x::BATFETControl::ShutdownMode);
}


TEST_CASE("power cycle", MODULE_NAME)
{
    // Test shutdown mode can be entered
    // Measure shutdown mode current
    // Tie QON to reset, check that ship mode can be exited
    board.init();
    board.getCharger().setBATFETControl(BQ2562x::BATFETControl::SystemPowerReset);
}

TEST_CASE("current loading", MODULE_NAME)
{
    // Perform iperf test
    // 5V is loaded up to 2.5A
    // 3.3V is loaded up to 500mA
    board.init();
}