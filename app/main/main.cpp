#include <stdio.h>
#include <sdkconfig.h>
#include <driver/ledc.h>

#include <esp_sleep.h>
#include <esp_private/system_internal.h>

#include <Board.h>

PowerFeather::Board board;

#define microseconds(x)         (x * 1000000)

void test1()
{
    board.enableHeader3V3(true);
    board.enableSTEMMAQT3V3(true);
    board.setEN(false);
    esp_deep_sleep(microseconds(5));
}

void test2()
{
    board.enableHeader3V3(false);
    board.enableSTEMMAQT3V3(false);
    esp_restart();
}

void test3()
{
    board.enableHeader3V3(true);
    board.enableSTEMMAQT3V3(true);
    esp_restart_noos_dig();
}

void test4()
{
    board.enableHeader3V3(true);
    board.enableSTEMMAQT3V3(true);
    esp_restart_noos_dig();
}

void test5()
{
    board.setEN(true);

    board.enableHeader3V3(false);
    board.enableSTEMMAQT3V3(false);
    board.setEN(true);
    test1();
}

void test6()
{
    #define LEDC_TIMER              LEDC_TIMER_0
    #define LEDC_MODE               LEDC_LOW_SPEED_MODE
    #define LEDC_OUTPUT_IO          (PowerFeather::Board::Signal::LED) // Define the output GPIO
    #define LEDC_CHANNEL            LEDC_CHANNEL_0
    #define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
    #define LEDC_DUTY(x)            (((1 << 13 ) - 1) * x) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
    #define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

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
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.3f)));

    while (true)
    {
        switch (board.getPowerInput())
        {
        case PowerFeather::Board::PowerInput::Battery:
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.99f)));
            printf("battery\n");
            break;

        case PowerFeather::Board::PowerInput::USB:
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.5f)));
            printf("usb\n");
            break;

        case PowerFeather::Board::PowerInput::DC:
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.01f)));
            printf("dc\n");
            break;
        
        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void test7()
{
    while (true)
    {
        bool battery = board.isBatteryConnected();
        gpio_set_level(PowerFeather::Board::Signal::LED, battery);
        printf("battery: %d\n", battery);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#include "esp_rom_gpio.h"

extern "C" void app_main(void)
{
    board.init();
    test7();
}

