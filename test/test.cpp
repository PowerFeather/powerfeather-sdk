// #include <stdio.h>
// #include <sdkconfig.h>
// #include <driver/ledc.h>

// #include <esp_private/system_internal.h>

#include <esp_sleep.h>
#include "unity.h"

#include <Board.h>

PowerFeather::Board board;

static constexpr char MODULE_NAME[] = "[PowerFeather::Board]";
static inline size_t MS_TO_US(size_t ms) { return ms * 1000; }

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

// void test_3v3_rails_on_deep_sleep_current()
// {
//     board.enableHeader3V3(false);
//     board.enableSTEMMAQT3V3(false);
//     esp_deep_sleep(ms2micro(5000));
// }

// RTC_NOINIT_ATTR bool high = false;
// void test_3v3_rails_on_off_changeable_even_deep_sleep()
// {
//     board.enableHeader3V3(high);
//     board.enableSTEMMAQT3V3(high);
//     high = !high;
//     esp_deep_sleep(ms2micro(100));
// }

// void test_3v3_rails_off_deep_sleep_no_glitch()
// {
//     board.enableHeader3V3(false);
//     board.enableSTEMMAQT3V3(false);
//     esp_deep_sleep(ms2micro(100));
// }


// void test_3v3_rails_off_digital_reset_no_glitch()
// {
//     board.enableHeader3V3(false);
//     board.enableSTEMMAQT3V3(false);
//     esp_restart_noos_dig();
// }

// void test_3v3_rails_on_digital_reset_no_glitch()
// {
//     board.enableHeader3V3(true);
//     board.enableSTEMMAQT3V3(true);
//     esp_restart_noos_dig();
// }


// void test6()
// {
//     #define LEDC_TIMER              LEDC_TIMER_0
//     #define LEDC_MODE               LEDC_LOW_SPEED_MODE
//     #define LEDC_OUTPUT_IO          (PowerFeather::Board::Signal::LED) // Define the output GPIO
//     #define LEDC_CHANNEL            LEDC_CHANNEL_0
//     #define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
//     #define LEDC_DUTY(x)            (((1 << 13 ) - 1) * x) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
//     #define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

//     ledc_timer_config_t ledc_timer;
//     ledc_timer.speed_mode       = LEDC_MODE;
//     ledc_timer.timer_num        = LEDC_TIMER;
//     ledc_timer.duty_resolution  = LEDC_DUTY_RES;
//     ledc_timer.freq_hz          = LEDC_FREQUENCY;
//     ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
//     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

//     // Prepare and then apply the LEDC PWM channel configuration
//     ledc_channel_config_t ledc_channel;
//     ledc_channel.speed_mode     = LEDC_MODE;
//     ledc_channel.channel        = LEDC_CHANNEL;
//     ledc_channel.timer_sel      = LEDC_TIMER;
//     ledc_channel.intr_type      = LEDC_INTR_DISABLE;
//     ledc_channel.gpio_num       = LEDC_OUTPUT_IO;
//     ledc_channel.duty           = 0;
//     ledc_channel.hpoint         = 0;
//     ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
//     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.01f)));
//     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

//     vTaskDelay(pdMS_TO_TICKS(5000));

//     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.95f)));
//     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

//     vTaskDelay(pdMS_TO_TICKS(5000));

//     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.999f)));
//     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

//     vTaskDelay(pdMS_TO_TICKS(5000));

//     while (true)
//     {
//         switch (board.getPowerInput())
//         {
//         case PowerFeather::Board::PowerInput::Battery:
//             ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.999f)));
//             ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
//             printf("battery\n");
//             break;

//         case PowerFeather::Board::PowerInput::USB:
//             ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.95f)));
//             ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
//             printf("usb\n");
//             break;

//         case PowerFeather::Board::PowerInput::DC:
//             ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(0.01f)));
//             ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
//             printf("adapter\n");
//             break;
        
//         default:
//             break;
//         }
//     }
// }

// void test7()
// {
//     while (true)
//     {
//         bool battery = board.isBatteryConnected();
//         gpio_set_level(PowerFeather::Board::Signal::LED, battery);
//         printf("battery: %d\n", battery);
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }



    // test1();

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