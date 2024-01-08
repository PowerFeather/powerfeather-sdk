#include <test_utils.h>
#include <esp_event.h>

#include <unity.h>

#include <PowerFeather.h>

using namespace PowerFeather;

extern "C" void app_main(void)
{
    TEST_ASSERT_EQUAL(Result::Ok, Board.init(650));

    TEST_ASSERT_EQUAL(ESP_OK, esp_event_loop_create_default());

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((uint64_t)0b1 << MainBoard::Pin::LED);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = ((uint64_t)0b1 << MainBoard::Pin::BTN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    gpio_set_level(MainBoard::Pin::LED, true);
    test_main();
}