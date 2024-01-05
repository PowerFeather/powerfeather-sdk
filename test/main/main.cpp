#include <test_utils.h>
#include <esp_event.h>

#include <unity.h>

#include <PowerFeather.h>

using namespace PowerFeather;

ESP_EVENT_DEFINE_BASE(TEST_EVENTS);

#define CHARGER_INTERRUPT           1
#define FUEL_GAUGE_ALARM            2

static void display_charger_status(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    Board.getCharger().displayInfo();
}

static void display_fuel_gauge_status(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
}

extern "C" void app_main(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, esp_event_loop_create_default());
    TEST_ASSERT_EQUAL(ESP_OK, esp_event_handler_instance_register(TEST_EVENTS, CHARGER_INTERRUPT, display_charger_status, NULL, NULL));
    TEST_ASSERT_EQUAL(ESP_OK, esp_event_handler_instance_register(TEST_EVENTS, CHARGER_INTERRUPT, display_fuel_gauge_status, NULL, NULL));

    TEST_ASSERT_EQUAL(Result::Ok, Board.init());

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

    io_conf.pin_bit_mask = ((uint64_t)0b1 << MainBoard::Pin::FGA) | ((uint64_t)0b1 << MainBoard::Pin::CHGI);
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(MainBoard::Pin::CHGI, [](void* arg) { esp_event_post(TEST_EVENTS, CHARGER_INTERRUPT, NULL, 0, portMAX_DELAY); }, NULL);
    gpio_isr_handler_add(MainBoard::Pin::CHGI, [](void* arg) { esp_event_post(TEST_EVENTS, FUEL_GAUGE_ALARM, NULL, 0, portMAX_DELAY); }, NULL);

    gpio_set_level(MainBoard::Pin::LED, true);
    test_main();
}