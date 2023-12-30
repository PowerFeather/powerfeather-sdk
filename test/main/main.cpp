#include "test_utils.h"

#include <unity.h>

#include <PowerFeather.h>

using namespace PowerFeather;

extern "C" void app_main(void)
{
    TEST_ASSERT_EQUAL(Result::Ok, Board.init(650));

    // Tie potentiometer to temperature sense
    // Check interrupt, may be combined with another test
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((uint64_t)0b1 << MainBoard::Pin::LED);
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (uint64_t)0b1 << MainBoard::Pin::BTN;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(MainBoard::Pin::LED, true);

    test_main();
}