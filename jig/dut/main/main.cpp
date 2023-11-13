#include <unity.h>
#include <Arduino.h>

extern "C" void test_board_init();
extern "C" void test_3V3();
extern "C" void test_en();

extern "C" void app_main(void)
{
    test_board_init();
    test_3V3();
    test_en();
}