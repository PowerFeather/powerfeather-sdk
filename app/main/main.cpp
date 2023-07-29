#include <stdio.h>
#include <sdkconfig.h>

#include <esp_sleep.h>
#include <esp_private/system_internal.h>

#include <Board.h>

PowerFeather::Board board;

#define microseconds(x)         (x * 1000000)

void test1()
{
    board.enableHeader3V3(true);
    board.enableStemmaQT3V3(true);
    esp_deep_sleep(microseconds(5));
}

void test2()
{
    board.enableHeader3V3(false);
    board.enableStemmaQT3V3(false);
    esp_restart();
}

void test3()
{
    board.enableHeader3V3(true);
    board.enableStemmaQT3V3(true);
    esp_restart_noos_dig();
}

void test3()
{
    board.enableHeader3V3(true);
    board.enableStemmaQT3V3(true);
    esp_restart_noos_dig();
}

extern "C" void app_main(void)
{
    board.init();
    // test1();a

    board.enableHeader3V3(false);
    board.enableStemmaQT3V3(false);
    esp_deep_sleep_start();
}

