#include <stdio.h>
#include <sdkconfig.h>

#include <esp_sleep.h>

#include <Board.h>

PowerFeather::Board board;

#define microseconds(x)         (x * 1000000)

void test1()
{
    board.enableHeader3V3(true);
    board.enableStemmaQT3V3(true);
    board.setEN(true);
    esp_deep_sleep(microseconds(5));
}

void test2()
{
    board.enableHeader3V3(false);
    board.enableStemmaQT3V3(false);
    board.setEN(true);
    esp_deep_sleep(microseconds(5));
}

void enablePowerOutputAndCrash()
{
    board.enableHeader3V3(true);
    board.enableStemmaQT3V3(true);
    board.setEN(true);
    abort();
}

extern "C" void app_main(void)
{
    board.init();
}

