#include <stdio.h>
#include <sdkconfig.h>

#include <esp_sleep.h>

#include <Board.h>

PowerFeather::Board board;

extern "C" void app_main(void)
{
    board.init();
}

