/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <sdkconfig.h>

#include <esp_sleep.h>

#include <Board.hpp>

extern "C" void app_main(void)
{
    Board board;
    board.Initialize(1000);

    board.Enable3V3(true);
    printf("Hello World!\n");
    // esp_deep_sleep_start();
}

