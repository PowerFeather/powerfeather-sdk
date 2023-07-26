/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <sdkconfig.h>

#include <esp_sleep.h>

#include <Board.hpp>

PowerFeather::Board board(320);

extern "C" void app_main(void)
{
    board.init();
}

