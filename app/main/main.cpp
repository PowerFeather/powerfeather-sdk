/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <sdkconfig.h>

#include <Board.hpp>

extern "C" void app_main(void)
{
    Board board(1000);

    printf("Hello World!\n");
}

