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
    PowerFeather::Board board;
    board.Initialize(1000);

    printf("PowerFeather!\n");

    while (true)
    {
        // Display the power source
        PowerFeather::Board::PowerSource power_source = board.GetPowerSource();

        // Battery status
        //  - is charging/discharghing
        //  - charging/discharging current
        //  - is battery connected
        //  - temperature
        //  - state of charge
        //  - estimated health

        // Put into low power mode
        //  - enable wake source
        //  - disable 3V3 and 5V
        //  - disable 

        // Sleep for 1s
        esp_deep_sleep_start();

        // Re-enable power sources
        board.Enable3V3(true);
    }
}

