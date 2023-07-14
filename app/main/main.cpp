/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <sdkconfig.h>

#include <esp_sleep.h>

#include <BQ2562x.hpp>
#include <BQ27441_Definitions.h>
#include <driver/adc.h>
#include <esp_wifi.h>


// #include <Board.hpp>
void print_bytes(BQ2562x &bq, uint8_t address)
{
    uint8_t data[1] = { 0 };
    bq.i2cReadBytes(address, data, sizeof(data));

    for (int i = 0; i < sizeof(data); i++)
    {
        printf("%d ", data[i]);
    }
    printf("\n");
}

#include <esp_sleep.h>

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */

extern "C" void app_main(void)
{
    BQ2562x bq;

    uint8_t data = 0;

    bq.i2cReadBytes(0x15, &data, sizeof(data));
    data |= 0b10000000;
    bq.i2cWriteBytes(0x15, &data, sizeof(data));

    bq.i2cReadBytes(0x16, &data, sizeof(data));
    data |= 0b00010000;
    bq.i2cWriteBytes(0x16, &data, sizeof(data));

    // esp_wifi_stop();
    // adc_power_off();
    // esp_sleep_enable_timer_wakeup(1 * uS_TO_S_FACTOR);
    esp_deep_sleep_start();

    // while(true)
    // {

    // }

	// int16_t timeout = BQ72562x_I2C_TIMEOUT;	


	// Wire.endTransmission(true);
	
	// Wire.requestFrom(_deviceAddress, count);
	
	// for (int i=0; i<count; i++)
	// {
	// 	dest[i] = Wire.read();
	// }
	
	// return timeout;


    // PowerFeather::Board board;
    // board.Initialize(1000);

    // printf("PowerFeather!\n");

    // while (true)
    // {
        // // Display the power source
        // PowerFeather::Board::PowerSource power_source = board.GetPowerSource();

        // // Battery status
        // //  - is charging/discharghing
        // //  - charging/discharging current
        // //  - is battery connected
        // //  - temperature
        // //  - state of charge
        // //  - estimated health

        // // Put into low power mode
        // //  - enable wake source
        // //  - disable 3V3 and 5V
        // //  - disable 

        // // Sleep for 1s
        // esp_deep_sleep_start();

        // // Re-enable power sources
        // board.Enable3V3(true);
    // }
}

