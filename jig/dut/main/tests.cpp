#include <cstdint>

#include <Arduino.h>

#include <MainBoard.h>

using namespace PowerFeather;

MainBoard& board = Board;

static void test(bool assertion, const char* error)
{
}

extern "C" {

/**
 * Test communication with charger and fuel gauge; as well as default
 * configuration of SDK-managed pins.
 */
void test_board_init()
{
    board.init();
}

void test_ref_analog()
{
    /**
     * Read a reference analog voltage as reference.
     * Make sures that analog read is working properly.
     * 
    */
}

void test_internal_3V3()
{
    /**
     * Test the supply voltage.
    */

   #define TEST_INT_3V3_PIN 0

}

/**
 * Check that 3V3 can be enabled/disabled.
*/
void test_3V3()
{
   uint16_t val = 0;

   board.enable3V3(false);
   vTaskDelay(pdMS_TO_TICKS(100));
   val = analogRead(A0);
   printf("val: %d volt: %.02f\n", val, (val/4095.0f) * 3.3f);

   board.enable3V3(true);
   vTaskDelay(pdMS_TO_TICKS(100));
   val = analogRead(A0);
   printf("val: %d volt: %.02f\n", val, (val/4095.0f) * 3.3f);
}

void test_SQT()
{
    /**
     * Test SQT enable, disable. Configure SDA and SCL pin
     * as input with pull downs. Read voltage.
    */

}


void test_ts()
{
    /**
     * Test the voltage of the TS PIN is expected from the
     * voltage divider.
    */
}

void test_en()
{
    /**
     * Pull down from header pin, read value.
     * Pull down using EN0, read value.
     * 
     * 
     * D13 <-> EN
    */

   uint16_t val = 0;

   board.setEN(false);
   vTaskDelay(pdMS_TO_TICKS(100));
   val = analogRead(EN);
   printf("val: %d volt: %.02f\n", val, (val/4095.0f) * 3.3f);

   board.setEN(true);
   vTaskDelay(pdMS_TO_TICKS(100));
   val = analogRead(EN);
   printf("val: %d volt: %.02f\n", val, (val/4095.0f) * 3.3f);
}



void test_power_input_output()
{
    /**
     * Test power outputs voltage level:
     * 1. 3V3
     * 2. VBAT
     * 3. VS
     * 
     * With power inputs:
     * A. VDC
     * B. VUSB
     * 
     * VBAT_EN <-> D11
     * VDC_EN <-> D7
     * VUSB_EN <-> D12
    */
}

void test_gpio_pins()
{
    /**
     * Test that pins are brought out properly. Pair up remaining pins,
     * one generates a frequency and the other measures.
     * 
     * 
     * LED - BTN
     * USBDP - USBDM
    */
}

void test_qon()
{
    /**
     * Hold QON for 10s, see if it resets.
     * 
     * QON <->
    */
}
}