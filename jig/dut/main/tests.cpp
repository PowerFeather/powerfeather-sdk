#include "tests.h"


void test_gpio_pins()
{
    /**
     * Test that pins are brought out properly. Pair up pins,
     * one generates a frequency and the other measures.
    */
}

void test_vusb_vdc()
{
    /**
     * Test the transition from VDC->VUSB and vice versa.
     * Test that VDC and VUSB can be enabled at the same time.
    */
}

void test_vbat()
{
    /**
     * Enable VBAT regulator output. Read the voltage using charger and fuel guage.
     */
}

void test_qon()
{
    /**
     * Hold QON for 10s, see if it resets.
     * 
     * 1 digital.
    */
}

void test_i2c()
{

}

void test_en()
{
    /**
     * Pull down from header pin, read value.
     * Pull down using EN0, read value.
     * 
     * 1 digital
    */

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
     * 3 analog-in, 2 digital
    */
}

void test_ts_voltage()
{
    /**
     * Test the voltage of the TS PIN is expected from the
     * voltage divider.
    */
}