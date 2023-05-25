#include <esp32-hal-i2c.h>
#include <esp32-hal-gpio.h>

#include "Board.hpp"

Board::Board(uint16_t batteryCapacity, uint16_t maxInputCurrent, uint16_t maxChargeCurrent)
{
    this->_batteryCapacity = batteryCapacity;

    /**
     * Initialize pins
    */
    // Input

    // Output
    pinMode(ENABLE_3V3, OPEN_DRAIN);
    pinMode(CE, OPEN_DRAIN);

    // Input-Output
    pinMode(EN, INPUT | OUTPUT | OPEN_DRAIN);

    /**
     * Initialize communication peripherals
    */

    /**
     * Set initial state
    */
    this->Enable3V3(true);
    this->Enable5V(true);
    this->SetEN(true);
    this->EnableCharging(true);
    this->EnableGauge(true);
}

void Board::Enable3V3(bool enable)
{
    digitalWrite(ENABLE_3V3, enable);
}

void Board::Enable5V(bool enable)
{

}

bool Board::GetEN()
{
    return digitalRead(EN);
}

void Board::SetEN(bool enable)
{
    digitalWrite(EN, enable);
}

void Board::EnableCharging(bool enable)
{
    digitalWrite(ENABLE_3V3, enable);
}

void Board::EnableGauge(bool enable)
{

}

void Board::EnterShipMode()
{

}

void Board::GetPowerSource()
{
}