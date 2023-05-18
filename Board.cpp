#include "Board.hpp"
#include "esp32-hal-i2c.h"

Board::Board(uint16_t batteryCapacity, uint16_t maxInputCurrent, uint16_t maxChargeCurrent)
{
    this->_batteryCapacity = batteryCapacity;

    this->Enable3V3(true);
    this->Enable5V(true);
    this->SetEN(true);
    this->EnableCharging(true);
    this->EnableGauge(true);
}

void Board::Enable3V3(bool enable)
{

}

void Board::Enable5V(bool enable)
{

}

void Board::SetEN(bool enable)
{

}

void Board::EnableCharging(bool enable)
{

}

void Board::EnableGauge(bool enable)
{

}