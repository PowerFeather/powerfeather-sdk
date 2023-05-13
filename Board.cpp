#include "Board.hpp"
#include "esp32-hal-i2c.h"

Board::Board(uint16_t batteryCapacity, uint16_t maxChargeCurrent)
{
    i2cInit(0, 0, 0, 0);
}