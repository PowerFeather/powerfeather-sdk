#include <PowerFeather.h>

// Battery capacity in mAh; a value of 0 means there is no battery.
// Replace with the capacity of the battery connected to PowerFeather.
#define BATTERY_CAPACITY    0 

using namespace PowerFeather; // For PowerFeather::Board

void setup() {
  Board.init(BATTERY_CAPACITY);
  printf("Hello, PowerFeather!\n");
}

void loop() {

  // Get about supply
  uint16_t supplyVoltage = 0;
  int16_t supplyCurrent = 0;
  Board.getSupplyVoltage(supplyVoltage);
  Board.getSupplyCurrent(supplyCurrent);

  printf("Supply voltage: %d mV\nSupply current: %d mA\n", supplyVoltage, supplyCurrent);

  // Get information about battery
  uint16_t batteryVoltage = 0;
  int16_t batteryCurrent = 0;
  uint8_t batteryCharge = 0;
  Board.getBatteryVoltage(supplyVoltage);
  Board.getBatteryCurrent(supplyCurrent);
  Board.getBatteryCurrent(supplyCurrent);

  printf("\n");

  delay(1000);
}
