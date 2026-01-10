#include <PowerFeather.h>

// Battery capacity in mAh; set to 0 when no battery is connected.
#define BATTERY_CAPACITY 500

using namespace PowerFeather; // for PowerFeather::Board

bool inited = false;

void setup()
{
  pinMode(BTN, INPUT);
  pinMode(LED, OUTPUT);

  delay(1000);

  printf("===============================\n");
  printf("PowerFeather - Supply and Battery Info Example\n");
  printf("===============================\n");
  printf("\n\n");

#if BATTERY_CAPACITY == 0
  if (Board.init() == Result::Ok) // check if initialization succeeded
#else
  if (Board.init(BATTERY_CAPACITY) == Result::Ok) // check if initialization succeeded
#endif
  {
    printf("Board initialized successfully\n\n");
    Board.setBatteryChargingMaxCurrent(100); // set max charging current to 100 mA
    inited = true;
  }
}

void loop()
{

  if (inited)
  {
    // Toggle green user LED
    digitalWrite(LED, !(digitalRead(LED)));

    // Only enable charging when button is pressed.
    // When charging is enabled, red CHG LED turns on.
    Board.enableBatteryCharging(digitalRead(BTN) == LOW); // BTN is LOW when pressed

    // Get information about supply and battery
    uint16_t supplyVoltage = 0, batteryVoltage = 0;
    int16_t supplyCurrent = 0, batteryCurrent = 0;
    uint8_t batteryCharge = 0;

    Board.getSupplyVoltage(supplyVoltage);
    Board.getSupplyCurrent(supplyCurrent);

    Board.getBatteryVoltage(batteryVoltage);
    Board.getBatteryCurrent(batteryCurrent);

    printf("[Supply]  Voltage: %d mV    Current: %d mA\n", supplyVoltage, supplyCurrent);
    printf("[Battery] Voltage: %d mV    Current: %d mA    ", batteryVoltage, batteryCurrent);

    // Check the result for getting battery charge.
    Result res = Board.getBatteryCharge(batteryCharge);

    if (res == Result::Ok)
    {
      printf("Charge: %d %%\n", batteryCharge);
    }
    else if (res == Result::InvalidState)
    {
      printf("Charge: <no battery configured>\n");
    }
    else
    {
      printf("Charge: <battery not detected>\n");
    }
  }
  else
  {
    printf("Board not initialized\n");
  }

  printf("\n");
  delay(500);
}
