#include <PowerFeather.h>

// Battery capacity in mAh; set to 0 when no battery is connected.
#define BATTERY_CAPACITY 500

using namespace PowerFeather; // for PowerFeather::Board

bool inited = false;

void setup()
{
  pinMode(Mainboard::Pin::BTN, INPUT);
  pinMode(Mainboard::Pin::LED, OUTPUT);

  delay(1000);

  printf("===============================\n");
  printf("PowerFeather - Supply and Battery Info Example\n");
  printf("===============================\n");
  printf("\n\n");

  Result initResult = (BATTERY_CAPACITY == 0) ? Board.init() : Board.init(BATTERY_CAPACITY);
  if (initResult == Result::Ok) // check if initialization succeeded
  {
    printf("Board initialized successfully\n\n");
    Board.setBatteryChargingMaxCurrent(100.0f); // set max charging current to 100 mA
    inited = true;
  }
}

void loop()
{

  if (inited)
  {
    // Toggle green user LED
    digitalWrite(Mainboard::Pin::LED, !(digitalRead(Mainboard::Pin::LED)));

    // Only enable charging when button is pressed.
    // When charging is enabled, red CHG LED turns on.
    Board.enableBatteryCharging(digitalRead(Mainboard::Pin::BTN) == LOW); // BTN is LOW when pressed

    // Get information about supply and battery
    float supplyVoltage = 0.0f, batteryVoltage = 0.0f;
    float supplyCurrent = 0.0f, batteryCurrent = 0.0f;
    uint8_t batteryCharge = 0;

    Board.getSupplyVoltage(supplyVoltage);
    Board.getSupplyCurrent(supplyCurrent);

    Board.getBatteryVoltage(batteryVoltage);
    Board.getBatteryCurrent(batteryCurrent);

    printf("[Supply]  Voltage: %.3f V    Current: %.1f mA\n", supplyVoltage, supplyCurrent);
    printf("[Battery] Voltage: %.3f V    Current: %.1f mA    ", batteryVoltage, batteryCurrent);

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
