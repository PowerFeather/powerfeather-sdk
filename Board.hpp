
#include <cstdbool>
#include <cstdint>

class Board
{
public:

    // Initialize the board
    // batteryCapacity - advertised capacity of the battery
    // chargeRate - charge rate of the battery expressed as fraction of the capacity, 
    //              ex. 2000mAh, 0.5 charge rate = 1000mA charge current
    //
    // event handler - when significant events occur
    //      - change in external power source
    //      - enable pin pulled down low
    //      - charging started/stopped
    //
    //
    //      - temperature too high or too low (hardware set)
    //      - battery state of charge low
    //      - input current exceeded
    //      - charge current exceeded
    //
    //  charger status
    //      - charging changed
    //      - vbus changed
    //  charger fault
    //      - bat overvoltage or overcurrent
    //      - sys overvoltage or short
    //      - otg overvoltage,
    //      - tshut thermal shutdown
    //
    // take OCV for gauge, then enable charging
    // enable 3V3 by default
    // enable 5V by default

    Board(uint16_t batteryCapacity, uint16_t maxInputCurrent = 500, uint16_t maxChargeCurrent = 100);

    // By default, 3V3 is enabled even in sleep.
    // This allows the application to disable it before going to deep sleep.
    void Enable3V3(bool enable);

    // By default, 5V is enabled even in sleep.
    // This allows the application to disable it before going to deep sleep.
    void Enable5V(bool enable);

    // Read the state of the EN pin. It can be pulled low
    // either by external circuitry or by the ESP32-S3 itself.
    bool GetEN();

    // Set the state of the EN pin.
    void SetEN(bool state);

    // Source can either be:
    //  1. USB - board is drawing current from USB
    //  2. DC - board is drawing current from external power adapter
    //  3. Battery - no external power source, board is drawing current from battery
    void GetPowerSource();

    /**
     * Charger
     *
    */
    // This disables charging. If the battery is not full, the charger will not
    // attempt to charge the battery even if there is an external source powering the device.
    // If the device is full, enable charging does not force the device to start charging.
    // It only makes sure that if the device is discharged, the charger is able to charge the
    // battyer.
    void EnableCharging(bool enable);

    // Check if currently charging the battery.
    bool IsCharging();

    // Puts the device in ship mode, disenaging the battery. This cannot be exited
    // out until external power is applied, or the RST button is pressed.
    void EnterShipMode();

    /**
     * Gauge
    */
    // Exit gauge shutdown mode. GetBatteryLevel, GetBatteryHealth, and
    // GetBatteryTimeLeft will not work if gauge is shutdown.
    void EnableGauge(bool enable);

    // Charge left in the battery expressed as fraction of the battery capacity.
    // 0 - if no battery capacity left, or no battery, or gauge is shutdown
    float GetChargeLevel();

    // Fraction of the original design battery capacity left.
    // 0 - faulty battery, no battery, or gauge is shutdown
    float GetBatteryHealth();

    // Estimate the time left for charging the battery or discharging the battery given
    // the current draw/charge rate.
    float EstimateBatteryTimeLeft();

private:
    uint16_t _batteryCapacity;
};