
#include <cstdbool>
#include <cstdint>

namespace PowerFeather
{
    class Board
    {
    public:

        enum class PowerInput
        {
            USB,
            DC,
            Battery,
        };

        // The board has two 3.3V rails which can be individually switched off/on.
        enum class PowerOutput
        {
            Header5V,
            Header3V3,
            Stemma3V3
        };

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

        void Initialize(uint16_t batteryCapacity, uint16_t maxInputCurrent = 500, uint16_t maxChargeCurrent = 100);

        // By default, 3V3 is enabled even in sleep.
        // This allows the application to disable it before going to deep sleep.
        // The 5V rail can be disabled even when there is extenal power source plugged in.
        // This is remembered during sleep, and 5V remains off if it was previously disabled.
        void SetPowerOutputEnabled(Board::PowerInput input, bool state);

        // Source can either be:
        //  1. USB - board is drawing current from USB
        //  2. DC - board is drawing current from external power adapter
        //  3. BAT - no external power source, board is drawing current from battery
        PowerInput GetPowerInput();

        // Read the state of the EN pin. It can be pulled low
        // either by external circuitry or by the ESP32-S3 itself through SetEN.
        // Up to the developer what is done when this is detected, recommend shutting all
        // power rails and going to deep sleep.
        bool GetEN();

        // Set the state of the EN pin. For disabling wings that might be attached to
        // the board.
        void SetEN(bool state);

        // This disables charging. If the battery is not full, the charger will not
        // attempt to charge the battery even if there is an external source powering the device.
        // If the device is full, enable charging does not force the device to start charging.
        // It only makes sure that if the device is discharged, the charger is able to charge the
        // battyer.
        void SetChargingEnabled(bool enable);

        // Puts the device in ship mode, disenaging the battery.
        // The only way to disable ship mode is to press the RST button, hence the absence of
        // parameter.
        void SetShipMode();

        /**
         * Gauge
        */
        bool GetBatteryCurrent();

        // Exit gauge shutdown mode. GetBatteryLevel, GetBatteryHealth, and
        // GetBatteryTimeLeft will not work if gauge is shutdown.
        void SetGaugeEnabled(bool enable);

        // Charge left in the battery expressed as fraction of the battery capacity.
        // 0 - if no battery capacity left, or no battery, or gauge is shutdown
        float GetBatteryCharge();

        // Fraction of the original design battery capacity left.
        // 0 - faulty battery, no battery, or gauge is shutdown
        float GetBatteryHealth();

        // float
        float GetBatteryTemperature();

    private:
        uint16_t _batteryCapacity;

        size_t readI2C(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len);
        size_t writeI2C(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len);
    };
}
