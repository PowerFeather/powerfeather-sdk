
#include <cstdbool>
#include <cstdint>

namespace PowerFeather
{
    class Board
    {
    public:
        static constexpr uint8_t Enable3V3HeaderPin = 4;
        static constexpr uint8_t Enable3V3StemmaPin = 14;
        static constexpr uint8_t ChargeEnablePin = 7;
        static constexpr uint8_t EnablePin = 13;
        static constexpr uint8_t SCL0Pin = 47;
        static constexpr uint8_t SDA0Pin = 48;
        static constexpr uint8_t GPOUTPin = 21;
        static constexpr uint8_t VDDTypePin = 38;
        static constexpr uint8_t InterruptPin = 5;

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

        enum class BatteryModeHeader5V
        {
            None, // Disabled when PowerInput = Battery
            Bypass, // Battery value when 
            Reg,
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
        Board(uint16_t batteryCapacity, bool useTSPin);

        bool init();

        bool setBatteryModeHeader5V(Board::BatteryModeHeader5V mode, float voltage = 5.0f);
        bool setVoltageHeader5V(float voltage);

        bool setChargeFactor(float factor);

        // By default, 3V3 is enabled even in sleep.
        // This allows the application to disable it before going to deep sleep.
        // The 5V rail can be disabled even when there is extenal power source plugged in.
        // This is remembered during sleep, and 5V remains off if it was previously disabled.
        void EnablePowerOutput(Board::PowerOutput output, bool state);

        // Source can either be:
        //  1. USB - board is drawing current from USB
        //  2. DC - board is drawing current from external power adapter
        //  3. Battery - no external power source, board is drawing current from battery
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
        bool _useTSPin;
        uint8_t _i2cNum;

        bool _enableStatLed(bool enable);
        bool _enableTS(bool enable);
        uint8_t _getChargerFault();

        bool _readI2C(uint8_t address, uint8_t reg, uint8_t *data);
        bool _writeI2C(uint8_t address, uint8_t reg, uint8_t data);
        bool _setChargerRegister(uint8_t address, uint8_t bit, bool value);
    };
}
