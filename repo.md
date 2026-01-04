# MAX17260 Support Status

The support for **MAX17260** is currently **feature-complete** and fully integrated into the main SDK logic.

### Summary of Effort

1.  **Driver Implementation (`src/Mainboard/MAX17260.{cpp,h}`)**
    *   Implemented the full `MAX17260` class inheriting from `RegisterFuelGauge`.
    *   **Features:** Supports I2C communication, ModelGauge m5 algorithm configuration (model loading/model ID), and telemetry (Voltage, SOC, Time-to-empty/full, SOH, etc.).
    *   **Configuration:** Implemented mechanisms to wait for the "Data Not Ready" (DNR) bit during initialization and model loading.

2.  **SDK Integration (`src/Mainboard/Mainboard.cpp`)**
    *   **Auto-Detection:** The `Mainboard` now probes for both `LC709204F` and `MAX17260`.
    *   **Selection Logic:**
        *   **LFP Batteries:** Prioritizes `MAX17260` (Model ID 6).
        *   **Li-ion/Others:** Prioritizes `LC709204F` but falls back to `MAX17260` (Model ID 2) if detected.
    *   **Custom Profiles:** Added support for `BatteryType::Profile` to load custom `MAX17260::Model` structures for specific battery characterization.

### Next Steps

1.  **Hardware Verification**
    *   Test on physical hardware with a `MAX17260` to verify I2C stability and the startup sequence (specifically the `_waitForDNRClear` loop).
    *   Verify that `Mainboard::_selectFuelGauge` correctly identifies the chip when it is the only one present or when it is the preferred one.

2.  **Refactoring**
    *   **Magic Numbers:** Replace the hardcoded model IDs (`6` for LFP, `2` for others) in `Mainboard.cpp` with named constants (e.g., `ModelID_LFP`, `ModelID_LiPo`) in `MAX17260.h`.

3.  **Documentation & Examples**
    *   Add a simple example or comment block in `examples/` demonstrating how to use `BatteryType::Profile` with a custom model array, as this is a powerful but complex feature for the MAX17260.
