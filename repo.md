# MAX17260 Support Status

The support for **MAX17260** is integrated into the SDK, but it is **not yet feature-complete**. The driver and selection logic exist, however there are gaps in documentation, examples, and validation, and some aspects of detection/configuration are still brittle.

### Summary of Effort (What Exists)

1. **Driver Implementation (`src/Mainboard/MAX17260.{cpp,h}`)**
   - Implements `MAX17260` (derived from `RegisterFuelGauge`) with I2C access, basic telemetry, alarms, enable/TSENSE, and termination current.
   - Supports model loading and model ID selection with DNR wait handling.

2. **SDK Integration (`src/Mainboard/Mainboard.cpp`)**
   - Auto-detects LC709204F vs MAX17260 and selects a fuel gauge at runtime.
   - LFP batteries prioritize MAX17260 and apply a model ID.
   - `Mainboard::init(const MAX17260::Model &profile)` allows passing a custom `MAX17260::Model`.

### Gaps / Risks (Why It Is Not Feature-Complete)

- **No shipped profile data or example usage** for `Mainboard::init(const MAX17260::Model &profile)`, so the API is hard to use in practice.
- **Documentation is missing** (README/examples do not mention MAX17260 behavior, profile loading, or selection rules).
- **Probe logic is minimal** (relies on POR bit read), which risks false positives.
- **Magic values are embedded** (model IDs and many register addresses without named constants or notes).
- **Limited validation** (no known hardware verification or test artifacts in repo).

### Implementation Plan (Easiest -> Hardest)

1. **Docs update**
   - Add README section for MAX17260: detection order, LFP behavior, profile loading.
   - Add a short note in examples or a minimal snippet showing `Mainboard::init(const MAX17260::Model &profile)`.

2. **Constants cleanup**
   - Replace hardcoded model IDs with named constants in `MAX17260.h`.
   - Introduce named register constants for frequently used “magic addresses” to improve readability.

3. **Example + profile scaffolding**
   - Add a minimal profile header/source with a sample `MAX17260::Model`.
   - Add an example that passes the profile through `Mainboard::init`.

4. **Probe hardening**
   - Update `probe()` to check a stable ID register or known register defaults to avoid false positives.
   - Ensure `_fuelGaugeMax.init()` only runs when MAX17260 is detected.

5. **Hardware verification**
   - Validate telemetry scaling, model loading sequence, and alarm behavior on real hardware.
   - Capture findings and update docs (and code) based on behavior.
