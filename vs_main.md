# v2_ai_test vs main: Fuel Gauge and Charger Behavior

This compares the current branch against `main` for fuel gauge and charger behavior
(init, use, retry, detection). References point to current files unless noted.

## Fuel gauge selection and detection
- main: Always uses LC709204F. `Mainboard::getFuelGauge()` returns a concrete `LC709204F &`
  and there is no detection logic. (`main:src/Mainboard/Mainboard.h`)
- v2_ai_test: Gauge is selected at compile time via `FuelGaugeImpl` in
  `src/Mainboard/Mainboard.h` (Kconfig macro `CONFIG_ESP32S3_POWERFEATHER_V2` or
  `POWERFEATHER_BOARD_V2`). There is no runtime detection; `FuelGauge::probe()` exists
  but is unused. Wrong build selection results in I2C failures talking to the wrong gauge.

## Fuel gauge initialization
- main: `_initFuelGauge()` is LC709204F-specific. It checks `getInitialized`, then performs
  `setAPA`, `setChangeOfParameter`, computes termination factor using LC709204F limits,
  disables TSENSE, enables operation mode, and sets initialized. (`main:src/Mainboard/Mainboard.cpp`)
- v2_ai_test: `_initFuelGauge()` is generic. It builds `FuelGauge::InitConfig` and calls
  `FuelGauge::init`, which:
  - Reads `getInitialized`; if already initialized it returns immediately.
  - Calls gauge-specific `initImpl` (LC709204F applies APA/ChangeOfParameter; MAX17260 loads
    a profile or sets model ID).
  - Calls `FuelGauge::_finalizeInit` for shared steps (termination factor clamped to the
    gauge’s min/max, TSENSE disabled, gauge enabled, set initialized). (`src/Mainboard/FuelGauge.h`,
    `src/Mainboard/LC709204F.cpp`, `src/Mainboard/MAX17260.cpp`)

## Profile and LFP behavior
- main: No profile or LFP support.
- v2_ai_test:
  - `Mainboard::init(const MAX17260::Model &profile)` stores a profile in the MAX17260
    instance and sets `FuelGauge::ProfileKind::Max17260` in the init config.
  - Capacity can be inferred from `profile.designCap`.
  - `profile.chargeVoltageMv` can override charger charge voltage.
  - `BatteryType::Generic_LFP` is supported only when the active gauge advertises
    `SupportsLfp` (MAX17260). LFP forces a 3.6 V charge voltage and uses MAX17260’s LFP model ID.

## Capacity validation and termination factor
- main: Capacity is validated against LC709204F constants (min/max). This caps max capacity
  to LC709204F limits. (`main:src/Mainboard/Mainboard.cpp`)
- v2_ai_test: Capacity is validated against the active gauge’s `getBatteryCapacityRange`.
  MAX17260 permits larger (and smaller) capacities based on RSENSE. Termination factor
  uses the gauge’s `getTerminationFactorRange` instead of LC709204F constants.

## Use and retry behavior (mostly unchanged)
- Both branches:
  - Fuel gauge init is attempted during `Mainboard::init` if capacity is non-zero, but failure
    is tolerated so boards can boot with no battery.
  - Subsequent APIs call `_initFuelGauge()` again, so gauge init is retried lazily when
    the battery/gauge becomes available.
  - `getBatteryVoltage` prefers the fuel gauge if enabled and initialized, otherwise
    falls back to the charger VBAT ADC path.
  - `enableBatteryFuelGauge(true)` also triggers `_initFuelGauge()` for retry.

## Fuel gauge usage differences
- main: Voltage/temperature/termination range checks use LC709204F constants.
- v2_ai_test: Range checks are pulled from the active gauge (`getVoltageAlarmRange`,
  `getTemperatureRange`, `getTerminationFactorRange`), so limits vary by gauge.
- main: `Mainboard::getFuelGauge()` exposes LC709204F-specific API to callers.
- v2_ai_test: `Mainboard::getFuelGauge()` returns a `FuelGauge &`, limiting calls
  to the common interface.

## Charger behavior differences
- main: Charger init sequence is unchanged from the LC709204F-only design and does not
  set a charge voltage limit explicitly (defaults apply).
- v2_ai_test: After charger init, `setChargeVoltageLimit` is called:
  - Default 4.2 V for Generic_3V7 and chemistry profiles without overrides.
  - 3.6 V for LFP.
  - `profile.chargeVoltageMv` for MAX17260 profile init.
- main: `setBatteryChargingMaxCurrent` checks against LC709204F min capacity.
- v2_ai_test: `setBatteryChargingMaxCurrent` checks against BQ2562x min current instead,
  decoupling the charger limit from the fuel gauge min capacity.

## Net effect summary
- v2_ai_test adds MAX17260 support, profile init, and LFP handling with charger voltage
  enforcement that does not exist on main.
- v2_ai_test generalizes gauge range checks and termination factor limits to the active
  gauge, while main is LC709204F-specific.
- Detection remains non-runtime in both branches; v2_ai_test relies on compile-time board
  selection rather than hard-coding LC709204F.

## Discrepancies and which is correct
- Fuel gauge selection:
  - Discrepancy: main hard-codes LC709204F; v2_ai_test selects at compile time.
  - Correct: compile-time selection is correct when the board revision is known at build time.
    For unknown/mixed hardware, runtime detection would be the correct model (neither branch does this).
- Charger charge voltage:
  - Discrepancy: main does not explicitly set a charge voltage limit; v2_ai_test sets it based
    on battery type/profile (including LFP 3.6 V).
  - Correct: v2_ai_test is correct for LFP and profile-driven chemistry; main is only correct
    if the default charger limit matches the intended chemistry (typical Li-ion).
- Capacity and termination factor limits:
  - Discrepancy: main validates and clamps against LC709204F constants; v2_ai_test uses
    the active gauge’s ranges.
  - Correct: v2_ai_test is correct for MAX17260 and any future gauge; main is only correct
    for LC709204F.
- Max charging current validation:
  - Discrepancy: main uses LC709204F min capacity as the lower bound; v2_ai_test uses
    BQ2562x min current.
  - Correct: v2_ai_test is correct because the charger IC defines the valid current range.
