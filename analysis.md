# Analysis

Ranked by severity (highest to lowest). Tags in brackets indicate area.

## Critical
- [battery] Wrong battery chemistry selected (charge voltage/profile mismatch).

## High
- [board] Wrong board revision build (MAX17260 vs LC709204F) not detected early.
- [board] No runtime board ID check on ESP-IDF (wrong pin map/I2C bus).
- [board] Compile-time gauge selection only; no runtime probe/fallback.
- [charger] Charger reset while watchdog disabled; init path skips reapplying settings.
- [charger] Charger IC identity/availability not verified before use.
- [battery] Fuel gauge init short-circuits once initialized; no re-init after battery/chemistry swap.
- [battery] Battery disconnected then reconnected (gauge state can stay stale).
- [init] VSQT toggled off/on can reset charger/gauge with no re-init.
- [init] init or battery APIs called before scheduler or from ISR (uses vTaskDelay/mutex).
- [init] Multiple init calls recreate the mutex without guard.
- [init] Deep-sleep wake with VSQT held low skips I2C/device init; later ops fail.

## Medium
- [battery] Capacity set to 0 prevents later battery operations even if a battery is inserted.
- [battery] No battery present at boot (fuel gauge init can fail with capacity set).
- [battery] Profile init with designCap 0 or non-mappable capacity returns Ok but leaves battery disabled.
- [battery] Gauge read failures have no degraded path or NotReady result.
- [charger] ADC reads not ready or invalid (ADC_DONE false or IBAT invalid).
- [charger] Thermistor open/short or bias out of range (temperature calc always runs).
- [charger] Ship/shutdown called while supply is good; precondition not checked.
