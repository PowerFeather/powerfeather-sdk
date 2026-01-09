# FuelGauge Comparison: LC709204F vs MAX17260

Both drivers implement all `FuelGauge` methods, but the meaning and scaling of
many values differ. Below is a method-by-method comparison based on the current
implementations in `src/Mainboard/LC709204F.cpp` and `src/Mainboard/MAX17260.cpp`.

## Telemetry (read)

- `getCellVoltage`: LC returns the register value directly (mV in chip format). MAX converts raw to mV using `raw * 5 / 64`.
- `getRSOC`: LC returns the register value as percent. MAX returns the MSB of `RepSOC` with rounding and clamps to 100%.
- `getTimeToEmpty` / `getTimeToFull`: LC returns raw register values. MAX converts to minutes using `raw * 3 / 32`.
- `getCellTemperature`: LC reads TSENSE1 and applies a 0.1 C scale with an 80 C offset. MAX uses signed 1/256 C.
- `getCycles`: LC returns the register value directly. MAX divides raw by 100 with rounding.
- `getSOH`: LC reads `State_Of_Health` directly. MAX computes from `FullCapRep / DesignCap`.

## Status and control

- `probe`: LC reads `IC_Power_Mode` (CRC-protected). MAX reads the `Status.POR` field.
- `getEnabled`: LC checks `IC_Power_Mode` (Operational vs Sleep). MAX checks `Config.SHND` bit.
- `setEnabled`: LC writes `IC_Power_Mode`. MAX writes `Config.SHND`.
- `getInitialized`: LC checks BatteryStatus Initialized bit (0 means initialized). MAX checks `Status.POR` (0 means initialized).
- `setInitialized`: LC clears BatteryStatus Initialized bit. MAX clears `Status.POR`.
- `setCellTemperature`: LC writes TSENSE1 using 0.1 C scale and 80 C offset. MAX writes signed 1/256 C.
- `enableTSENSE`: LC writes `Status_Bit` with TSENSE1/2 flags. MAX adjusts multiple `Config` bits (TEn/TEx/TSel/ETHRM).
- `setTerminationFactor`: LC writes `Termination_Current_Rate` as `factor * 100` within 0.02 to 0.3. MAX computes `IChgTerm` from `DesignCap * factor * 3.2`.

## Alarms

- `setLowVoltageAlarm` / `setHighVoltageAlarm`: LC writes mV directly. MAX encodes in `VAlrtTh` with ~20 mV steps; 0 disables.
- `setLowRSOCAlarm`: LC writes percent directly. MAX updates the low byte of `SAlrtTh`; 0 disables.
- `clearLowVoltageAlarm` / `clearHighVoltageAlarm` / `clearLowRSOCAlarm`: LC clears BatteryStatus bits. MAX clears Status bits.

## Ranges and limits reported by the drivers

- `getVoltageAlarmRange`: LC 2500 to 5000 mV; MAX 0 to 5120 mV.
- `getTemperatureRange`: LC -30 to 80 C; MAX -128 to 127.996 C.
- `getTerminationFactorRange`: LC 0.02 to 0.3; MAX 0.01 to 1.0.
- `getBatteryCapacityRange`: LC 50 to 6000 mAh; MAX 1 to ~16383 mAh (20 mΩ sense resistor).

## Notes

- LC709204F uses CRC8 on register reads/writes; MAX17260 does not.
- Both drivers use 16-bit registers, but the scaling is chip-specific as shown above.
