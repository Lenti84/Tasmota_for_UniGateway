# Solis Smart Meter Bridge

Enable `USE_SOLIS_METER` and `USE_SDM630_MULTI` (enabled in the local user_config_override.h).
The bridge uses the bundled TasmotaSerial library; no external libraries are required.

Assign **Solis Meter Tx**, **Solis Meter Rx** and **Solis Meter ENA** in the module GPIO configuration to a dedicated RS485 transceiver. ENA drives DE and active-low /RE together. The SDM630 Multi meter bus needs a separate serial interface. The bridge uses 9600 baud, 8N1, Modbus address 1 and function 04.

Supported requests are input registers 0x0000 / 76 registers and 0x0156 / 2 registers. Phase powers are negated SDM630 Multi meter 1 values; total power is negated grid power. SolisMeterSetMode and SolisMeterSetPower retain the upstream manual offset API (limited to +/-10000 W).

The upstream emulation still reports fixed 230 V phase voltages and placeholder energy readings (4444, 5555 and 1234 kWh). SDM630 Multi currently supplies power only. These energy readings are not measured consumption. Validate power direction on the inverter before operating automatic control.

TasmotaSerial uses hardware UARTs on ESP32. Ensure enough UARTs remain available for all configured serial drivers. Initialization failure is reported in the log. The connection indicator expires ten seconds after the last valid supported request. Hardware communication must be verified with the inverter.
