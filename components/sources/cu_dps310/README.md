# cu_dps310

Copper source driver for Infineon DPS310 barometer.

The source emits `cu_sensor_payloads::BarometerPayload` and uses per-device I2C
resources that hide hardware addresses in the board bundle.
