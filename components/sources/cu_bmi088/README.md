# cu-bmi088

## Overview

Copper source component for the BMI088 6-axis IMU (accelerometer + gyroscope). This crate is `no_std` by default and uses `embedded-hal` 0.2 traits for portability across embedded platforms.

The BMI088 is a high-performance inertial measurement unit from Bosch Sensortec, featuring:
- 16-bit triaxial accelerometer (±3g to ±24g range)
- 16-bit triaxial gyroscope (up to ±2000°/s)
- On-chip temperature sensor
- SPI interface (up to 10 MHz)

This driver is commonly used in flight controllers and robotics applications where precise motion sensing is required.

## Interface

### Output

The driver outputs [`cu_sensor_payloads::ImuPayload`] containing:
- Acceleration in m/s² (SI units)
- Angular velocity in rad/s (SI units)
- Temperature in °C

### Axis Mapping

The driver remaps BMI088 axes to NED (North-East-Down) body frame convention:
- +X: Forward
- +Y: Right
- +Z: Down

**Note**: The axis mapping in this driver assumes a specific board orientation. You may need to adjust the mapping in `read_measure()` for your hardware.

## Configuration

```ron
(
    id: "bmi088",
    type: "cu_bmi088::Bmi088Source<MySpi, MyAccCs, MyGyrCs, MyDelay>",
    resources: {
        "spi": "hal.bmi088_spi",
        "acc_cs": "hal.bmi088_acc_cs",
        "gyr_cs": "hal.bmi088_gyr_cs",
        "delay": "hal.bmi088_delay",
    },
)
```

### Default Settings

- Gyroscope: ±2000°/s range, 2000 Hz ODR, 230 Hz filter bandwidth
- Accelerometer: Uses power-on default range (±3g), active mode

## Usage

Register the source type and wire its output to downstream tasks/sinks in `copperconfig.ron`.

## Compatibility

### Features

- `std`: Enable standard library support
- `defmt`: Enable `defmt` logging for embedded debugging
- `textlogs`: Enable text-based logging

### Hardware Notes

The BMI088 has **separate chip-select lines** for the accelerometer and gyroscope. Both sensors share the same SPI bus but require independent CS control:

| Signal | Description |
|--------|-------------|
| SPI MOSI | Shared data in |
| SPI MISO | Shared data out |
| SPI SCLK | Shared clock |
| ACC_CS | Accelerometer chip-select (active low) |
| GYR_CS | Gyroscope chip-select (active low) |

The accelerometer requires a **dummy byte** after the register address for reads, while the gyroscope uses standard SPI read protocol.

### Dependencies

This crate depends on:
- `cu29`: Copper runtime
- `cu-sensor-payloads`: Standard sensor payload types
- `embedded-hal` 0.2: Hardware abstraction traits

## Links

- Crate path: `components/sources/cu_bmi088`
- docs.rs: <https://docs.rs/cu-bmi088>
