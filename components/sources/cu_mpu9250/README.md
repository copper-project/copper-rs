# cu-mpu9250

Copper source component for the MPU9250 IMU. This crate is `no_std` by default and ships an `embedded-hal`-compatible SPI driver so it can run on both baremetal and Linux.

## Features

- `embedded-hal`: Generic SPI+CS+delay backend over `embedded-hal` 1.0 traits.
- `rp235x-hal`: SPI1 backend for RP235x (Pico 2 class) boards wired to GPIO10-13.
- `linux-embedded-hal`: spidev + gpio-cdev backend for Linux boards.
- `mock`: Reserved for mock backends.

## RP235x usage

```bash
# Build the source + AHRS stack for RP2350
CARGO_CONFIG=components/tasks/cu_ahrs/.cargo/config.rp2350.toml \
    cargo run --no-default-features --features "rp2350-demo" \
    -p cu-ahrs --example rp2350_copper
```

In Copper apps, bind `spi`, `cs`, and `delay` from a board resource bundle into
`cu_mpu9250::Mpu9250Source<...>`.

The RP2350 AHRS demo initializes those HAL handles in firmware setup, exposes
them through `resources::Rp2350ImuBundle`, and configures the task with:
- `gyro_cal_ms` (u32, default 0 = disabled)
- `gyro_sample_delay_ms` (u32, default 10)

## Linux embedded-hal usage

Enable the `linux-embedded-hal` feature to talk to MPU9250 via spidev and a gpio-cdev chip-select.

Config fields:
- `spi_path` (string, default `/dev/spidev0.0`): spidev bus path.
- `spi_hz` (u32, default 1_000_000): SPI bus speed.
- `gpio_chip` (string, default `/dev/gpiochip0`): GPIO chip for the CS line.
- `cs_line` (u32, required): Line offset within the GPIO chip to drive CS.
- `gyro_cal_ms` / `gyro_sample_delay_ms`: same calibration knobs as the RP backend.
