# cu-mpu9250

Copper source component for the MPU9250 IMU. It is `no_std` by default and can run on embedded targets using any `embedded-hal` 0.2 backend supported by the `mpu9250` crate.

## Features

- `std` (default): Enables the Copper logging runtime helpers.
- `linux-embedded`: Linux I2C implementation using `linux-embedded-hal` and the helper application in `src/bin/logger.rs`.
- `mock`: Reserved for mock backends.

## Quick try on Linux

```shell
cargo run -p cu-mpu9250 --features=linux-embedded --bin logger
```

The bundled `copperconfig.ron` wires `LinuxMpu9250Source` to a simple logger sink that prints IMU readings with `debug!`.
