# cu-ahrs

`cu-ahrs` is a Copper task that runs the [`dcmimu`](https://crates.io/crates/dcmimu) filter on standardized IMU payloads to output roll, pitch, and yaw in radians.

## Axis convention

- Body frame matches the aerospace/NED convention: **+X forward**, **+Y right**, **+Z down**.
- Positive rotations follow the right-hand rule about each axis:
  - Roll: right wing down is positive (rotation about +X).
  - Pitch: nose up is positive (rotation about +Y).
  - Yaw: clockwise when looking down along +Z (turning right) is positive.
- Accelerometer readings are expected to include gravity (a level IMU yields roughly `[0, 0, +9.81]` m/s²).

The task captures the first valid pose as a zero reference, so outputs are relative to the start orientation by default.

## Run a full MPU9250 → AHRS → print example

An end-to-end bare-metal demo for RP2350 (mirrors `mpu9250-whoami`) lives in `examples/rp2350_ahrs.rs`:

```bash
# Host clippy/tests remain on the default host target.
cargo clippy -p cu-ahrs
cargo test -p cu-ahrs

# RP2350 firmware build (no_std): opt-in config/target.
cd components/tasks/cu_ahrs
CARGO_CONFIG=.cargo/config.rp2350.toml \
    cargo run --no-default-features --features rp2350-demo --example rp2350_ahrs
```

The RP2350 build uses `thumbv8m.main-none-eabihf` and `memory.x` matching the original `mpu9250-whoami` app. It prints RPY (radians) over RTT/defmt. Wire your MPU9250 to SPI1 pins (SCK=GPIO10, MOSI=GPIO11, MISO=GPIO12, CS=GPIO13) on Pico 2W-class boards and flash with `probe-rs`.
