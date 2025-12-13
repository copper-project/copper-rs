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

## Bare-metal RP2350 demos

Two RP2350 no-std options mirroring `mpu9250-whoami`:
- `examples/rp2350_ahrs.rs`: stand-alone loop (no Copper runtime) printing RPY.
- `examples/rp2350_copper.rs` + `examples/rp_copperconfig.ron`: full Copper app wiring `registry::RpMpu9250Source -> cu_ahrs::CuAhrs -> tasks::RpySink`, with SPI/CS/delay registered via `cu_embedded_registry` in board init.

```bash
# Host clippy/tests remain on the default host target.
cargo clippy -p cu-ahrs
cargo test -p cu-ahrs

# RP2350 firmware builds (no_std): opt-in config/target.
CARGO_CONFIG=.cargo/config.rp2350.toml cargo run --no-default-features --features rp2350-demo --example rp2350_copper
```

The RP2350 build uses `thumbv8m.main-none-eabihf` and `memory.x` matching the original `mpu9250-whoami` app. It prints RPY (radians) over RTT/defmt. Wire your MPU9250 to SPI1 pins (SCK=GPIO10, MOSI=GPIO11, MISO=GPIO12, CS=GPIO13) on Pico 2W-class boards and flash with `probe-rs`.
