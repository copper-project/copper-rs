# cu-bdshot-demo

A Copper mission that drives four bidirectional DSHOT (BDShot) ESCs through
`cu_bdshot::RpBdshotBridge` on the RP2350 reference board. The demo cycles the
throttle of each ESC between idle and ~30% duty while printing the telemetry
samples returned by the ESCs (temperature, voltage, eRPM…).

## Hardware wiring

Use the RP2350 reference board shipped with Copper:

| ESC channel | RP2350 pin |
|-------------|-----------|
| ESC 0       | GPIO6     |
| ESC 1       | GPIO7     |
| ESC 2       | GPIO8     |
| ESC 3       | GPIO9     |

Each ESC’s signal wire connects to its pin, with the ground shared with the
board. The example expects the ESCs to speak bidirectional DSHOT and provide
telemetry immediately after arming.

## Building & flashing

`cargo run -p cu-bdshot-demo --release`  
The `.cargo/config.toml` in this example already targets `thumbv8m.main-none-eabihf`
and configures `probe-rs` with the onboard CMSIS-DAP probe, so `cargo run`
builds and flashes the firmware automatically.  
After flashing, use `probe-rs defmt` (or your RTT client of choice) to see the
throttle ramps and telemetry logs.

## What it demonstrates

- `cu_bdshot::RpBdshotBridge` encapsulates the RP2350 PIO program, DMA and timer
  handling needed to speak BDShot, exposing four static channels to the Copper
  runtime.
- Four `ThrottleSource` tasks feed commands into the bridge; the commands ramp up
  and down to exercise the ESCs deterministically.
- Four `TelemetrySink` tasks consume the bridge telemetry channels, logging the
  decoded DSHOT telemetry packets.

This mission is a starting point for integrating BDShot ESCs in a Copper robot.
Replace the throttle tasks with your flight controller logic or scheduler and
extend the sinks to react to the telemetry. The bridge itself lives in
`components/bridges/cu_bdshot` and can be reused in other missions or boards.
