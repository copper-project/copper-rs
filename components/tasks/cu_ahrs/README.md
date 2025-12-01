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
