# cu-sensor-payloads

Standardized sensor payload definitions for Copper.

The crate contains common payload types used by Copper sources, tasks, and
sinks, including image and point-cloud related data structures.

## Features

- `std` (default)
- `textlogs`
- `image`
- `kornia`
- `rerun`
- `reflect`: reflection support for payload fields.

Native encoding descriptions are always available, including captured image and
depth buffers. Enable `cu29/self-describing-logs` in the catalog application to
package these descriptions for offline decoding.

With `rerun`, IMU and magnetometer payloads expose sensor-frame direction arrows
in Copper's debugger and implement Rerun's `AsComponents`. Arrows have unit
length; labels carry the measured magnitude in m/s², rad/s, or µT. Separate
origins distinguish quantities and do not represent sensor mounting positions.
Zero vectors remain zero, nonfinite vectors are omitted, and temperature remains
available through field inspection.

Record an example with `cargo run -p cu-sensor-payloads --features rerun --example rerun_imu -- imu.rrd`,
then open `imu.rrd` in Rerun.
