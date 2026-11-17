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

## Rerun depth images

Enable `rerun` to inspect `CuDepthMap` in Copper's 2D debug view or log it directly
with Rerun. All `CuDepthEncoding` implementations are decoded into metric depth
only when visualization is requested. Row padding is removed; invalid, nonfinite,
and nonpositive samples become zero (missing depth). Malformed layouts and short
buffers clear the previous depth image. Recorded payloads retain their native encoding.

Generate a sample recording with `cargo run -p cu-sensor-payloads --features rerun
--example rerun_depth -- depth.rrd`, then open `depth.rrd` in Rerun.

## Rerun images

Enable `rerun` to inspect `CuImage` in Copper's 2D debug view or log it directly
with Rerun. RGB/BGR, RGBA/BGRA, grayscale, NV12/NV21, I420/YV12, and YUYV/YUY2/UYVY are supported.
Subsampled 4:2:0 images require even width and height; packed 4:2:2 images
require even width.
Row padding is removed during visualization; the original buffer is unchanged.
Unsupported formats, malformed layouts, and short buffers clear the previous image.

Generate a sample recording with `cargo run -p cu-sensor-payloads --features rerun
--example rerun_image -- image.rrd`, then open `image.rrd` in Rerun.
