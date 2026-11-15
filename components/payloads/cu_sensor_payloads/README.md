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

## Rerun images

Enable `rerun` to inspect `CuImage` in Copper's 2D debug view or log it directly
with Rerun. RGB/BGR, RGBA/BGRA, grayscale, NV12, and YUYV/YUY2 are supported.
Row padding is removed during visualization; the original buffer is unchanged.
Unsupported formats, malformed layouts, and short buffers clear the previous image.

Generate a sample recording with `cargo run -p cu-sensor-payloads --features rerun
--example rerun_image -- image.rrd`, then open `image.rrd` in Rerun.
