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
