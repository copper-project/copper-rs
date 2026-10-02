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
- `self-describing-logs`: native payload descriptions for host catalog generation,
  including captured image and depth buffers; enables `std` and reflection.
