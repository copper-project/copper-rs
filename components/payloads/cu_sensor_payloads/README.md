# cu-sensor-payloads

Standardized sensor payload definitions for Copper.

The crate contains common payload types used by Copper sources, tasks, and
sinks, including image, depth-map, camera-calibration, and point-cloud data
structures.

## Camera calibration propagation

`CuCameraModel` provides a fixed-size, allocation-free camera model containing
pinhole intrinsics and a standard distortion model. It is intentionally separate
from `CuImage` and `CuDepthMap`: a source can expose the model on a dedicated
output as `CuCameraModelUpdate` and send `Set(model)` only when calibration first
becomes available or changes. Later cycles carry `NoChange`, while each consumer
keeps a `CuCameraModelState` cache.

This keeps the per-frame payload small without hiding calibration changes from
Copper's unified log and deterministic replay.

## Features

- `std` (default)
- `textlogs`
- `image`
- `kornia`
- `rerun`
