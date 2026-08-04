# cu-sensor-payloads

Standardized sensor payload definitions for Copper.

The crate contains common payload types used by Copper sources, tasks, and
sinks, including image, depth-map, camera-calibration, and point-cloud data
structures.

## Dynamic camera calibration

`CuCameraModel<D>` provides a fixed-size, allocation-free camera model containing
pinhole intrinsics and a compile-time distortion type. A robot chooses `D` when
it is built—for example, `CuPlumbBobDistortion` or
`CuEquidistantDistortion`—so a running camera cannot silently change its
mathematical model. The intrinsics and coefficients may still be updated by a
dynamic-calibration task.

The model is intentionally separate from `CuImage` and `CuDepthMap`: a source
can expose it on a dedicated output as `CuCameraModelUpdate<D>` and send
`Set(model)` only when calibration first becomes available or changes. Later
cycles carry `NoChange`, while each consumer keeps a `CuCameraModelState<D>`
cache.

This keeps the per-frame payload small without hiding calibration changes from
Copper's unified log and deterministic replay.

## Features

- `std` (default)
- `textlogs`
- `image`
- `kornia`
- `rerun`
