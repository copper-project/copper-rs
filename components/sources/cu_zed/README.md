# cu-zed

Copper source task for Stereolabs ZED cameras, plus a pure Copper task that turns the published ZED depth map into a standard Copper `PointCloudSoa`.

This crate gives you two main building blocks:

- `cu_zed::Zed`: the camera source task
- `cu_zed::ZedDepthToPointCloud<const MAX_POINTS: usize>`: a pure task that projects a ZED depth map into a standard Copper point cloud

The demo in this directory wires both together and streams the result to Rerun:

- `components/sources/cu_zed/examples/zed_rerun_demo.ron`
- `components/sources/cu_zed/examples/zed_rerun_demo.rs`

## What This Crate Publishes

`cu_zed::Zed` is a `CuSrcTask` with this output tuple:

1. `cu_zed::ZedStereoImages`
2. `cu_zed::ZedDepthMap<Vec<f32>>`
3. `cu_zed::ZedConfidenceMap<Vec<f32>>`
4. `cu29::prelude::CuLatchedStateUpdate<cu_zed::ZedCalibrationBundle>`
5. `cu29::prelude::CuLatchedStateUpdate<cu_zed::ZedRigTransforms>`
6. `cu_sensor_payloads::ImuPayload`
7. `cu_sensor_payloads::MagnetometerPayload`
8. `cu_sensor_payloads::BarometerPayload`
9. `cu_zed::ZedFrameMeta`

Semantics:

- Stereo images are left/right RGBA frames in pooled host memory.
- Depth is a dense `f32` raster in pooled host memory.
- Confidence is optional and only emitted when enabled in config.
- Calibration and rig transforms are latched state updates:
  the source publishes `Set(...)` when the task starts, then `NoChange` after that.
- IMU, magnetometer, and barometer are emitted when enabled and available from the camera.
- Frame metadata contains sequence/timestamp/fps/temperature information for the current frame.

## Platform Support

Real camera operation is Linux-only.

On non-Linux targets, `cu_zed::Zed` still exists so Copper configs can compile, but it is a stub task that emits no payloads and sends `NoChange` for latched calibration/transform outputs.

## Prerequisites

You need both:

- The proprietary Stereolabs ZED SDK installed under `/usr/local/zed` or `/opt/zed-sdk`
- The vendored `zed-c-api` submodule initialized under `components/libs/zed_sdk_sys/vendor/zed-c-api`

Useful repo-local commands:

```bash
just --justfile components/sources/cu_zed/justfile doctor
just --justfile components/sources/cu_zed/justfile check
just --justfile components/sources/cu_zed/justfile demo
```

What they do:

- `doctor` checks for the native SDK install and the vendored wrapper submodule
- `check` initializes the submodule if needed and builds the example
- `demo` runs the bundled Rerun example

Notes:

- The native library is normally detected under `/usr/local/zed/lib` or `/opt/zed-sdk/lib`.
- If the native wrapper is not preinstalled, `zed-sdk-sys` can build `sl_zed_c` from the vendored `zed-c-api` sources.
- A final binary that links the camera driver still needs the native ZED SDK available on the machine.

## The Base Source Task

### Task Type

Use this type in `copperconfig.ron`:

```ron
(
    id: "zed",
    type: "cu_zed::Zed",
    config: {
        "frame_id_prefix": "zed",
        "depth_mode": "NEURAL_PLUS",
        "depth_maximum_distance_m": 5.0,
        "enable_fill_mode": true,
        "emit_confidence": true,
        "emit_imu": true,
        "emit_mag": true,
        "emit_baro": true,
    },
)
```

### Input Source Selection

The source chooses one camera/input source from config in this priority order:

1. `gmsl_serial_number` + `gmsl_port`
2. `serial_number`
3. `svo_file`
4. `stream_ip` + optional `stream_port`
5. `device_id` (default USB camera id `0`)

### Supported Config Keys

Source selection:

- `device_id: i32`
- `serial_number: u32`
- `gmsl_serial_number: u32`
- `gmsl_port: i32`
- `svo_file: String`
- `stream_ip: String`
- `stream_port: i32`

Camera/open options:

- `resolution: String`
- `fps: i32`
- `depth_mode: String`
- `coordinate_system: String`
- `coordinate_unit: String`
- `depth_minimum_distance_m: f64`
- `depth_maximum_distance_m: f64`
- `open_timeout_ms: u32`
- `sensors_required: bool`
- `sdk_verbose: i32`
- `settings_path: String`
- `opencv_calibration_path: String`

Runtime parameters:

- `reference_frame: String`
- `enable_fill_mode: bool`
- `confidence_threshold: i32`
- `texture_confidence_threshold: i32`

Copper/task behavior:

- `emit_confidence: bool` default `false`
- `emit_imu: bool` default `true`
- `emit_mag: bool` default `true`
- `emit_baro: bool` default `true`
- `pool_slots: u32` default `4`
- `frame_id_prefix: String` default `"zed"`

Accepted `resolution` values:

- `HD4K`
- `QHDPLUS`
- `HD2K`
- `HD1536`
- `HD1080`
- `HD1200`
- `HD720`
- `SVGA`
- `VGA`
- `AUTO`

Accepted `coordinate_system` values:

- `IMAGE`
- `LEFT_HANDED_Y_UP`
- `RIGHT_HANDED_Y_UP`
- `RIGHT_HANDED_Z_UP`
- `LEFT_HANDED_Z_UP`
- `RIGHT_HANDED_Z_UP_X_FWD`
- `RIGHT_HANDED_Z_UP_X_FORWARD`

Accepted `coordinate_unit` values:

- `MILLIMETER`
- `CENTIMETER`
- `METER`
- `INCH`
- `FOOT`

Accepted `reference_frame` values:

- `CAMERA`
- `WORLD`

## Calibration And Transform Messages

`cu_zed::ZedCalibrationBundle` contains:

- left/right camera intrinsics
- image dimensions and fps
- stereo extrinsics
- optional camera-to-IMU transform data
- the configured ZED coordinate system
- the configured ZED coordinate unit

`cu_zed::ZedRigTransforms` contains:

- `left_to_right`
- `camera_to_imu`
- `has_camera_to_imu`

These are designed to be consumed as Copper latched state. A downstream task should cache the last `Set(...)` value and ignore `NoChange`.

## The Depth-To-PointCloud Task

### What It Does

`ZedDepthToPointCloud<const MAX_POINTS: usize>` is a pure `CuTask` that consumes:

- `cu_zed::ZedDepthMap<Vec<f32>>`
- `cu29::prelude::CuLatchedStateUpdate<cu_zed::ZedCalibrationBundle>`

and produces:

- `cu_sensor_payloads::PointCloudSoa<MAX_POINTS>`

The task:

- caches the latest calibration bundle
- rescales intrinsics if the depth raster size differs from the calibration image size
- converts the ZED depth unit into meters
- skips invalid depth samples (`NaN`, non-finite, or `<= 0`)
- writes a standard Copper point cloud with:
  - `x`, `y`, `z` in meters
  - `i = 0%`
  - `return_order = 0`

### Important Constraint

`MAX_POINTS` must be large enough for the number of valid depth samples you expect.

The task returns an error if the depth map would overflow the output capacity.

### Supported Projection Conventions

The projection task currently supports these ZED coordinate systems:

- `IMAGE`
- `LEFT_HANDED_Y_UP`

That covers the common dense depth projection cases currently used by this crate.

### Ready-Made HD720 Alias

For the default ZED source resolution, use:

- `cu_zed::ZedDepthToPointCloudHd720`
- `cu_zed::ZedPointCloudHd720`

Those aliases are sized for:

```text
1280 * 720 = 921600 points
```

If you run the camera at another resolution, prefer the generic task:

```text
cu_zed::ZedDepthToPointCloud<{ WIDTH * HEIGHT }>
```

and connect it to:

```text
cu_sensor_payloads::PointCloudSoa<{ WIDTH * HEIGHT }>
```

## Using It In Copper

### Minimal Wiring

This is the standard graph shape:

```ron
(
    tasks: [
        (
            id: "zed",
            type: "cu_zed::Zed",
            config: {
                "frame_id_prefix": "zed",
                "depth_mode": "NEURAL_PLUS",
                "depth_maximum_distance_m": 5.0,
                "enable_fill_mode": true,
            },
        ),
        (
            id: "pointcloud",
            type: "cu_zed::ZedDepthToPointCloudHd720",
        ),
        (
            id: "sink",
            type: "MyPointCloudSink",
        ),
    ],
    cnx: [
        (
            src: "zed",
            dst: "pointcloud",
            msg: "cu_zed::ZedDepthMap<Vec<f32>>",
        ),
        (
            src: "zed",
            dst: "pointcloud",
            msg: "cu29::prelude::CuLatchedStateUpdate<cu_zed::ZedCalibrationBundle>",
        ),
        (
            src: "pointcloud",
            dst: "sink",
            msg: "cu_zed::ZedPointCloudHd720",
        ),
    ],
)
```

### Rust Sink Example

```rust
use cu29::prelude::*;
use cu_zed::ZedPointCloudHd720;

#[derive(Reflect)]
#[reflect(from_reflect = false)]
struct MyPointCloudSink;

impl Freezable for MyPointCloudSink {}

impl CuSinkTask for MyPointCloudSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, ZedPointCloudHd720);

    fn new(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let Some(pointcloud) = input.payload() else {
            return Ok(());
        };

        info!("pointcloud points={}", pointcloud.len);
        Ok(())
    }
}
```

## Rerun Usage Note

If you visualize both the camera and the projected point cloud in Rerun:

- log camera images/depth under the camera entity, for example `zed/left_camera/...`
- log the 3D point cloud outside the pinhole subtree, for example `zed/pointcloud`

Do not log a `Points3D` entity under a `Pinhole` subtree such as `zed/left_camera/pointcloud`, because Rerun will treat that subtree as image-projected content rather than free 3D geometry.

The bundled demo follows this rule and logs the projected cloud at `zed/pointcloud`.

## Development Commands

From the repo root:

```bash
just --justfile components/sources/cu_zed/justfile doctor
just --justfile components/sources/cu_zed/justfile dag
just --justfile components/sources/cu_zed/justfile check
just --justfile components/sources/cu_zed/justfile demo
```

`dag` opens the rendered Copper graph for the demo config.

## Implementation Notes

- The source uses pooled host buffers for image/depth/confidence payloads.
- Calibration and rig transforms are emitted as latched state updates on startup.
- The point cloud task is intentionally pure: it only depends on the incoming depth raster and the cached calibration message.
- On non-Linux targets, the source task is a compile-time-compatible stub with no live camera output.
