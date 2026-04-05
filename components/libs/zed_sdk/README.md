# zed-sdk

Safe Rust bindings for the Stereolabs ZED C API.

This crate wraps the low-level `sl_zed_c` interface in a typed Rust API for:

- opening a camera, SVO file, stream, or GMSL source
- grabbing image and depth frames into typed matrices
- reading calibration, camera, and sensor metadata
- keeping an escape hatch to the raw FFI through `zed_sdk::sys`

## Requirements

- Linux
- the proprietary ZED SDK installed under `/usr/local/zed` or `/opt/zed-sdk`
- a working CUDA/ZED runtime suitable for your camera and chosen depth mode

The sibling `zed-sdk-sys` crate links against an installed `libsl_zed_c` if it is already present.
If that wrapper is not installed yet, the build script can compile the vendored `zed-c-api` wrapper with CMake instead.

If you need the vendored wrapper path, initialize the submodule from the repository root:

```bash
git submodule update --init --recursive components/libs/zed_sdk_sys/vendor/zed-c-api
```

## Basic Use

```rust,ignore
use zed_sdk::{Camera, Mat, OpenOptions, ResolutionPreset, Rgba8, RuntimeParameters};

let options = OpenOptions::default()
    .camera_device_id(0)
    .resolution(ResolutionPreset::Hd720)
    .fps(30);

let mut camera = Camera::open(options)?;
let resolution = camera.resolution()?;
let runtime = RuntimeParameters::default();
let mut left = Mat::<Rgba8>::new_cpu(resolution)?;

camera.grab(&runtime)?;
camera.retrieve_left(&mut left)?;

let view = left.view()?;
println!("captured {}x{}", view.width(), view.height());
# Ok::<(), zed_sdk::Error>(())
```

`Mat<T>` keeps the element type explicit, so color, depth, and point-cloud buffers stay aligned with the SDK view or measure you request.

## Running The Demo

The crate includes `zed_viz_demo`, an `egui` viewer that overlays clipped depth on the left image feed.

From this directory:

```bash
cargo run --release --bin zed_viz_demo -- PERFORMANCE
```

Supported depth mode arguments are:

- `NONE`
- `PERFORMANCE`
- `QUALITY`
- `ULTRA`
- `NEURAL_LIGHT`
- `NEURAL`
- `NEURAL_PLUS`

Convenience targets are also available:

```bash
just performance
just neural-light
just neural
```

`just` defaults to `neural-light`.

## Raw FFI Access

If you need an SDK entry point that is not wrapped yet, use `zed_sdk::sys` directly and keep the safe layer for everything else.
