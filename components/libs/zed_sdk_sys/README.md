# zed-sdk-sys

Raw Rust FFI bindings for the Stereolabs ZED C API (`sl_zed_c`).

This crate vendors the open-source `zed-c-api` wrapper under
`vendor/zed-c-api`. The proprietary ZED SDK itself is not vendored; it still
needs to be installed separately under `/usr/local/zed` or `/opt/zed-sdk`.

## Vendoring

The intended entry points in this crate are:

- `just build`
- `just demo`

`just build` compiles the Rust FFI crate itself. `just demo` runs the small
`zed_lowlevel_demo` smoke test binary in this crate.

Install the native ZED SDK and run the crate-local target you need.
The C wrapper sources are included in the checkout.

At build time the flow is:

1. If `libsl_zed_c.so` is already present under `/usr/local/zed/lib` or
   `/opt/zed-sdk/lib`, the crate links against that installed wrapper directly.
2. Otherwise, the build script configures and builds `sl_zed_c` from the
   vendored `vendor/zed-c-api` sources with CMake.
3. That vendored build still depends on the native ZED SDK headers and
   libraries being installed locally. "Vendored" here means the C wrapper
   source is in-repo, not that the full Stereolabs SDK is bundled.

If the native SDK is missing, the Rust crate can still compile as an `rlib`,
but any final binary that actually links the ZED API will still need the native
library available. ## Demo

`just demo` runs the direct `zed-sdk-sys` smoke test binary. It opens the
camera, grabs a frame, retrieves the left image, and optionally reads a center
depth sample. This is the smallest end-to-end check that the raw FFI layer is
wired correctly.

`just demo QUALITY` or another ZED depth mode can be used to override the
default `NEURAL_LIGHT` mode.

When invoking Cargo directly, enable the hardware-only demo target with
`--features demo`.
