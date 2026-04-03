# zed-sdk-sys

Raw Rust FFI bindings for the Stereolabs ZED C API (`sl_zed_c`).

This crate vendors the `zed-c-api` wrapper as a git submodule under `vendor/zed-c-api`.
The actual proprietary ZED SDK is still external and must be installed separately, typically under `/usr/local/zed`.

Current behavior:

- If `/usr/local/zed/lib/libsl_zed_c.so` already exists, the crate links against it.
- Otherwise, the build script tries to build `sl_zed_c` from the vendored `zed-c-api` submodule with CMake.
- If the native SDK is not installed, the Rust crate still compiles as an `rlib`, but consumers that actually link a final binary will need the native library available.
