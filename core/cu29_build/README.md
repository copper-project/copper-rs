# cu29-build

Shared build-script setup for Copper crates and applications.

Call `cu29_build::setup()` once from `build.rs`. It configures Copper's logging
macros and forwards the crate's active Cargo features to Copper code generation.

Enable `self-describing-logs` to package a `ValueDecodeCatalog` on the host with
`catalog::write_value_decode_catalog("catalog.rs", &catalog)`. The helper emits a
Rust static byte array into Cargo's `OUT_DIR`. Include the generated file in the
application and pass `VALUE_DECODE_CATALOG` to the generated builder's
`with_value_decode_catalog` method. See `examples/cu_self_describing_logs` for a
shared payload crate, generated slot registrations and startup recording.
