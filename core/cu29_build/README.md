# cu29-build

Shared build-script setup for Copper crates and applications.

Call `cu29_build::setup()` once from `build.rs`. It configures Copper's logging
macros and forwards the crate's active Cargo features to Copper code generation.

Applications enable `cu29/self-describing-logs` to record a compressed catalog
through the normal generated builder. The usual `cu29_build::setup()` call is
sufficient; see `examples/cu_self_describing_logs`.

For explicit host packaging, this crate's `self-describing-logs` feature provides
`catalog::write_value_decode_catalog("catalog.rs", &catalog)`. It emits a Heatshrink
blob into Cargo's `OUT_DIR`, usable as an override with the generated builder's
`with_value_decode_catalog` method. Host schema construction uses `cu29/decode-catalog`.
