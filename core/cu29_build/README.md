# cu29-build

Shared build-script setup for Copper crates and applications.

Call `cu29_build::setup()` once from `build.rs`. It configures Copper's logging
macros and forwards the crate's active Cargo features to Copper code generation.

Applications enable `cu29/self-describing-logs` to record a compressed catalog
through the normal generated builder. The usual `cu29_build::setup()` call is
sufficient; see `examples/cu_self_describing_logs`.
