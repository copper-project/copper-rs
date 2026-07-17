# cu29-build

Shared build-script setup for Copper crates and applications.

Call `cu29_build::setup()` once from `build.rs`. It configures Copper's logging
macros and forwards the crate's active Cargo features to Copper code generation.
