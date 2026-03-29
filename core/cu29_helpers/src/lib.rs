#![cfg_attr(not(feature = "std"), no_std)]
#![doc = include_str!("../README.md")]

const _: () = {
    if option_env!("CARGO_PRIMARY_PACKAGE").is_none() {
        panic!(
            r#"`cu29-helpers` was retired. Port your app to the generated App builder API.

Old pattern:
    let ctx = cu29_helpers::basic_copper_setup(log_path, slab_size, text_log, Some(clock))?;
    let mut app = MyApp::new(clock.clone(), ctx.unified_logger.clone(), config_override)?;

New pattern:
    let mut app = MyApp::builder()
        .with_clock(clock)                 // optional; defaults to RobotClock::default()
        .with_config(config_override)      // optional
        .with_log_path(log_path, slab_size)?
        .build()?;

If you already constructed the unified logger yourself:
    let mut app = MyApp::builder()
        .with_clock(clock)
        .with_logger::<MmapSectionStorage, UnifiedLoggerWrite>(unified_logger)
        .build()?;

Notes:
    - Drop `basic_copper_setup(...)`.
    - Drop `App::new(...)` / `App::new_with_resources(...)` in favor of `App::builder()`.
    - The app now owns logging setup.
    - See `templates/cu_project/src/main.rs` or `examples/cu_caterpillar/src/main.rs`.
"#
        );
    }
};

/// Compatibility stub for workspace builds.
///
/// When `cu29-helpers` is used as a dependency, this crate emits a compile
/// error with migration instructions to the app-builder API.
pub mod compatibility_stub {}
