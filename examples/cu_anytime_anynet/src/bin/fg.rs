//! Foreground AnyNet anytime demo.

use cu29::prelude::*;
use std::fs;
use std::path::Path;

#[cfg_attr(
    not(feature = "smoketest"),
    copper_runtime(config = "copperconfig.ron")
)]
#[cfg_attr(
    feature = "smoketest",
    copper_runtime(config = "copperconfig_smoke.ron")
)]
struct App {}

#[cfg(not(feature = "smoketest"))]
use kitti::App as KittiApp;
#[cfg(not(feature = "smoketest"))]
use synthetic::App as SyntheticApp;

/// Must agree with `slab_size_mib` in `copperconfig.ron`: the writer only sees
/// this value, while config validation checks section sizes against the RON one.
const SLAB_SIZE: Option<usize> = Some(32 * 1024 * 1024);

fn main() {
    run().expect("AnyNet foreground demo failed");
}

fn prepare_log(log_path: &str) -> CuResult<()> {
    if let Some(parent) = Path::new(log_path).parent() {
        fs::create_dir_all(parent)
            .map_err(|error| CuError::new_with_cause("Failed to create logs directory", error))?;
    }
    Ok(())
}

#[cfg(feature = "smoketest")]
fn run() -> CuResult<()> {
    let log_path = "examples/cu_anytime_anynet/logs/anynet_fg_smoketest.copper";
    prepare_log(log_path)?;
    let mut app = App::builder().with_log_path(log_path, SLAB_SIZE)?.build()?;
    app.run()
}

#[cfg(not(feature = "smoketest"))]
fn run() -> CuResult<()> {
    let mission = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "synthetic".to_owned());
    let log_path = format!("examples/cu_anytime_anynet/logs/anynet_fg_{mission}.copper");
    prepare_log(&log_path)?;

    match mission.as_str() {
        "synthetic" => {
            let mut app = SyntheticApp::builder()
                .with_log_path(&log_path, SLAB_SIZE)?
                .build()?;
            app.run()
        }
        "kitti" => {
            let mut app = KittiApp::builder()
                .with_log_path(&log_path, SLAB_SIZE)?
                .build()?;
            app.run()
        }
        _ => Err(CuError::from(format!(
            "Unknown mission '{mission}'; expected 'synthetic' or 'kitti'"
        ))),
    }
}
