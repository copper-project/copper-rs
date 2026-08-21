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

/// Must agree with `slab_size_mib` in `copperconfig.ron`: the writer only sees
/// this value, while config validation checks section sizes against the RON one.
const SLAB_SIZE: Option<usize> = Some(32 * 1024 * 1024);

fn main() {
    run().expect("AnyNet foreground demo failed");
}

fn run() -> CuResult<()> {
    let log_path = "logs/anynet_fg.copper";
    if let Some(parent) = Path::new(log_path).parent() {
        fs::create_dir_all(parent)
            .map_err(|error| CuError::new_with_cause("Failed to create logs directory", error))?;
    }
    let mut app = App::builder().with_log_path(log_path, SLAB_SIZE)?.build()?;
    app.run()
}
