//! Background AnyNet anytime demo.

use cu29::prelude::*;
use std::fs;
use std::path::Path;

#[copper_runtime(config = "copperconfig_bg.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = Some(32 * 1024 * 1024);

fn main() {
    run().expect("AnyNet background demo failed");
}

fn run() -> CuResult<()> {
    let log_path = "examples/cu_anytime_anynet/logs/anynet_bg.copper";
    if let Some(parent) = Path::new(log_path).parent() {
        fs::create_dir_all(parent)
            .map_err(|error| CuError::new_with_cause("Failed to create logs directory", error))?;
    }
    let app = App::builder().with_log_path(log_path, SLAB_SIZE)?.build()?;
    app.run_until_shutdown()?;
    Ok(())
}
