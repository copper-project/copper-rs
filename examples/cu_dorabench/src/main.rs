pub mod tasks;

use cu29::prelude::*;
use std::fs;
use std::path::PathBuf;

#[copper_runtime(config = "copperconfig.ron")]
struct DoraBench {}

// This will create a torrent of crap so preallocate a lot.
const SLAB_SIZE: Option<usize> = Some(4096 * 1024 * 1024);

fn main() {
    let logger_path = PathBuf::from("logs/dorabench.copper");
    if let Some(parent) = logger_path.parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }

    let mut application = DoraBench::builder()
        .with_log_path(&logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");

    if let Err(error) = application.run() {
        debug!("Application Ended: {}", error)
    }
}
