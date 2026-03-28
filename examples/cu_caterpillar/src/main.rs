pub mod tasks;

use cu29::prelude::*;
use std::fs;
use std::path::Path;

#[copper_runtime(config = "copperconfig.ron")]
struct CaterpillarApplication {}

const SLAB_SIZE: Option<usize> = Some(1024 * 1024 * 1024);

fn main() {
    let logger_path = "logs/caterpillar.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }

    let clock = RobotClock::default();
    let mut application = CaterpillarApplication::builder()
        .with_clock(clock)
        .with_log_path(logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");

    if let Err(error) = application.run() {
        debug!("Application Ended: {}", error)
    }
}

#[cfg(all(test, feature = "determinism_ci"))]
mod determinism_test;
