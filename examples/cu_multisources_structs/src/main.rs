pub mod tasks;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::path::{Path, PathBuf};
use std::thread::sleep;
use std::time::Duration;

use crate::default::SimStep;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct SimTestApplication {}

fn main() {
    let logger_path = "logs/sim-test.copper";
    if let Some(parent) = Path::new(logger_path).parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }
    let copper_ctx = basic_copper_setup(
        &PathBuf::from(&logger_path),
        PREALLOCATED_STORAGE_SIZE,
        true,
        None,
    )
    .expect("Failed to setup logger.");
    debug!("Logger created at {}.", logger_path);
    debug!("Creating application... ");
    let mut application = SimTestApplicationBuilder::new()
        .with_sim_callback(&mut default_callback)
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create application.");
    let clock = copper_ctx.clock.clone();
    debug!("Running... starting clock: {}.", clock.now());

    application
        .run(&mut default_callback)
        .expect("Failed to run application.");
    debug!("End of program: {}.", clock.now());
    sleep(Duration::from_secs(1));
}

fn default_callback(_sim_step: SimStep) -> SimOverride {
    SimOverride::ExecutedBySim
}
