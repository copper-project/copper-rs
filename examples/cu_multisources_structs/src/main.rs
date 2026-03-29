pub mod tasks;

use cu29::prelude::*;
use std::path::Path;
use std::thread::sleep;
use std::time::Duration;

use crate::default::SimStep;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct SimTestApplication {}

fn main() {
    let logger_path = "logs/sim-test.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        std::fs::create_dir_all(parent).expect("Failed to create logs directory");
    }
    debug!("Logger created at {}.", &logger_path);
    debug!("Creating application... ");
    let mut application = SimTestApplication::builder()
        .with_sim_callback(&mut default_callback)
        .with_log_path(logger_path, PREALLOCATED_STORAGE_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");
    debug!("Running... starting clock: {}.", application.clock().now());

    application
        .run(&mut default_callback)
        .expect("Failed to run application.");
    debug!("End of program: {}.", application.clock().now());
    sleep(Duration::from_secs(1));
}

fn default_callback(_sim_step: SimStep) -> SimOverride {
    SimOverride::ExecutedBySim
}
