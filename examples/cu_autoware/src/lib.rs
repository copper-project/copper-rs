//! The Autoware reference system as a Copper application.
//!
//! The graph lives here rather than in `main.rs` so the `calibrate` binary links the same
//! `tasks::crunch` the application runs.

pub mod payload;
pub mod tasks;

use cu29::curuntime::LoopRateLimiter;
use cu29::prelude::*;
use std::fs;
use std::path::PathBuf;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);
/// Bounded so the example exits on its own. At the calibrated crunch limits one pass over
/// the graph costs far more than the 5ms grid, so wall time per iteration is what the run
/// measures rather than something to predict.
pub const ITERATIONS: usize = 200;

pub fn run(iterations: usize) {
    // Anchored to the crate, not the cwd: step 4's logreader needs a stable location.
    let logger_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("logs")
        .join("autoware.copper");
    if let Some(parent) = logger_path.parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }
    let mut application = App::builder()
        .with_log_path(&logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");
    let rate_target_hz = application
        .copper_runtime_mut()
        .runtime_config
        .rate_target_hz
        .expect("copperconfig.ron must set runtime.rate_target_hz.");
    let mut rate_limiter =
        LoopRateLimiter::from_rate_target_hz(rate_target_hz, &application.clock())
            .expect("Failed to create the rate limiter.");
    application
        .start_all_tasks()
        .expect("Failed to start application.");
    for _ in 0..iterations {
        application
            .run_one_iteration()
            .expect("Failed to run application.");
        rate_limiter.limit(&application.clock());
    }
    application
        .stop_all_tasks()
        .expect("Failed to stop application.");
}
