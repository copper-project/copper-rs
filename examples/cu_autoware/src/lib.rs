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
use std::time::{Duration, Instant};

#[cfg_attr(
    feature = "hybrid-background",
    copper_runtime(config = "copperconfig-hybrid.ron")
)]
#[cfg_attr(
    not(feature = "hybrid-background"),
    copper_runtime(config = "copperconfig.ron")
)]
struct App {}

const SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);
/// Bounded so the example exits on its own. At the calibrated crunch limits one pass over
/// the graph costs far more than the 5ms grid, so wall time per iteration is what the run
/// measures rather than something to predict.
pub const ITERATIONS: usize = 400;

/// Stop condition for a benchmark recording.
pub enum RunLimit {
    Iterations(usize),
    Duration(Duration),
}

/// `log_base` defaults to `<crate>/logs/autoware.copper`; give it a distinct base per
/// variant so parallel or successive runs do not clobber each other's log.
pub fn run(limit: RunLimit, log_base: Option<PathBuf>, config_path: Option<PathBuf>) {
    // Anchored to the crate, not the cwd: step 4's logreader needs a stable location.
    let logger_path = log_base.unwrap_or_else(|| {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("logs")
            .join("autoware.copper")
    });
    if let Some(parent) = logger_path.parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }
    let mut builder = App::builder()
        .with_log_path(&logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.");
    if let Some(path) = config_path {
        let path = path.to_str().expect("configuration path must be UTF-8");
        builder = builder.with_config(read_configuration(path).expect("Failed to read config."));
    }
    let mut application = builder.build().expect("Failed to create application.");
    let rate_target_hz = application
        .copper_runtime_mut()
        .runtime_config
        .rate_target_hz
        .expect("copperconfig.ron must set runtime.rate_target_hz.");
    let mut rate_limiter =
        LoopRateLimiter::from_rate_target_hz(rate_target_hz, &application.clock())
            .expect("Failed to create the rate limiter.");
    let mut running = application.start().expect("Failed to start application.");
    let started = Instant::now();
    let mut iterations = 0usize;
    while match limit {
        RunLimit::Iterations(target) => iterations < target,
        RunLimit::Duration(duration) => started.elapsed() < duration,
    } {
        running
            .run_one_iteration()
            .expect("Failed to run application.");
        rate_limiter.limit(&running.clock());
        iterations += 1;
    }
    let stopped = running.stop().expect("Failed to stop application.");
    // The generated `run()` writes this; a manual loop has to. `kpi` refuses to report on
    // a log that does not end with it, which is how a truncated log is caught.
    let mut application = stopped.into_inner();
    application
        .log_shutdown_completed()
        .expect("Failed to record shutdown.");
}
