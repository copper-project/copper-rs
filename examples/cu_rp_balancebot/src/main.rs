pub mod tasks;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};

#[copper_runtime(config = "copperconfig.ron")]
struct BalanceBot {}

// preallocate a lot.
#[allow(clippy::identity_op)]
const SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);

fn main() {
    static STOP_FLAG: AtomicBool = AtomicBool::new(false);
    let logger_path = "logs/balance.copper";
    if let Some(parent) = Path::new(logger_path).parent() {
        if !parent.exists() {
            fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }

    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, true, None)
        .expect("Failed to setup logger.");
    debug!("Logger created at {}.", path = logger_path);

    debug!("Creating application... ");

    let mut application = BalanceBotBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create runtime.");

    let clock = copper_ctx.clock;
    ctrlc::set_handler(move || {
        STOP_FLAG.store(true, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    debug!("Running... starting clock: {}.", clock.now());
    application
        .start_all_tasks()
        .expect("Failed to start all tasks.");
    while !STOP_FLAG.load(Ordering::SeqCst) {
        application
            .run_one_iteration()
            .expect("Failed to run application.");
    }
    application
        .stop_all_tasks()
        .expect("Failed to stop all tasks.");
    debug!("End of app: final clock: {}.", clock.now());
}
