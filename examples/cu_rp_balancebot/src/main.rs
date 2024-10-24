pub mod tasks;

use cu29_derive::copper_runtime;
use cu29_helpers::basic_copper_setup;
use cu29_log_derive::debug;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};

#[copper_runtime(config = "copperconfig.ron")]
struct BalanceBot {}

// preallocate a lot.
const SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);

fn main() {
    static STOP_FLAG: AtomicBool = AtomicBool::new(false);
    let logger_path = "logs/balance.copper";
    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, false, None)
        .expect("Failed to setup logger.");
    debug!("Logger created at {}.", path = logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");

    let mut application = BalanceBot::new(clock.clone(), copper_ctx.unified_logger.clone())
        .expect("Failed to create runtime.");

    ctrlc::set_handler(move || {
        println!("Ctrl-C pressed. Stopping all tasks...");
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
