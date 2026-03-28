pub mod tasks;

use cu29::prelude::*;
use std::fs;
use std::path::Path;

#[copper_runtime(config = "copperconfig.ron")]
struct BalanceBot {}

// preallocate a lot.
#[allow(clippy::identity_op)]
const SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);

fn main() {
    let logger_path = "logs/balance.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }

    debug!("Logger created at {}.", path = &logger_path);

    debug!("Creating application... ");

    let mut application = BalanceBot::builder()
        .with_log_path(logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create runtime.");

    debug!("Running... starting clock: {}.", application.clock().now());
    application.run().expect("Failed to run application.");
    debug!("End of app: final clock: {}.", application.clock().now());
}
