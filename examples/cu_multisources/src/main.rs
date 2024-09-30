pub mod tasks;

use cu29_derive::copper_runtime;
use cu29_helpers::basic_copper_setup;
use cu29_log_derive::debug;
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;

#[copper_runtime(config = "copperconfig.ron")]
struct MultiSourceApp {}

const SLAB_SIZE: Option<usize> = Some(1024 * 1024);

fn main() {
    let logger_path = "/tmp/caterpillar.copper";
    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, true)
        .expect("Failed to setup logger.");
    debug!("Logger created at {}.", path = logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application = MultiSourceApp::new(clock.clone(), copper_ctx.unified_logger.clone())
        .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());

    application.run().expect("Failed to run application.");
    debug!("End of program: {}.", clock.now());
    sleep(Duration::from_secs(1));
}
