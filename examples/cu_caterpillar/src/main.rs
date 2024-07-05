pub mod tasks;

use copper_derive::copper_runtime;
use copper_helpers::basic_copper_setup;
use copper_log_derive::debug;
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;

#[copper_runtime(config = "copperconfig.ron")]
struct CaterpillarApplication {}

fn main() {
    let logger_path = "/tmp/caterpillar.copper";
    let copper_ctx =
        basic_copper_setup(&PathBuf::from(logger_path), true).expect("Failed to setup logger.");
    debug!("Logger created at {}.", logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application =
        CaterpillarApplication::new(clock.clone(), copper_ctx.unified_logger.clone())
            .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());
    application.run(2).expect("Failed to run application.");
    debug!("End of program: {}.", clock.now());
    sleep(Duration::from_secs(1));
}
