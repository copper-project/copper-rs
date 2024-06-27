pub mod tasks;

use copper::clock::ClockProvider;
use copper_derive::copper_runtime;
use copper_helpers::basic_logger_runtime_setup;
use copper_log_derive::debug;
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;

#[copper_runtime(config = "copperconfig.ron")]
struct CaterpillarApplication {}

fn main() {
    let logger_runtime =
        basic_logger_runtime_setup(&PathBuf::from("/tmp/caterpillar.copper"), true)
            .expect("Failed to setup logger.");
    debug!("Logger created.");
    let clock = logger_runtime.get_clock();
    debug!("Creating application... ");
    let mut application =
        CaterpillarApplication::new(clock.clone()).expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());
    application.run(2).expect("Failed to run application.");
    debug!("End of program: {}.", clock.now());
    sleep(Duration::from_secs(1));
}
