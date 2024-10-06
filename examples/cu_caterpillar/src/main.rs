pub mod tasks;

use cu29_derive::copper_runtime;
use cu29_helpers::basic_copper_setup;
use cu29_log_derive::debug;
use std::path::PathBuf;

#[copper_runtime(config = "copperconfig.ron")]
struct CaterpillarApplication {}

const SLAB_SIZE: Option<usize> = Some(100 * 1024 * 1024);

fn main() {
    let logger_path = "/tmp/caterpillar.copper";
    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, false)
        .expect("Failed to setup logger.");
    println!("Logger created at {}.", path = logger_path);
    let clock = copper_ctx.clock.clone();
    let ulclone = copper_ctx.unified_logger.clone();
    println!("Creating application... ");
    let mut application =
        CaterpillarApplication::new(clock.clone(), ulclone).expect("Failed to create application.");
    println!("Running... starting clock: {}.", clock.now());

    let outcome = application.run();
    match outcome {
        Ok(result) => {}
        Err(error) => {
            debug!("Application Ended: {}", error)
        }
    }
    println!("End of main");
}
