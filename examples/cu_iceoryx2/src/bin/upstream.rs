use cu29_helpers::basic_copper_setup;
use cu29::prelude::*;
use std::path::PathBuf;

#[copper_runtime(config = "upstream.ron")]
struct UpstreamApplication {}

const SLAB_SIZE: Option<usize> = Some(100 * 1024 * 1024);

fn main() {
    let logger_path = "/tmp/upstream.copper";
    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, false, None)
        .expect("Failed to setup logger.");
    let clock = copper_ctx.clock.clone();
    let ulclone = copper_ctx.unified_logger.clone();
    let mut application =
        UpstreamApplication::new(clock.clone(), ulclone).expect("Failed to create application.");
    let outcome = application.run();
    match outcome {
        Ok(_result) => {}
        Err(error) => {
            debug!("Application Ended: {}", error)
        }
    }
}
