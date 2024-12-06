pub mod tasks;

use cu29_prelude::*;
use std::path::PathBuf;

#[copper_runtime(config = "copperconfig.ron")]
struct CaterpillarApplication {}

const SLAB_SIZE: Option<usize> = Some(100 * 1024 * 1024);

fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("caterpillar.copper");

    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, false, None)
        .expect("Failed to setup logger.");
    let clock = copper_ctx.clock.clone();
    let ulclone = copper_ctx.unified_logger.clone();
    let mut application =
        CaterpillarApplication::new(clock.clone(), ulclone).expect("Failed to create application.");

    let outcome = application.run();
    match outcome {
        Ok(_result) => {}
        Err(error) => {
            debug!("Application Ended: {}", error)
        }
    }
}
