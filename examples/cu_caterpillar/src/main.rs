pub mod tasks;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::fs;
use std::path::{Path, PathBuf};

#[copper_runtime(config = "copperconfig.ron")]
struct CaterpillarApplication {}

const SLAB_SIZE: Option<usize> = Some(1024 * 1024 * 1024);

fn main() {
    let logger_path = "logs/caterpillar.copper";
    if let Some(parent) = Path::new(logger_path).parent() {
        if !parent.exists() {
            fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }

    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, true, None)
        .expect("Failed to setup logger.");
    let mut application = CaterpillarApplicationBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create application.");

    let outcome = application.run();
    match outcome {
        Ok(_result) => {}
        Err(error) => {
            debug!("Application Ended: {}", error)
        }
    }
}
