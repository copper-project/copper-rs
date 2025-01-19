pub mod tasks;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;

#[copper_runtime(config = "copperconfig.ron")]
struct CaterpillarApplication {}

const SLAB_SIZE: Option<usize> = Some(100 * 1024 * 1024);

fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("caterpillar.copper");

    let copper_ctx =
        basic_copper_setup(&logger_path, SLAB_SIZE, true, None).expect("Failed to setup logger.");
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
