pub mod tasks;

use cu29::prelude::*;
use std::thread::sleep;
use std::time::Duration;

#[copper_runtime(config = "copperconfig.ron")]
struct MultiSourceApp {}

const SLAB_SIZE: Option<usize> = Some(1024 * 1024);

fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("caterpillar.copper");
    debug!("Logger created at {}.", path = &logger_path);
    debug!("Creating application... ");
    let mut application = MultiSourceApp::builder()
        .with_log_path(&logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create runtime.");

    application.run().expect("Failed to run application.");
    sleep(Duration::from_secs(1));
}
