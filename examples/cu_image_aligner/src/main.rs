pub mod tasks;

use cu29::prelude::*;

#[copper_runtime(config = "copperconfig.ron")]
struct ImageAlignerApp {}

const SLAB_SIZE: Option<usize> = Some(32 * 1024 * 1024);
const ITERATIONS: usize = 20;

fn main() {
    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("image-aligner.copper");
    debug!("Logger created at {}.", path = &logger_path);
    debug!("Creating application...");
    let clock = RobotClock::default();

    let mut application = ImageAlignerApp::builder()
        .with_clock(clock.clone())
        .with_log_path(&logger_path, SLAB_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());

    application
        .start_all_tasks()
        .expect("Failed to start tasks.");
    for _ in 0..ITERATIONS {
        application
            .run_one_iteration()
            .expect("Failed to run iteration.");
    }
    debug!("End of program: {}.", clock.now());
}
