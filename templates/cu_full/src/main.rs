pub mod tasks;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::thread::sleep;
use std::time::Duration;
use std::path::{Path,PathBuf};

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

#[copper_runtime(config = "copperconfig.ron")]
struct {{project-name | upper_camel_case}}Application {}

fn main() {
    let logger_path = "logs/{{project-name | kebab_case}}.copper";
    if let Some(parent) = Path::new(logger_path).parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }
    let copper_ctx =
        basic_copper_setup(&PathBuf::from(&logger_path), PREALLOCATED_STORAGE_SIZE, true, None).expect("Failed to setup logger.");
    debug!("Logger created at {}.", logger_path);
    debug!("Creating application... ");
    let mut application = {{project-name | upper_camel_case}}ApplicationBuilder::new()
            .with_context(&copper_ctx)
            .build()
            .expect("Failed to create application.");
    let clock = copper_ctx.clock.clone();
    debug!("Running... starting clock: {}.", clock.now());

    application.run().expect("Failed to run application.");
    debug!("End of program: {}.", clock.now());
    sleep(Duration::from_secs(1));
}
