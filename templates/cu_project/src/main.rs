pub mod tasks;

use cu29::prelude::*;
use std::path::Path;
use std::thread::sleep;
use std::time::Duration;

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
    debug!("Logger created at {}.", logger_path);
    debug!("Creating application... ");
    let mut application = {{project-name | upper_camel_case}}Application::builder()
        .with_log_path(logger_path, PREALLOCATED_STORAGE_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");
    debug!("Running... starting clock: {}.", application.clock().now());

    application.run().expect("Failed to run application.");
    debug!("End of program: {}.", application.clock().now());
    sleep(Duration::from_secs(1));
}
