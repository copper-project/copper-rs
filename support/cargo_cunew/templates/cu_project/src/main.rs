pub mod tasks;

use clap::Parser;
use cu29::prelude::*;
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

#[derive(Parser)]
struct Args {
    /// Unified log base path.
    #[arg(long, default_value = "logs/{{project-name|kebab_case}}.copper")]
    log: PathBuf,
}

#[copper_runtime(config = "copperconfig.ron")]
struct {{project-name | upper_camel_case}}Application {}

fn main() {
    let logger_path = Args::parse().log;
    if let Some(parent) = logger_path.parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }
    debug!("Logger created at {}.", logger_path.to_string_lossy().into_owned());
    debug!("Creating application... ");
    let application = {{project-name | upper_camel_case}}Application::builder()
        .with_log_path(&logger_path, PREALLOCATED_STORAGE_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");
    debug!(
        "Running... starting clock: {}.",
        application.clock().now()
    );

    let stopped = application.run_until_shutdown().expect("Failed to run application.");
    debug!("End of program: {}.", stopped.clock().now());
    sleep(Duration::from_secs(1));
}
