use clap::Parser;
use cu29::prelude::*;
use std::path::PathBuf;

mod autonomy_bridge;
mod compute_tasks;
mod messages;

mod tasks {
    pub use crate::compute_tasks::*;
}

#[copper_runtime(config = "flight_controller.ron", subsystem = "compute")]
struct ComputeApp {}

const LOG_SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);

#[derive(Parser)]
struct Args {
    /// Unified-log base path for the deployed compute runtime.
    #[arg(long, default_value = "logs/compute.copper")]
    log: PathBuf,
}

fn main() {
    if let Err(err) = drive() {
        eprintln!("quad-compute failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let args = Args::parse();
    let app = ComputeApp::builder()
        .with_log_path(args.log, LOG_SLAB_SIZE)?
        .build()?;
    app.run_until_shutdown()?;
    Ok(())
}
