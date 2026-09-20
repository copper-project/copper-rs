use clap::Parser;
use cu29::prelude::*;
use std::path::PathBuf;

mod autonomy_bridge;
mod compute_tasks;
mod messages;

mod tasks {
    pub use crate::compute_tasks::*;
}

#[derive(Parser)]
struct Args {
    /// Unified-log base path for this candidate run.
    #[arg(long)]
    log: PathBuf,
}

#[copper_runtime(config = "pgs/plan-1.config.ron")]
struct CandidateApp {}

fn main() -> CuResult<()> {
    let args = Args::parse();
    let app = CandidateApp::builder()
        .with_log_path(args.log, Some(64 * 1024 * 1024))?
        .build()?;
    app.run_until_shutdown()?;
    Ok(())
}
