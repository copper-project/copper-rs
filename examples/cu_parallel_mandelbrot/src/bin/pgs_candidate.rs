use clap::Parser;
use cu_parallel_mandelbrot::{payloads, tasks};
use cu29::prelude::*;
use std::path::PathBuf;

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
    let app = log_only::CandidateApp::builder()
        .with_log_path(&args.log, Some(512 * 1024 * 1024))?
        .build()
        .map_err(|error| CuError::from(format!("failed to build PGS candidate: {error}")))?;
    app.run_until_shutdown()?;
    Ok(())
}
