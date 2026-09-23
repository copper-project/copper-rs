//! Runs a compile-time selected PGS candidate configuration.

mod messages;
mod tasks;

use clap::Parser;
use cu29::prelude::*;
use std::path::PathBuf;

#[derive(Parser)]
struct Args {
    /// Unified log base path for the measured candidate.
    #[arg(long)]
    log: PathBuf,
}

#[copper_runtime(config = "target/pgs/selected.config.ron")]
struct PgsCandidateApplication {}

fn main() -> CuResult<()> {
    let args = Args::parse();
    if let Some(parent) = args.log.parent() {
        std::fs::create_dir_all(parent)
            .map_err(|error| CuError::new_with_cause("Could not create PGS log directory", error))?;
    }
    let app = PgsCandidateApplication::builder()
        .with_log_path(&args.log, Some(100 * 1024 * 1024))?
        .build()?;
    app.run_until_shutdown()?;
    Ok(())
}
