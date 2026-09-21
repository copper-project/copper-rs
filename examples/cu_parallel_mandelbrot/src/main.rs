use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
struct Args {
    /// Unified-log base path. Defaults to this example's logs directory.
    #[arg(long)]
    log: Option<PathBuf>,
}

fn main() -> cu29::prelude::CuResult<()> {
    match Args::parse().log {
        Some(path) => cu_parallel_mandelbrot::run_log_only_with_path(&path),
        None => cu_parallel_mandelbrot::run_log_only(),
    }
}
