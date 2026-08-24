use clap::Parser;
use std::path::PathBuf;
use std::time::Duration;

#[derive(Parser)]
#[command(about = "Run the Copper Autoware reference-system benchmark")]
struct Args {
    /// Wall-clock measurement duration. Ignored when --iterations is supplied.
    #[arg(long, default_value_t = 60)]
    seconds: u64,
    /// Run an exact number of Copper graph passes instead of using wall time.
    #[arg(long)]
    iterations: Option<usize>,
    /// Unified Copper log base.
    #[arg(long)]
    log_base: Option<PathBuf>,
    /// Runtime RON configuration, normally the host-calibrated generated copy.
    #[arg(long)]
    config: Option<PathBuf>,
}

fn main() {
    let args = Args::parse();
    let limit = args.iterations.map_or_else(
        || cu_autoware::RunLimit::Duration(Duration::from_secs(args.seconds)),
        cu_autoware::RunLimit::Iterations,
    );
    cu_autoware::run(limit, args.log_base, args.config);
}
