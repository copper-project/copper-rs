use clap::Parser;
use cu_bridge_test::{MissionArg, run_mission};
use cu29::prelude::*;
use std::{fs, path::PathBuf};

#[derive(Parser)]
#[command(author, version, about = "Run bridge scheduling examples", long_about = None)]
struct Cli {
    /// Mission graph to run
    #[arg(value_enum, default_value_t = MissionArg::BridgeFanout, value_name = "MISSION")]
    mission: MissionArg,
}

fn main() {
    if let Err(err) = drive() {
        eprintln!("cu-bridge-test failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let args = Cli::parse();

    let logger_path = PathBuf::from("logs/cu_bridge_test.copper");
    if let Some(parent) = logger_path.parent()
        && let Err(err) = fs::create_dir_all(parent)
    {
        return Err(CuError::new_with_cause(
            "failed to create log directory",
            err,
        ));
    }

    run_mission(args.mission, &logger_path)
}
