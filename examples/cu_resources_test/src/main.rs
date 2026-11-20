use clap::{Parser, ValueEnum};
use cu29::prelude::*;
use std::{fs, path::PathBuf};

mod bridges;
mod resources;
mod tasks;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}
#[derive(Parser)]
#[command(author, version, about = "Resource coverage demo", long_about = None)]
struct Cli {
    /// Mission graph to run
    #[arg(value_enum, default_value_t = MissionArg::A, value_name = "MISSION")]
    mission: MissionArg,
}

#[derive(Copy, Clone, Debug, ValueEnum)]
enum MissionArg {
    #[value(name = "A")]
    A,
    #[value(name = "B")]
    B,
}

const SLAB_SIZE: Option<usize> = None;

fn main() {
    if let Err(err) = drive() {
        eprintln!("cu-resources-test failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let args = Cli::parse();

    let logger_path = PathBuf::from("logs/cu_resources_test.copper");
    if let Some(parent) = logger_path.parent()
        && let Err(err) = fs::create_dir_all(parent)
    {
        return Err(CuError::new_with_cause(
            "failed to create log directory",
            err,
        ));
    }

    let mission = match args.mission {
        MissionArg::A => AppMission::A,
        MissionArg::B => AppMission::B,
    };
    let mut runner = AppRunner::builder()
        .with_log_path(&logger_path, SLAB_SIZE)?
        .build()?;
    runner.start(mission)?;
    runner.run_until(|| false)?;
    runner.stop()
}
