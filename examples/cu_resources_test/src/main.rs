use clap::{Parser, ValueEnum};
use cu29::prelude::*;
use cu29_unifiedlog::{UnifiedLoggerWrite, memmap::MmapSectionStorage};
use std::{fs, path::PathBuf};

mod bridges;
mod resources;
mod tasks;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}
// Mission-specific builders emitted by the macro.
use A::App as MissionAApp;
use B::App as MissionBApp;

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

fn run_once<App>(app: App) -> CuResult<()>
where
    App: CuApplication<MmapSectionStorage, UnifiedLoggerWrite>,
{
    // `run` drives the full start/iterate/stop cycle; the typestate wrapper
    // makes the previous extra start_all_tasks/stop_all_tasks calls around it
    // a compile error instead of a double start/stop at runtime.
    CuAppLifecycle::new(app).run()?;
    Ok(())
}

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

    match args.mission {
        MissionArg::A => {
            let app = MissionAApp::builder()
                .with_log_path(&logger_path, SLAB_SIZE)?
                .build()?;
            run_once(app)?;
        }
        MissionArg::B => {
            let app = MissionBApp::builder()
                .with_log_path(&logger_path, SLAB_SIZE)?
                .build()?;
            run_once(app)?;
        }
    }

    Ok(())
}
