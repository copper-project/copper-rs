use clap::{Parser, ValueEnum};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use cu29_unifiedlog::{UnifiedLoggerWrite, memmap::MmapSectionStorage};
use std::{fs, path::PathBuf};

mod bridges;
mod resources;
mod tasks;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

// Export bridges under the module name referenced in copperconfig.
pub use bridges;

// Mission-specific builders emitted by the macro.
use A::AppBuilder as AppBuilderA;
use B::AppBuilder as AppBuilderB;

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

fn run_once<App>(app: &mut App) -> CuResult<()>
where
    App: CuApplication<MmapSectionStorage, UnifiedLoggerWrite>,
{
    app.start_all_tasks()?;
    app.run()?;
    app.stop_all_tasks()?;
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

    let ctx = basic_copper_setup(&logger_path, SLAB_SIZE, true, None)?;

    match args.mission {
        MissionArg::A => {
            let mut app = AppBuilderA::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
        MissionArg::B => {
            let mut app = AppBuilderB::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
    }

    Ok(())
}
