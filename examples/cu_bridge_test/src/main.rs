use clap::{Parser, ValueEnum};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use cu29_unifiedlog::{memmap::MmapSectionStorage, UnifiedLoggerWrite};
use std::{fs, path::PathBuf};

use cu_bridge_test::{
    BridgeFanoutBuilder, BridgeLoopbackBuilder, BridgeOnlyABBuilder, BridgeTaskSameBuilder,
    BridgeToSinkBuilder, SourceToBridgeBuilder,
};

const SLAB_SIZE: Option<usize> = Some(32 * 1024 * 1024);

#[derive(Parser)]
#[command(author, version, about = "Run bridge scheduling examples", long_about = None)]
struct Cli {
    /// Mission graph to run
    #[arg(value_enum, default_value_t = MissionArg::BridgeOnlyAb, value_name = "MISSION")]
    mission: MissionArg,
}

#[derive(Copy, Clone, Debug, ValueEnum)]
enum MissionArg {
    #[value(name = "BridgeOnlyAB")]
    BridgeOnlyAb,
    #[value(name = "BridgeLoopback")]
    BridgeLoopback,
    #[value(name = "SourceToBridge")]
    SourceToBridge,
    #[value(name = "BridgeToSink")]
    BridgeToSink,
    #[value(name = "BridgeTaskSame")]
    BridgeTaskSame,
    #[value(name = "BridgeFanout")]
    BridgeFanout,
}

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
        eprintln!("cu-bridge-test failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let args = Cli::parse();

    let logger_path = PathBuf::from("logs/cu_bridge_test.copper");
    if let Some(parent) = logger_path.parent() {
        if let Err(err) = fs::create_dir_all(parent) {
            return Err(CuError::new_with_cause(
                "failed to create log directory",
                err,
            ));
        }
    }

    let ctx = basic_copper_setup(&logger_path, SLAB_SIZE, true, None)?;

    match args.mission {
        MissionArg::BridgeOnlyAb => {
            let mut app = BridgeOnlyABBuilder::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
        MissionArg::BridgeLoopback => {
            let mut app = BridgeLoopbackBuilder::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
        MissionArg::SourceToBridge => {
            let mut app = SourceToBridgeBuilder::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
        MissionArg::BridgeToSink => {
            let mut app = BridgeToSinkBuilder::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
        MissionArg::BridgeTaskSame => {
            let mut app = BridgeTaskSameBuilder::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
        MissionArg::BridgeFanout => {
            let mut app = BridgeFanoutBuilder::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
    }

    Ok(())
}
