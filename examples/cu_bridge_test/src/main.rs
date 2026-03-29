use clap::Parser;
use cu29::prelude::*;
use cu29_unifiedlog::{UnifiedLoggerWrite, memmap::MmapSectionStorage};
use std::{fs, path::PathBuf};

use cu_bridge_test::{
    BridgeFanoutApp, BridgeLoopbackApp, BridgeOnlyABApp, BridgeTaskSameApp, BridgeToSinkApp,
    MissionArg, SourceToBridgeApp,
};

const SLAB_SIZE: Option<usize> = Some(32 * 1024 * 1024);

#[derive(Parser)]
#[command(author, version, about = "Run bridge scheduling examples", long_about = None)]
struct Cli {
    /// Mission graph to run
    #[arg(value_enum, default_value_t = MissionArg::BridgeFanout, value_name = "MISSION")]
    mission: MissionArg,
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
    if let Some(parent) = logger_path.parent()
        && let Err(err) = fs::create_dir_all(parent)
    {
        return Err(CuError::new_with_cause(
            "failed to create log directory",
            err,
        ));
    }

    match args.mission {
        MissionArg::BridgeOnlyAb => {
            let mut app = BridgeOnlyABApp::builder()
                .with_log_path(&logger_path, SLAB_SIZE)?
                .build()?;
            run_once(&mut app)?;
        }
        MissionArg::BridgeLoopback => {
            let mut app = BridgeLoopbackApp::builder()
                .with_log_path(&logger_path, SLAB_SIZE)?
                .build()?;
            run_once(&mut app)?;
        }
        MissionArg::SourceToBridge => {
            let mut app = SourceToBridgeApp::builder()
                .with_log_path(&logger_path, SLAB_SIZE)?
                .build()?;
            run_once(&mut app)?;
        }
        MissionArg::BridgeToSink => {
            let mut app = BridgeToSinkApp::builder()
                .with_log_path(&logger_path, SLAB_SIZE)?
                .build()?;
            run_once(&mut app)?;
        }
        MissionArg::BridgeTaskSame => {
            let mut app = BridgeTaskSameApp::builder()
                .with_log_path(&logger_path, SLAB_SIZE)?
                .build()?;
            run_once(&mut app)?;
        }
        MissionArg::BridgeFanout => {
            let mut app = BridgeFanoutApp::builder()
                .with_log_path(&logger_path, SLAB_SIZE)?
                .build()?;
            run_once(&mut app)?;
        }
    }

    Ok(())
}
