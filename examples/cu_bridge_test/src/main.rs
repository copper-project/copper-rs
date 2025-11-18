use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use cu29_unifiedlog::{memmap::MmapSectionStorage, UnifiedLoggerWrite};
use std::{env, fs, path::PathBuf};

use cu_bridge_test::{
    BridgeLoopbackBuilder, BridgeOnlyABBuilder, BridgeTaskSameBuilder, BridgeToSinkBuilder,
    SourceToBridgeBuilder,
};

const SLAB_SIZE: Option<usize> = Some(32 * 1024 * 1024);
const DEFAULT_MISSION: &str = "BridgeOnlyAB";

fn run_once<App>(app: &mut App) -> CuResult<()>
where
    App: CuApplication<MmapSectionStorage, UnifiedLoggerWrite>,
{
    app.start_all_tasks()?;
    app.run_one_iteration()?;
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
    let mission = env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_MISSION.to_string());

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

    match mission.as_str() {
        "BridgeOnlyAB" => {
            let mut app = BridgeOnlyABBuilder::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
        "BridgeLoopback" => {
            let mut app = BridgeLoopbackBuilder::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
        "SourceToBridge" => {
            let mut app = SourceToBridgeBuilder::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
        "BridgeToSink" => {
            let mut app = BridgeToSinkBuilder::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
        "BridgeTaskSame" => {
            let mut app = BridgeTaskSameBuilder::new().with_context(&ctx).build()?;
            run_once(&mut app)?;
        }
        unknown => {
            eprintln!(
                "Unknown mission '{unknown}'. Choose one of: BridgeOnlyAB, BridgeLoopback, SourceToBridge, BridgeToSink, BridgeTaskSame."
            );
            std::process::exit(2);
        }
    }

    Ok(())
}
