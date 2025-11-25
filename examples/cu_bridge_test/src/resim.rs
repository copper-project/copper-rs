#![allow(unused_lifetimes)]

use clap::{Parser, ValueEnum};
use cu29::clock::CuDuration;
use cu29::cutask::CuMsgMetadata;
use cu29::prelude::*;
use cu29::simulation::CuTaskCallbackState;
use cu29_export::copperlists_reader;
use cu29_helpers::basic_copper_setup;
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use std::{
    fs,
    path::{Path, PathBuf},
};

// Bring the runtime modules generated in the library into scope for the macro below.
use cu_bridge_test::{bridges, messages, tasks};

const LOG_SLAB_SIZE: Option<usize> = Some(32 * 1024 * 1024);
const INPUT_LOG: &str = "logs/cu_bridge_test.copper";
const RESIM_LOG: &str = "logs/cu_bridge_test_resim.copper";

#[derive(Parser)]
#[command(author, version, about = "Resimulate cu-bridge-test logs", long_about = None)]
struct Cli {
    /// Mission graph to resimulate
    #[arg(value_enum, default_value_t = MissionArg::BridgeFanout, value_name = "MISSION")]
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

// Enable simulation support for the existing copper runtime.
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct BridgeSchedulerReSim {}

fn main() {
    if let Err(err) = drive() {
        eprintln!("cu-bridge-test resim failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let args = Cli::parse();
    let input_path = Path::new(INPUT_LOG);
    let output_path = PathBuf::from(RESIM_LOG);
    ensure_log_dir(&output_path)?;

    let (robot_clock, robot_clock_mock) = RobotClock::mock();
    let ctx = basic_copper_setup(&output_path, LOG_SLAB_SIZE, true, Some(robot_clock.clone()))?;

    match args.mission {
        MissionArg::BridgeOnlyAb => resim_bridge_only_ab(&ctx, &robot_clock_mock, input_path)?,
        MissionArg::BridgeLoopback => resim_bridge_loopback(&ctx, &robot_clock_mock, input_path)?,
        MissionArg::SourceToBridge => resim_source_to_bridge(&ctx, &robot_clock_mock, input_path)?,
        MissionArg::BridgeToSink => resim_bridge_to_sink(&ctx, &robot_clock_mock, input_path)?,
        MissionArg::BridgeTaskSame => resim_bridge_task_same(&ctx, &robot_clock_mock, input_path)?,
        MissionArg::BridgeFanout => resim_bridge_fanout(&ctx, &robot_clock_mock, input_path)?,
    }

    Ok(())
}

fn ensure_log_dir(path: &Path) -> CuResult<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .map_err(|err| CuError::new_with_cause("failed to create resim log directory", err))?;
    }
    Ok(())
}

fn sync_robot_clock<'m>(
    metadata: impl IntoIterator<Item = &'m CuMsgMetadata>,
    robot_clock: &RobotClockMock,
) {
    if let Some(start) = metadata.into_iter().find_map(|meta| {
        let maybe_start: Option<CuDuration> = meta.process_time.start.into();
        maybe_start
    }) {
        robot_clock.set_value(start.as_nanos());
    }
}

fn open_copperlist_reader(log_path: &Path) -> CuResult<UnifiedLoggerIOReader> {
    let UnifiedLogger::Read(reader) = UnifiedLoggerBuilder::new()
        .file_base_name(log_path)
        .build()
        .map_err(|err| CuError::new_with_cause("failed to open log for resim", err))?
    else {
        return Err(CuError::from(format!(
            "log file {log_path:?} could not be opened for reading"
        )));
    };
    Ok(UnifiedLoggerIOReader::new(
        reader,
        UnifiedLogType::CopperList,
    ))
}

fn resim_bridge_only_ab(
    ctx: &CopperContext,
    robot_clock: &RobotClockMock,
    log_path: &Path,
) -> CuResult<()> {
    let mut default_cb = |_step: BridgeOnlyAB::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut app = BridgeOnlyAB::BridgeSchedulerReSimBuilder::new()
        .with_context(ctx)
        .with_sim_callback(&mut default_cb)
        .build()?;
    app.start_all_tasks(&mut default_cb)?;

    let mut reader = open_copperlist_reader(log_path)?;
    for entry in copperlists_reader::<BridgeOnlyAB::CuStampedDataSet>(&mut reader) {
        sync_robot_clock(BridgeOnlyAB::collect_metadata(&entry), robot_clock);
        let mut sim_cb = |_step: BridgeOnlyAB::SimStep<'_>| SimOverride::ExecuteByRuntime;
        app.run_one_iteration(&mut sim_cb)?;
    }

    app.stop_all_tasks(&mut default_cb)?;
    Ok(())
}

fn resim_bridge_loopback(
    ctx: &CopperContext,
    robot_clock: &RobotClockMock,
    log_path: &Path,
) -> CuResult<()> {
    let mut default_cb = |_step: BridgeLoopback::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut app = BridgeLoopback::BridgeSchedulerReSimBuilder::new()
        .with_context(ctx)
        .with_sim_callback(&mut default_cb)
        .build()?;
    app.start_all_tasks(&mut default_cb)?;

    let mut reader = open_copperlist_reader(log_path)?;
    for entry in copperlists_reader::<BridgeLoopback::CuStampedDataSet>(&mut reader) {
        sync_robot_clock(BridgeLoopback::collect_metadata(&entry), robot_clock);
        let mut sim_cb = |_step: BridgeLoopback::SimStep<'_>| SimOverride::ExecuteByRuntime;
        app.run_one_iteration(&mut sim_cb)?;
    }

    app.stop_all_tasks(&mut default_cb)?;
    Ok(())
}

fn resim_bridge_fanout(
    ctx: &CopperContext,
    robot_clock: &RobotClockMock,
    log_path: &Path,
) -> CuResult<()> {
    let mut default_cb = |_step: BridgeFanout::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut app = BridgeFanout::BridgeSchedulerReSimBuilder::new()
        .with_context(ctx)
        .with_sim_callback(&mut default_cb)
        .build()?;
    app.start_all_tasks(&mut default_cb)?;

    let mut reader = open_copperlist_reader(log_path)?;
    for entry in copperlists_reader::<BridgeFanout::CuStampedDataSet>(&mut reader) {
        sync_robot_clock(BridgeFanout::collect_metadata(&entry), robot_clock);
        let mut sim_cb = |_step: BridgeFanout::SimStep<'_>| SimOverride::ExecuteByRuntime;
        app.run_one_iteration(&mut sim_cb)?;
    }

    app.stop_all_tasks(&mut default_cb)?;
    Ok(())
}

fn resim_source_to_bridge(
    ctx: &CopperContext,
    robot_clock: &RobotClockMock,
    log_path: &Path,
) -> CuResult<()> {
    let mut default_cb = |_step: SourceToBridge::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut app = SourceToBridge::BridgeSchedulerReSimBuilder::new()
        .with_context(ctx)
        .with_sim_callback(&mut default_cb)
        .build()?;
    app.start_all_tasks(&mut default_cb)?;

    let mut reader = open_copperlist_reader(log_path)?;
    for entry in copperlists_reader::<SourceToBridge::CuStampedDataSet>(&mut reader) {
        sync_robot_clock(SourceToBridge::collect_metadata(&entry), robot_clock);
        let mut sim_cb = make_source_to_bridge_cb(&entry);
        app.run_one_iteration(&mut sim_cb)?;
    }

    app.stop_all_tasks(&mut default_cb)?;
    Ok(())
}

fn make_source_to_bridge_cb(
    entry: &CopperList<SourceToBridge::CuStampedDataSet>,
) -> impl FnMut(SourceToBridge::SimStep<'_>) -> SimOverride {
    let expected_output = entry.msgs.0 .0.clone();
    move |step| match step {
        SourceToBridge::SimStep::SrcToBridge(CuTaskCallbackState::Process(_, output)) => {
            *output = expected_output.clone();
            SimOverride::ExecutedBySim
        }
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn resim_bridge_to_sink(
    ctx: &CopperContext,
    robot_clock: &RobotClockMock,
    log_path: &Path,
) -> CuResult<()> {
    let mut default_cb = |_step: BridgeToSink::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut app = BridgeToSink::BridgeSchedulerReSimBuilder::new()
        .with_context(ctx)
        .with_sim_callback(&mut default_cb)
        .build()?;
    app.start_all_tasks(&mut default_cb)?;

    let mut reader = open_copperlist_reader(log_path)?;
    for entry in copperlists_reader::<BridgeToSink::CuStampedDataSet>(&mut reader) {
        sync_robot_clock(BridgeToSink::collect_metadata(&entry), robot_clock);
        let mut sim_cb = make_bridge_to_sink_cb(&entry);
        app.run_one_iteration(&mut sim_cb)?;
    }

    app.stop_all_tasks(&mut default_cb)?;
    Ok(())
}

fn make_bridge_to_sink_cb(
    entry: &CopperList<BridgeToSink::CuStampedDataSet>,
) -> impl FnMut(BridgeToSink::SimStep<'_>) -> SimOverride {
    let sink_output = entry.msgs.0 .1.clone();
    move |step| match step {
        BridgeToSink::SimStep::SinkFromBridge(CuTaskCallbackState::Process(_, output)) => {
            *output = sink_output.clone();
            SimOverride::ExecutedBySim
        }
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn resim_bridge_task_same(
    ctx: &CopperContext,
    robot_clock: &RobotClockMock,
    log_path: &Path,
) -> CuResult<()> {
    let mut default_cb = |_step: BridgeTaskSame::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut app = BridgeTaskSame::BridgeSchedulerReSimBuilder::new()
        .with_context(ctx)
        .with_sim_callback(&mut default_cb)
        .build()?;
    app.start_all_tasks(&mut default_cb)?;

    let mut reader = open_copperlist_reader(log_path)?;
    for entry in copperlists_reader::<BridgeTaskSame::CuStampedDataSet>(&mut reader) {
        sync_robot_clock(BridgeTaskSame::collect_metadata(&entry), robot_clock);
        let mut sim_cb = make_bridge_task_same_cb(&entry);
        app.run_one_iteration(&mut sim_cb)?;
    }

    app.stop_all_tasks(&mut default_cb)?;
    Ok(())
}

fn make_bridge_task_same_cb(
    entry: &CopperList<BridgeTaskSame::CuStampedDataSet>,
) -> impl FnMut(BridgeTaskSame::SimStep<'_>) -> SimOverride {
    let passthrough_output = entry.msgs.0 .1.clone();
    move |step| match step {
        BridgeTaskSame::SimStep::Passthrough(CuTaskCallbackState::Process(_, output)) => {
            *output = passthrough_output.clone();
            SimOverride::ExecutedBySim
        }
        _ => SimOverride::ExecuteByRuntime,
    }
}
