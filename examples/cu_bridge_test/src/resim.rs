#![allow(unused_lifetimes)]

use clap::Parser;
use cu29::cutask::CuMsgMetadata;
use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29::remote_debug::SessionOpenParams;
use cu29::replay::{
    ReplayArgs, ReplayDefaults, ensure_log_family_exists, per_session_replay_log_base,
    remove_log_family, serve_remote_debug,
};
use cu29::simulation::CuTaskCallbackState;
use cu29_export::copperlists_reader;
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use std::path::Path;

// Bring the runtime modules generated in the library into scope for the macro below.
use cu_bridge_test::{MissionArg, bridges, messages, tasks};

const LOG_SLAB_SIZE: Option<usize> = Some(32 * 1024 * 1024);
const INPUT_LOG: &str = "logs/cu_bridge_test.copper";
const RESIM_LOG: &str = "logs/cu_bridge_test_resim.copper";

#[derive(Parser, Debug, Clone)]
#[command(author, version, about = "Resimulate cu-bridge-test logs", long_about = None)]
struct Cli {
    /// Mission graph to resimulate
    #[arg(value_enum, default_value_t = MissionArg::BridgeFanout, value_name = "MISSION")]
    mission: MissionArg,

    #[command(flatten)]
    replay: ReplayArgs,
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
    let replay = args
        .replay
        .resolve(&ReplayDefaults::new(INPUT_LOG, RESIM_LOG));

    if let Some(debug_base) = replay.debug_base.as_deref() {
        drive_remote_debug(
            args.mission,
            debug_base,
            &replay.log_base,
            &replay.replay_log_base,
        )
    } else {
        ensure_log_family_exists(&replay.log_base)?;
        drive_one_shot(args.mission, &replay.log_base, &replay.replay_log_base)
    }
}

fn drive_one_shot(mission: MissionArg, log_base: &Path, replay_log_base: &Path) -> CuResult<()> {
    remove_log_family(replay_log_base)?;

    let (robot_clock, robot_clock_mock) = RobotClock::mock();
    match mission {
        MissionArg::BridgeOnlyAb => resim_bridge_only_ab(
            replay_log_base,
            robot_clock.clone(),
            &robot_clock_mock,
            log_base,
        )?,
        MissionArg::BridgeLoopback => resim_bridge_loopback(
            replay_log_base,
            robot_clock.clone(),
            &robot_clock_mock,
            log_base,
        )?,
        MissionArg::SourceToBridge => resim_source_to_bridge(
            replay_log_base,
            robot_clock.clone(),
            &robot_clock_mock,
            log_base,
        )?,
        MissionArg::BridgeToSink => resim_bridge_to_sink(
            replay_log_base,
            robot_clock.clone(),
            &robot_clock_mock,
            log_base,
        )?,
        MissionArg::BridgeTaskSame => resim_bridge_task_same(
            replay_log_base,
            robot_clock.clone(),
            &robot_clock_mock,
            log_base,
        )?,
        MissionArg::BridgeFanout => {
            resim_bridge_fanout(replay_log_base, robot_clock, &robot_clock_mock, log_base)?
        }
    }

    Ok(())
}

fn drive_remote_debug(
    mission: MissionArg,
    debug_base: &str,
    log_base: &Path,
    replay_log_base: &Path,
) -> CuResult<()> {
    match mission {
        MissionArg::BridgeOnlyAb => {
            run_bridge_only_ab_server(debug_base, log_base, replay_log_base)
        }
        MissionArg::BridgeLoopback => {
            run_bridge_loopback_server(debug_base, log_base, replay_log_base)
        }
        MissionArg::SourceToBridge => {
            run_source_to_bridge_server(debug_base, log_base, replay_log_base)
        }
        MissionArg::BridgeToSink => {
            run_bridge_to_sink_server(debug_base, log_base, replay_log_base)
        }
        MissionArg::BridgeTaskSame => {
            run_bridge_task_same_server(debug_base, log_base, replay_log_base)
        }
        MissionArg::BridgeFanout => run_bridge_fanout_server(debug_base, log_base, replay_log_base),
    }
}

fn sync_robot_clock<'m>(
    metadata: impl IntoIterator<Item = &'m CuMsgMetadata>,
    robot_clock: &RobotClockMock,
) {
    if let Some(start) = metadata.into_iter().find_map(|meta| {
        let maybe_start: Option<CuTime> = meta.process_time.start.into();
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

macro_rules! resim_default {
    ($app:ident, $output_path:expr, $robot_clock:expr, $clock_mock:expr, $input_log_path:expr) => {{
        let mut default_cb = |_step: $app::SimStep<'_>| SimOverride::ExecuteByRuntime;
        let mut app = $app::BridgeSchedulerReSim::builder()
            .with_clock($robot_clock)
            .with_log_path($output_path, LOG_SLAB_SIZE)?
            .with_sim_callback(&mut default_cb)
            .build()?;
        app.start_all_tasks(&mut default_cb)?;

        let mut reader = open_copperlist_reader($input_log_path)?;
        for entry in copperlists_reader::<$app::CuStampedDataSet>(&mut reader) {
            sync_robot_clock($app::collect_metadata(&entry), $clock_mock);
            let mut sim_cb = |_step: $app::SimStep<'_>| SimOverride::ExecuteByRuntime;
            app.run_one_iteration(&mut sim_cb)?;
        }

        app.stop_all_tasks(&mut default_cb)?;
        Ok(())
    }};
}

fn resim_bridge_only_ab(
    output_path: &Path,
    clock: RobotClock,
    clock_mock: &RobotClockMock,
    input_log_path: &Path,
) -> CuResult<()> {
    resim_default!(BridgeOnlyAB, output_path, clock, clock_mock, input_log_path)
}

fn resim_bridge_loopback(
    output_path: &Path,
    clock: RobotClock,
    clock_mock: &RobotClockMock,
    input_log_path: &Path,
) -> CuResult<()> {
    resim_default!(
        BridgeLoopback,
        output_path,
        clock,
        clock_mock,
        input_log_path
    )
}

fn resim_bridge_fanout(
    output_path: &Path,
    clock: RobotClock,
    clock_mock: &RobotClockMock,
    input_log_path: &Path,
) -> CuResult<()> {
    resim_default!(BridgeFanout, output_path, clock, clock_mock, input_log_path)
}

fn resim_source_to_bridge(
    output_path: &Path,
    clock: RobotClock,
    clock_mock: &RobotClockMock,
    input_log_path: &Path,
) -> CuResult<()> {
    let mut default_cb = |_step: SourceToBridge::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut app = SourceToBridge::BridgeSchedulerReSim::builder()
        .with_clock(clock)
        .with_log_path(output_path, LOG_SLAB_SIZE)?
        .with_sim_callback(&mut default_cb)
        .build()?;
    app.start_all_tasks(&mut default_cb)?;

    let mut reader = open_copperlist_reader(input_log_path)?;
    for entry in copperlists_reader::<SourceToBridge::CuStampedDataSet>(&mut reader) {
        sync_robot_clock(SourceToBridge::collect_metadata(&entry), clock_mock);
        let mut sim_cb = make_source_to_bridge_cb(&entry);
        app.run_one_iteration(&mut sim_cb)?;
    }

    app.stop_all_tasks(&mut default_cb)?;
    Ok(())
}

fn make_source_to_bridge_cb(
    entry: &CopperList<SourceToBridge::CuStampedDataSet>,
) -> impl FnMut(SourceToBridge::SimStep<'_>) -> SimOverride {
    let expected_output = entry.msgs.0.0.clone();
    move |step| match step {
        SourceToBridge::SimStep::SrcToBridge(CuTaskCallbackState::Process(_, output)) => {
            *output = expected_output.clone();
            SimOverride::ExecutedBySim
        }
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn resim_bridge_to_sink(
    output_path: &Path,
    clock: RobotClock,
    clock_mock: &RobotClockMock,
    input_log_path: &Path,
) -> CuResult<()> {
    let mut default_cb = |_step: BridgeToSink::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut app = BridgeToSink::BridgeSchedulerReSim::builder()
        .with_clock(clock)
        .with_log_path(output_path, LOG_SLAB_SIZE)?
        .with_sim_callback(&mut default_cb)
        .build()?;
    app.start_all_tasks(&mut default_cb)?;

    let mut reader = open_copperlist_reader(input_log_path)?;
    for entry in copperlists_reader::<BridgeToSink::CuStampedDataSet>(&mut reader) {
        sync_robot_clock(BridgeToSink::collect_metadata(&entry), clock_mock);
        let mut sim_cb = make_bridge_to_sink_cb(&entry);
        app.run_one_iteration(&mut sim_cb)?;
    }

    app.stop_all_tasks(&mut default_cb)?;
    Ok(())
}

fn make_bridge_to_sink_cb(
    entry: &CopperList<BridgeToSink::CuStampedDataSet>,
) -> impl FnMut(BridgeToSink::SimStep<'_>) -> SimOverride {
    let sink_output = entry.msgs.0.1.clone();
    move |step| match step {
        BridgeToSink::SimStep::SinkFromBridge(CuTaskCallbackState::Process(_, output)) => {
            *output = sink_output.clone();
            SimOverride::ExecutedBySim
        }
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn resim_bridge_task_same(
    output_path: &Path,
    clock: RobotClock,
    clock_mock: &RobotClockMock,
    input_log_path: &Path,
) -> CuResult<()> {
    let mut default_cb = |_step: BridgeTaskSame::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut app = BridgeTaskSame::BridgeSchedulerReSim::builder()
        .with_clock(clock)
        .with_log_path(output_path, LOG_SLAB_SIZE)?
        .with_sim_callback(&mut default_cb)
        .build()?;
    app.start_all_tasks(&mut default_cb)?;

    let mut reader = open_copperlist_reader(input_log_path)?;
    for entry in copperlists_reader::<BridgeTaskSame::CuStampedDataSet>(&mut reader) {
        sync_robot_clock(BridgeTaskSame::collect_metadata(&entry), clock_mock);
        let mut sim_cb = make_bridge_task_same_cb(&entry);
        app.run_one_iteration(&mut sim_cb)?;
    }

    app.stop_all_tasks(&mut default_cb)?;
    Ok(())
}

fn make_bridge_task_same_cb(
    entry: &CopperList<BridgeTaskSame::CuStampedDataSet>,
) -> impl FnMut(BridgeTaskSame::SimStep<'_>) -> SimOverride {
    let passthrough_output = entry.msgs.0.1.clone();
    move |step| match step {
        BridgeTaskSame::SimStep::Passthrough(CuTaskCallbackState::Process(_, output)) => {
            *output = passthrough_output.clone();
            SimOverride::ExecutedBySim
        }
        _ => SimOverride::ExecuteByRuntime,
    }
}

macro_rules! define_remote_debug_mission {
    ($make_app:ident, $build_callback:ident, $extract_time:ident, $run_server:ident, $mission_label:expr, $module:ident) => {
        fn $make_app(
            params: &SessionOpenParams,
            replay_template: &Path,
        ) -> CuResult<($module::BridgeSchedulerReSim, RobotClock, RobotClockMock)> {
            let (clock, clock_mock) = RobotClock::mock();
            let replay_log_base = per_session_replay_log_base(
                replay_template,
                [$mission_label, params.role.as_deref().unwrap_or("session")],
            );
            remove_log_family(&replay_log_base)?;

            let mut default_cb = |_step: $module::SimStep<'_>| SimOverride::ExecuteByRuntime;
            let app = $module::BridgeSchedulerReSim::builder()
                .with_clock(clock.clone())
                .with_log_path(replay_log_base, LOG_SLAB_SIZE)?
                .with_sim_callback(&mut default_cb)
                .build()?;
            Ok((app, clock, clock_mock))
        }

        fn $build_callback<'a>(
            copperlist: &'a CopperList<$module::CuStampedDataSet>,
            _process_clock: RobotClock,
            _clock_for_callbacks: RobotClockMock,
        ) -> Box<dyn for<'z> FnMut($module::SimStep<'z>) -> SimOverride + 'a> {
            Box::new(move |step: $module::SimStep<'_>| {
                $module::recorded_replay_step(step, copperlist)
            })
        }

        fn $extract_time(copperlist: &CopperList<$module::CuStampedDataSet>) -> Option<CuTime> {
            cu29::simulation::recorded_copperlist_timestamp(copperlist)
        }

        fn $run_server(debug_base: &str, log_base: &Path, replay_template: &Path) -> CuResult<()> {
            let replay_template = replay_template.to_path_buf();
            serve_remote_debug::<
                $module::BridgeSchedulerReSim,
                $module::CuStampedDataSet,
                _,
                _,
                MmapSectionStorage,
                MmapUnifiedLoggerWrite,
                _,
            >(
                debug_base,
                log_base,
                move |params| $make_app(params, &replay_template),
                $build_callback,
                $extract_time,
            )
        }
    };
}

define_remote_debug_mission!(
    make_bridge_only_ab_app,
    build_bridge_only_ab_callback,
    extract_bridge_only_ab_time,
    run_bridge_only_ab_server,
    "bridge_only_ab",
    BridgeOnlyAB
);
define_remote_debug_mission!(
    make_bridge_loopback_app,
    build_bridge_loopback_callback,
    extract_bridge_loopback_time,
    run_bridge_loopback_server,
    "bridge_loopback",
    BridgeLoopback
);
define_remote_debug_mission!(
    make_source_to_bridge_app,
    build_source_to_bridge_callback,
    extract_source_to_bridge_time,
    run_source_to_bridge_server,
    "source_to_bridge",
    SourceToBridge
);
define_remote_debug_mission!(
    make_bridge_to_sink_app,
    build_bridge_to_sink_callback,
    extract_bridge_to_sink_time,
    run_bridge_to_sink_server,
    "bridge_to_sink",
    BridgeToSink
);
define_remote_debug_mission!(
    make_bridge_task_same_app,
    build_bridge_task_same_callback,
    extract_bridge_task_same_time,
    run_bridge_task_same_server,
    "bridge_task_same",
    BridgeTaskSame
);
define_remote_debug_mission!(
    make_bridge_fanout_app,
    build_bridge_fanout_callback,
    extract_bridge_fanout_time,
    run_bridge_fanout_server,
    "bridge_fanout",
    BridgeFanout
);
