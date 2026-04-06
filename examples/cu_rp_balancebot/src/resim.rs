pub mod tasks;

use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29::remote_debug::SessionOpenParams;
use cu29::replay::{
    ReplayCli, ReplayDefaults, ensure_log_family_exists, per_session_replay_log_base,
    remove_log_family, serve_remote_debug,
};
use cu29_export::copperlists_reader;
use cu29_export::keyframes_reader;
use std::error::Error;
use std::path::Path;

const DEFAULT_LOG_BASE: &str = "logs/balance.copper";
const DEFAULT_REPLAY_LOG_BASE: &str = "logs/balanceresim.copper";
#[allow(clippy::identity_op)]
const REPLAY_LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);

// To enable resim, it is just your regular macro with sim_mode true
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct BalanceBotReSim {}

type ReplayCopperList = CopperList<default::CuStampedDataSet>;
type ReplayBuildCallback =
    for<'a> fn(
        &'a ReplayCopperList,
        RobotClock,
    ) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a>;
type ReplayTimeExtractor = fn(&ReplayCopperList) -> Option<CuTime>;

fn default_callback(step: default::SimStep) -> SimOverride {
    match step {
        // Don't let the real task execute process and override with our logic.
        default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::BalposPid(_) => SimOverride::ExecutedBySim,
        default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::RailposPid(_) => SimOverride::ExecutedBySim,
        default::SimStep::MergePids(_) => SimOverride::ExecutedBySim,
        default::SimStep::Motor(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn run_one_copperlist(
    copper_app: &mut BalanceBotReSim,
    robot_clock: &mut RobotClockMock,
    copper_list: CopperList<default::CuStampedDataSet>,
    pending_kf_ts: Option<CuDuration>,
) -> CuResult<()> {
    // Advance the mock clock to the recorded timestamp so any runtime bookkeeping stays aligned.
    let msgs = &copper_list.msgs;
    if let Some(ts) = pending_kf_ts {
        robot_clock.set_value(ts.as_nanos());
    } else {
        let process_time = msgs
            .get_balpos_output()
            .metadata
            .process_time
            .start
            .unwrap()
            .as_nanos();
        robot_clock.set_value(process_time);
    }
    let clock_for_callbacks = robot_clock.clone();

    let mut sim_callback = move |step: default::SimStep| -> SimOverride {
        match step {
            default::SimStep::Balpos(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_balpos_output().clone();
                if let Some(CuDuration(ts)) =
                    Option::<CuTime>::from(msgs.get_balpos_output().metadata.process_time.start)
                {
                    clock_for_callbacks.set_value(ts);
                }
                SimOverride::ExecutedBySim
            }
            default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
            default::SimStep::BalposPid(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_balpos_pid_output().clone();
                if let Some(CuDuration(ts)) =
                    Option::<CuTime>::from(msgs.get_balpos_pid_output().metadata.process_time.start)
                {
                    clock_for_callbacks.set_value(ts);
                }
                SimOverride::ExecutedBySim
            }
            default::SimStep::BalposPid(_) => SimOverride::ExecutedBySim,
            default::SimStep::Railpos(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_railpos_output().clone();
                if let Some(CuDuration(ts)) =
                    Option::<CuTime>::from(msgs.get_railpos_output().metadata.process_time.start)
                {
                    clock_for_callbacks.set_value(ts);
                }
                SimOverride::ExecutedBySim
            }
            default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
            default::SimStep::RailposPid(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_railpos_pid_output().clone();
                if let Some(CuDuration(ts)) = Option::<CuTime>::from(
                    msgs.get_railpos_pid_output().metadata.process_time.start,
                ) {
                    clock_for_callbacks.set_value(ts);
                }
                SimOverride::ExecutedBySim
            }
            default::SimStep::RailposPid(_) => SimOverride::ExecutedBySim,
            default::SimStep::MergePids(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_merge_pids_output().clone();
                if let Some(CuDuration(ts)) =
                    Option::<CuTime>::from(msgs.get_merge_pids_output().metadata.process_time.start)
                {
                    clock_for_callbacks.set_value(ts);
                }
                SimOverride::ExecutedBySim
            }
            default::SimStep::MergePids(_) => SimOverride::ExecutedBySim,
            default::SimStep::Motor(CuTaskCallbackState::Process(input, output)) => {
                // Replay the recorded motor output verbatim to keep logs bit-identical.
                let _ = input; // input unused; we rely on recorded output
                *output = msgs.get_motor_output().clone();
                if let Some(CuDuration(ts)) =
                    Option::<CuTime>::from(msgs.get_motor_output().metadata.process_time.start)
                {
                    clock_for_callbacks.set_value(ts);
                }
                SimOverride::ExecutedBySim
            }
            default::SimStep::Motor(_) => SimOverride::ExecutedBySim,
            _ => SimOverride::ExecuteByRuntime,
        }
    };
    copper_app.run_one_iteration(&mut sim_callback)?;
    Ok(())
}

fn build_callback<'a>(
    copper_list: &'a ReplayCopperList,
    _clock_for_callbacks: RobotClock,
) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a> {
    let msgs = &copper_list.msgs;
    Box::new(move |step: default::SimStep<'_>| match step {
        default::SimStep::Balpos(CuTaskCallbackState::Process(_, output)) => {
            *output = msgs.get_balpos_output().clone();
            SimOverride::ExecutedBySim
        }
        default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::BalposPid(CuTaskCallbackState::Process(_, output)) => {
            *output = msgs.get_balpos_pid_output().clone();
            SimOverride::ExecutedBySim
        }
        default::SimStep::BalposPid(_) => SimOverride::ExecutedBySim,
        default::SimStep::Railpos(CuTaskCallbackState::Process(_, output)) => {
            *output = msgs.get_railpos_output().clone();
            SimOverride::ExecutedBySim
        }
        default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::RailposPid(CuTaskCallbackState::Process(_, output)) => {
            *output = msgs.get_railpos_pid_output().clone();
            SimOverride::ExecutedBySim
        }
        default::SimStep::RailposPid(_) => SimOverride::ExecutedBySim,
        default::SimStep::MergePids(CuTaskCallbackState::Process(_, output)) => {
            *output = msgs.get_merge_pids_output().clone();
            SimOverride::ExecutedBySim
        }
        default::SimStep::MergePids(_) => SimOverride::ExecutedBySim,
        default::SimStep::Motor(CuTaskCallbackState::Process(input, output)) => {
            let _ = input;
            *output = msgs.get_motor_output().clone();
            SimOverride::ExecutedBySim
        }
        default::SimStep::Motor(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    })
}

fn extract_time(copperlist: &ReplayCopperList) -> Option<CuTime> {
    cu29::simulation::recorded_copperlist_timestamp(copperlist)
}

fn make_app(log_base: &Path) -> CuResult<(BalanceBotReSim, RobotClock, RobotClockMock)> {
    let (robot_clock, robot_clock_mock) = RobotClock::mock();
    let copper_app = BalanceBotReSim::builder()
        .with_clock(robot_clock.clone())
        .with_log_path(log_base, REPLAY_LOG_SLAB_SIZE)?
        .with_sim_callback(&mut default_callback)
        .build()?;
    Ok((copper_app, robot_clock, robot_clock_mock))
}

fn run_one_shot(log_base: &Path, replay_log_base: &Path) -> CuResult<()> {
    remove_log_family(replay_log_base)?;
    let (mut copper_app, _robot_clock, mut robot_clock_mock) = make_app(replay_log_base)?;

    copper_app.start_all_tasks(&mut default_callback)?;

    // Restore tasks from the first keyframe so sim starts from the recorded state.
    let UnifiedLogger::Read(dl_kf) = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .map_err(|err| CuError::new_with_cause("failed to open log for keyframes", err))?
    else {
        return Err(CuError::from("expected read logger for keyframes"));
    };
    let mut keyframes_ioreader = UnifiedLoggerIOReader::new(dl_kf, UnifiedLogType::FrozenTasks);
    let mut kf_iter = keyframes_reader(&mut keyframes_ioreader).peekable();

    if let Some(first_kf) = kf_iter.peek() {
        copper_app.copper_runtime_mut().lock_keyframe(first_kf);
        copper_app
            .copper_runtime_mut()
            .set_forced_keyframe_timestamp(first_kf.timestamp);
        let ts = first_kf.timestamp;
        robot_clock_mock.set_value(ts.as_nanos());
    }

    // Read back the logs from a previous run, applying keyframes exactly at their culistid.
    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .map_err(|err| CuError::new_with_cause("failed to open copperlist log", err))?
    else {
        return Err(CuError::from("expected read logger for copperlists"));
    };
    let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
    let iter = copperlists_reader::<default::CuStampedDataSet>(&mut reader).peekable();

    for entry in iter {
        let pending_kf_ts = if let Some(kf) = kf_iter.peek() {
            if kf.culistid == entry.id {
                let ts = kf.timestamp;
                copper_app
                    .copper_runtime_mut()
                    .set_forced_keyframe_timestamp(ts);
                copper_app.copper_runtime_mut().lock_keyframe(kf);
                kf_iter.next();
                Some(ts)
            } else {
                None
            }
        } else {
            None
        };

        if let Err(err) =
            run_one_copperlist(&mut copper_app, &mut robot_clock_mock, entry, pending_kf_ts)
        {
            error!("Simulation replay stopped: {err}");
            eprintln!("Simulation replay stopped: {err}");
            break;
        }
    }

    copper_app.stop_all_tasks(&mut default_callback)?;
    let _ = copper_app.log_shutdown_completed();
    Ok(())
}

fn app_factory(
    params: &SessionOpenParams,
    replay_template: &Path,
) -> CuResult<(BalanceBotReSim, RobotClock, RobotClockMock)> {
    let replay_log_base = per_session_replay_log_base(
        replay_template,
        [params.role.as_deref().unwrap_or("session")],
    );
    remove_log_family(&replay_log_base)?;
    make_app(&replay_log_base)
}

fn run_remote_debug_server(
    debug_base: &str,
    log_base: &Path,
    replay_log_base: &Path,
) -> CuResult<()> {
    let replay_template = replay_log_base.to_path_buf();
    serve_remote_debug::<
        BalanceBotReSim,
        default::CuStampedDataSet,
        ReplayBuildCallback,
        ReplayTimeExtractor,
        MmapSectionStorage,
        MmapUnifiedLoggerWrite,
        _,
    >(
        debug_base,
        log_base,
        move |params| app_factory(params, &replay_template),
        build_callback,
        extract_time,
    )
}

fn main() -> Result<(), Box<dyn Error>> {
    let cli = ReplayCli::parse(ReplayDefaults::new(
        DEFAULT_LOG_BASE,
        DEFAULT_REPLAY_LOG_BASE,
    ));

    ensure_log_family_exists(&cli.log_base)?;
    if let Some(debug_base) = cli.debug_base.as_deref() {
        run_remote_debug_server(debug_base, &cli.log_base, &cli.replay_log_base)?;
    } else {
        run_one_shot(&cli.log_base, &cli.replay_log_base)?;
    }
    Ok(())
}
