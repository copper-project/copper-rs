mod motor_model;
pub mod tasks;
use cu29::prelude::app::CuSimApplication;
use cu29::prelude::*;
use cu29_export::copperlists_reader;
use cu29_export::keyframes_reader;
use cu29_helpers::basic_copper_setup;
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use std::path::{Path, PathBuf};

// To enable resim, it is just your regular macro with sim_mode true
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct BalanceBotReSim {}

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

fn main() {
    // Create the Copper App in simulation mode.
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);
    let logger_path = "logs/balanceresim.copper";
    let (robot_clock, mut robot_clock_mock) = RobotClock::mock();
    let copper_ctx = basic_copper_setup(
        &PathBuf::from(logger_path),
        LOG_SLAB_SIZE,
        true,
        Some(robot_clock.clone()),
    )
    .expect("Failed to setup logger.");

    let mut copper_app = BalanceBotReSimBuilder::new()
        .with_context(&copper_ctx)
        .with_sim_callback(&mut default_callback)
        .build()
        .expect("Failed to create runtime.");

    copper_app
        .start_all_tasks(&mut default_callback)
        .expect("Failed to start all tasks.");

    // Restore tasks from the first keyframe so sim starts from the recorded state.
    let UnifiedLogger::Read(dl_kf) = UnifiedLoggerBuilder::new()
        .file_base_name(Path::new("logs/balance.copper"))
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger");
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
        .file_base_name(Path::new("logs/balance.copper"))
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger");
    };
    let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
    let mut iter = copperlists_reader::<default::CuStampedDataSet>(&mut reader).peekable();

    while let Some(entry) = iter.next() {
        // Apply keyframe that matches this CL id, if any.
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
    copper_app
        .stop_all_tasks(&mut default_callback)
        .expect("Failed to start all tasks.");
}
