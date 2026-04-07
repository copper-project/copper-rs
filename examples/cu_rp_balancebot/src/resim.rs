pub mod tasks;

use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29::remote_debug::SessionOpenParams;
use cu29::replay::{
    ReplayCli, ReplayDefaults, ensure_log_family_exists, per_session_replay_log_base,
    remove_log_family, serve_remote_debug,
};
use cu29::units::si::ratio::ratio;
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
        RobotClockMock,
    ) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a>;
type ReplayTimeExtractor = fn(&ReplayCopperList) -> Option<CuTime>;

fn default_callback(step: default::SimStep) -> SimOverride {
    match step {
        // Replay only injects recorded sensor inputs and emulates the motor sink.
        // Controller tasks must execute live so code changes show up in replay.
        default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::Motor(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn sync_clock_from_recorded<T: CuMsgPayload>(clock: &RobotClockMock, msg: &CuMsg<T>) {
    if let Some(CuDuration(ts)) = Option::<CuTime>::from(msg.metadata.process_time.start) {
        clock.set_value(ts);
    }
}

fn set_process_timing<T: CuMsgPayload>(clock: &RobotClock, msg: &mut CuMsg<T>) {
    let perf = cu29::curuntime::perf_now(clock);
    msg.metadata.process_time.start = perf.into();
    msg.metadata.process_time.end = perf.into();
}

fn balancebot_replay_step<'a>(
    step: default::SimStep<'a>,
    copper_list: &ReplayCopperList,
    process_clock: &RobotClock,
    clock_for_callbacks: &RobotClockMock,
) -> SimOverride {
    let msgs = &copper_list.msgs;
    match step {
        default::SimStep::Balpos(CuTaskCallbackState::Process(_, output)) => {
            *output = msgs.get_balpos_output().clone();
            sync_clock_from_recorded(clock_for_callbacks, msgs.get_balpos_output());
            SimOverride::ExecutedBySim
        }
        default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::BalposPid(CuTaskCallbackState::Process(_, _)) => {
            sync_clock_from_recorded(clock_for_callbacks, msgs.get_balpos_pid_output());
            SimOverride::ExecuteByRuntime
        }
        default::SimStep::Railpos(CuTaskCallbackState::Process(_, output)) => {
            *output = msgs.get_railpos_output().clone();
            sync_clock_from_recorded(clock_for_callbacks, msgs.get_railpos_output());
            SimOverride::ExecutedBySim
        }
        default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::RailposPid(CuTaskCallbackState::Process(_, _)) => {
            sync_clock_from_recorded(clock_for_callbacks, msgs.get_railpos_pid_output());
            SimOverride::ExecuteByRuntime
        }
        default::SimStep::MergePids(CuTaskCallbackState::Process(_, _)) => {
            sync_clock_from_recorded(clock_for_callbacks, msgs.get_merge_pids_output());
            SimOverride::ExecuteByRuntime
        }
        default::SimStep::Motor(CuTaskCallbackState::Process(input, output)) => {
            if let Some(motor_actuation) = input.payload() {
                output.metadata.set_status(format!(
                    "Replay power:{:.4}",
                    motor_actuation.power.get::<ratio>()
                ));
            } else {
                output.metadata.set_status("Safety Mode.");
            }
            set_process_timing(process_clock, output);
            SimOverride::ExecutedBySim
        }
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
    let process_clock = copper_app.clock();

    let mut sim_callback = move |step: default::SimStep| -> SimOverride {
        balancebot_replay_step(step, &copper_list, &process_clock, &clock_for_callbacks)
    };
    copper_app.run_one_iteration(&mut sim_callback)?;
    Ok(())
}

fn build_callback<'a>(
    copper_list: &'a ReplayCopperList,
    process_clock: RobotClock,
    clock_for_callbacks: RobotClockMock,
) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a> {
    Box::new(move |step: default::SimStep<'_>| {
        balancebot_replay_step(step, copper_list, &process_clock, &clock_for_callbacks)
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

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::bincode::de::{DecoderImpl, read::SliceReader};
    use cu29::bincode::{config::standard, decode_from_slice, encode_to_vec};
    use cu29::cutask::BincodeAdapter;
    use cu29::debug::CuDebugSession;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    type ReplayDebugSession = CuDebugSession<
        BalanceBotReSim,
        default::CuStampedDataSet,
        ReplayBuildCallback,
        ReplayTimeExtractor,
        MmapSectionStorage,
        MmapUnifiedLoggerWrite,
    >;

    fn recorded_log_base() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(DEFAULT_LOG_BASE)
    }

    fn temp_replay_log_base(label: &str) -> PathBuf {
        let unique = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0_u128, |duration| duration.as_nanos());
        std::env::temp_dir().join(format!(
            "cu_rp_balancebot_{label}_{}_{}.copper",
            std::process::id(),
            unique
        ))
    }

    fn load_recorded_copperlists(log_base: &Path) -> CuResult<Vec<ReplayCopperList>> {
        let UnifiedLogger::Read(read_logger) = UnifiedLoggerBuilder::new()
            .file_base_name(log_base)
            .build()
            .map_err(|err| CuError::new_with_cause("failed opening balancebot log", err))?
        else {
            return Err(CuError::from("expected read logger for balancebot log"));
        };
        let mut reader = UnifiedLoggerIOReader::new(read_logger, UnifiedLogType::CopperList);
        Ok(copperlists_reader::<default::CuStampedDataSet>(&mut reader).collect())
    }

    fn load_recorded_keyframes(log_base: &Path) -> CuResult<Vec<KeyFrame>> {
        let UnifiedLogger::Read(read_logger) = UnifiedLoggerBuilder::new()
            .file_base_name(log_base)
            .build()
            .map_err(|err| {
                CuError::new_with_cause("failed opening balancebot keyframe log", err)
            })?
        else {
            return Err(CuError::from(
                "expected read logger for balancebot keyframe log",
            ));
        };
        let mut reader = UnifiedLoggerIOReader::new(read_logger, UnifiedLogType::FrozenTasks);
        Ok(keyframes_reader(&mut reader).collect())
    }

    fn open_debug_session(log_base: &Path, replay_log_base: &Path) -> CuResult<ReplayDebugSession> {
        remove_log_family(replay_log_base)?;
        let (app, robot_clock, robot_clock_mock) = make_app(replay_log_base)?;
        CuDebugSession::from_log(
            log_base,
            app,
            robot_clock,
            robot_clock_mock,
            build_callback,
            extract_time,
        )
    }

    fn runtime_copperlist(session: &mut ReplayDebugSession) -> CuResult<ReplayCopperList> {
        let runtime_bytes = session.with_app(|app| {
            app.current_runtime_copperlist_bytes()
                .map(ToOwned::to_owned)
        });
        let Some(runtime_bytes) = runtime_bytes else {
            return Err(CuError::from("runtime copperlist bytes unavailable"));
        };
        decode_from_slice::<ReplayCopperList, _>(&runtime_bytes, standard())
            .map(|(copperlist, _)| copperlist)
            .map_err(|err| CuError::new_with_cause("failed decoding runtime copperlist bytes", err))
    }

    fn encoded<T: cu29::bincode::Encode>(value: &T, label: &str) -> Vec<u8> {
        encode_to_vec(value, standard()).unwrap_or_else(|err| panic!("encode {label}: {err}"))
    }

    fn assert_runtime_matches_recorded_exact(
        session: &mut ReplayDebugSession,
        expected: &ReplayCopperList,
        label: &str,
    ) -> CuResult<()> {
        let runtime = runtime_copperlist(session)?;
        assert_eq!(
            encoded(&runtime, "runtime copperlist"),
            encoded(expected, "expected copperlist"),
            "{label}: replay diverged at CL{}",
            expected.id
        );
        Ok(())
    }

    fn assert_runtime_matches_recorded_payloads(
        session: &mut ReplayDebugSession,
        expected: &ReplayCopperList,
        label: &str,
    ) -> CuResult<()> {
        let runtime = runtime_copperlist(session)?;
        let runtime_msgs = &runtime.msgs;
        let expected_msgs = &expected.msgs;

        assert_eq!(
            runtime_msgs
                .get_balpos_output()
                .payload()
                .map(|payload| encoded(payload, "runtime balpos payload")),
            expected_msgs
                .get_balpos_output()
                .payload()
                .map(|payload| encoded(payload, "expected balpos payload")),
            "{label}: balpos payload diverged at CL{}",
            expected.id
        );
        assert_eq!(
            runtime_msgs
                .get_railpos_output()
                .payload()
                .map(|payload| encoded(payload, "runtime railpos payload")),
            expected_msgs
                .get_railpos_output()
                .payload()
                .map(|payload| encoded(payload, "expected railpos payload")),
            "{label}: railpos payload diverged at CL{}",
            expected.id
        );
        assert_eq!(
            runtime_msgs
                .get_balpos_pid_output()
                .payload()
                .map(|payload| encoded(payload, "runtime balpos_pid payload")),
            expected_msgs
                .get_balpos_pid_output()
                .payload()
                .map(|payload| encoded(payload, "expected balpos_pid payload")),
            "{label}: balpos_pid payload diverged at CL{}",
            expected.id
        );
        assert_eq!(
            runtime_msgs
                .get_railpos_pid_output()
                .payload()
                .map(|payload| encoded(payload, "runtime railpos_pid payload")),
            expected_msgs
                .get_railpos_pid_output()
                .payload()
                .map(|payload| encoded(payload, "expected railpos_pid payload")),
            "{label}: railpos_pid payload diverged at CL{}",
            expected.id
        );
        assert_eq!(
            runtime_msgs
                .get_merge_pids_output()
                .payload()
                .map(|payload| encoded(payload, "runtime merge_pids payload")),
            expected_msgs
                .get_merge_pids_output()
                .payload()
                .map(|payload| encoded(payload, "expected merge_pids payload")),
            "{label}: merge_pids payload diverged at CL{}",
            expected.id
        );
        Ok(())
    }

    fn materialize_fresh_replay_log(
        source_log_base: &Path,
        replay_log_base: &Path,
    ) -> CuResult<()> {
        remove_log_family(replay_log_base)?;
        let recorded = load_recorded_copperlists(source_log_base)?;
        let (mut copper_app, _robot_clock, mut robot_clock_mock) = make_app(replay_log_base)?;
        copper_app.start_all_tasks(&mut default_callback)?;

        for entry in recorded {
            run_one_copperlist(&mut copper_app, &mut robot_clock_mock, entry, None)?;
        }

        copper_app.stop_all_tasks(&mut default_callback)?;
        let _ = copper_app.log_shutdown_completed();
        Ok(())
    }

    #[test]
    fn fresh_balancebot_keyframe_thaws_in_task_order() -> CuResult<()> {
        let source_log_base = recorded_log_base();
        let fresh_log_base = temp_replay_log_base("fresh_keyframe_decode");
        let debug_log_base = temp_replay_log_base("fresh_keyframe_decode_runtime");
        materialize_fresh_replay_log(&source_log_base, &fresh_log_base)?;

        let keyframes = load_recorded_keyframes(&fresh_log_base)?;
        let first_keyframe = keyframes
            .first()
            .ok_or_else(|| CuError::from("fresh replay log did not contain keyframes"))?;
        let keyframe_100 = keyframes
            .iter()
            .find(|keyframe| keyframe.culistid == 100)
            .ok_or_else(|| CuError::from("fresh replay log did not contain the CL100 keyframe"))?;

        let (mut app, _robot_clock, _robot_clock_mock) = make_app(&debug_log_base)?;
        app.start_all_tasks(&mut default_callback)?;

        let reader = SliceReader::new(&first_keyframe.serialized_tasks);
        let mut decoder = DecoderImpl::new(reader, standard(), ());
        let tasks = &mut app.copper_runtime_mut().tasks;
        tasks
            .0
            .thaw(&mut decoder)
            .map_err(|err| CuError::new_with_cause("task 0 thaw failed", err))?;
        tasks
            .1
            .thaw(&mut decoder)
            .map_err(|err| CuError::new_with_cause("task 1 thaw failed", err))?;
        tasks
            .2
            .thaw(&mut decoder)
            .map_err(|err| CuError::new_with_cause("task 2 thaw failed", err))?;
        tasks
            .3
            .thaw(&mut decoder)
            .map_err(|err| CuError::new_with_cause("task 3 thaw failed", err))?;
        tasks
            .4
            .thaw(&mut decoder)
            .map_err(|err| CuError::new_with_cause("task 4 thaw failed", err))?;
        tasks
            .5
            .thaw(&mut decoder)
            .map_err(|err| CuError::new_with_cause("task 5 thaw failed", err))?;

        let (mut app_100, _robot_clock_100, _robot_clock_mock_100) =
            make_app(&temp_replay_log_base("fresh_keyframe_decode_runtime_100"))?;
        app_100.start_all_tasks(&mut default_callback)?;

        let (mut app_runtime_100, _robot_clock_runtime_100, mut robot_clock_mock_runtime_100) =
            make_app(&temp_replay_log_base("fresh_keyframe_runtime_state_100"))?;
        app_runtime_100.start_all_tasks(&mut default_callback)?;
        for copperlist in load_recorded_copperlists(&fresh_log_base)? {
            if copperlist.id == 100 {
                break;
            }
            run_one_copperlist(
                &mut app_runtime_100,
                &mut robot_clock_mock_runtime_100,
                copperlist,
                None,
            )?;
        }
        let runtime_tasks = &app_runtime_100.copper_runtime_mut().tasks;
        let task_0_bytes = encode_to_vec(BincodeAdapter(&runtime_tasks.0), standard())
            .expect("encode task 0 runtime state for CL100");
        let task_1_bytes = encode_to_vec(BincodeAdapter(&runtime_tasks.1), standard())
            .expect("encode task 1 runtime state for CL100");
        let task_2_bytes = encode_to_vec(BincodeAdapter(&runtime_tasks.2), standard())
            .expect("encode task 2 runtime state for CL100");
        let expected_prefix = [
            task_0_bytes.as_slice(),
            task_1_bytes.as_slice(),
            task_2_bytes.as_slice(),
        ]
        .concat();
        assert!(
            keyframe_100.serialized_tasks.len() >= expected_prefix.len(),
            "CL100 keyframe shorter than encoded task prefix: {} < {}",
            keyframe_100.serialized_tasks.len(),
            expected_prefix.len()
        );
        assert_eq!(
            &keyframe_100.serialized_tasks[..expected_prefix.len()],
            expected_prefix.as_slice(),
            "CL100 keyframe prefix diverged from live task freeze bytes"
        );

        let reader_100 = SliceReader::new(&keyframe_100.serialized_tasks);
        let mut decoder_100 = DecoderImpl::new(reader_100, standard(), ());
        let tasks_100 = &mut app_100.copper_runtime_mut().tasks;
        tasks_100
            .0
            .thaw(&mut decoder_100)
            .map_err(|err| CuError::new_with_cause("task 0 thaw failed for CL100", err))?;
        tasks_100
            .1
            .thaw(&mut decoder_100)
            .map_err(|err| CuError::new_with_cause("task 1 thaw failed for CL100", err))?;
        tasks_100
            .2
            .thaw(&mut decoder_100)
            .map_err(|err| CuError::new_with_cause("task 2 thaw failed for CL100", err))?;
        tasks_100
            .3
            .thaw(&mut decoder_100)
            .map_err(|err| CuError::new_with_cause("task 3 thaw failed for CL100", err))?;
        tasks_100
            .4
            .thaw(&mut decoder_100)
            .map_err(|err| CuError::new_with_cause("task 4 thaw failed for CL100", err))?;
        tasks_100
            .5
            .thaw(&mut decoder_100)
            .map_err(|err| CuError::new_with_cause("task 5 thaw failed for CL100", err))?;

        for keyframe in &keyframes {
            <BalanceBotReSim as CuSimApplication<MmapSectionStorage, MmapUnifiedLoggerWrite>>::restore_keyframe(
                &mut app,
                keyframe,
            )
            .map_err(|err| {
                CuError::from(format!("restore_keyframe failed for CL{}", keyframe.culistid))
                    .add_cause(&err.to_string())
            })?;
        }

        remove_log_family(&debug_log_base)?;
        remove_log_family(&fresh_log_base)?;
        Ok(())
    }

    #[test]
    fn jump_then_replay_from_here_matches_fresh_balancebot_replay_log() -> CuResult<()> {
        let source_log_base = recorded_log_base();
        assert!(
            source_log_base.exists(),
            "missing recorded balancebot log at {}",
            source_log_base.display()
        );
        let fresh_log_base = temp_replay_log_base("fresh_recorded");
        materialize_fresh_replay_log(&source_log_base, &fresh_log_base)?;

        let recorded = load_recorded_copperlists(&fresh_log_base)?;
        assert!(
            recorded.len() > 16,
            "need enough copperlists to exercise rewind/step determinism, found {}",
            recorded.len()
        );

        let first_replay_end_idx = recorded.len() / 2;
        let rewind_idx = recorded.len() / 4;
        let debug_replay_log_base = temp_replay_log_base("debug_determinism");
        let mut session = open_debug_session(&fresh_log_base, &debug_replay_log_base)?;

        let first_jump = session.goto_cl(recorded[first_replay_end_idx].id)?;
        assert_eq!(
            first_jump.culistid, recorded[first_replay_end_idx].id,
            "first jump landed on the wrong copperlist"
        );
        assert_runtime_matches_recorded_exact(
            &mut session,
            &recorded[first_replay_end_idx],
            "initial partial replay",
        )?;

        let rewind_jump = session.goto_cl(recorded[rewind_idx].id)?;
        assert_eq!(
            rewind_jump.culistid, recorded[rewind_idx].id,
            "rewind jump landed on the wrong copperlist"
        );
        assert_runtime_matches_recorded_exact(
            &mut session,
            &recorded[rewind_idx],
            "seek back to replay cursor",
        )?;

        let repeat_cursor_jump = session.goto_cl(recorded[rewind_idx].id)?;
        assert_eq!(
            repeat_cursor_jump.replayed, 0,
            "repeat seek to the replay cursor should not advance runtime state"
        );
        assert_runtime_matches_recorded_exact(
            &mut session,
            &recorded[rewind_idx],
            "repeat seek to replay cursor",
        )?;

        for (idx, expected) in recorded.iter().enumerate().skip(rewind_idx + 1) {
            let step = session.step(1)?;
            assert_eq!(
                step.culistid, expected.id,
                "step forward landed on wrong copperlist at idx {idx}"
            );
            assert_runtime_matches_recorded_exact(
                &mut session,
                expected,
                format!("step forward after rewind idx={idx} cl={}", expected.id).as_str(),
            )?;
        }

        remove_log_family(&debug_replay_log_base)?;
        remove_log_family(&fresh_log_base)?;
        Ok(())
    }

    #[test]
    fn jump_then_replay_from_here_matches_regenerated_balancebot_log_payloads() -> CuResult<()> {
        let log_base = recorded_log_base();
        assert!(
            log_base.exists(),
            "missing recorded balancebot log at {}",
            log_base.display()
        );

        let recorded = load_recorded_copperlists(&log_base)?;
        assert!(
            recorded.len() > 16,
            "need enough copperlists to exercise rewind/step determinism, found {}",
            recorded.len()
        );

        let first_replay_end_idx = recorded.len() / 2;
        let rewind_idx = recorded.len() / 4;
        let replay_log_base = temp_replay_log_base("debug_regenerated_determinism");
        let mut session = open_debug_session(&log_base, &replay_log_base)?;

        let first_jump = session.goto_cl(recorded[first_replay_end_idx].id)?;
        assert_eq!(
            first_jump.culistid, recorded[first_replay_end_idx].id,
            "first jump landed on the wrong copperlist"
        );
        assert_runtime_matches_recorded_payloads(
            &mut session,
            &recorded[first_replay_end_idx],
            "initial partial replay",
        )?;

        let rewind_jump = session.goto_cl(recorded[rewind_idx].id)?;
        assert_eq!(
            rewind_jump.culistid, recorded[rewind_idx].id,
            "rewind jump landed on the wrong copperlist"
        );
        assert_runtime_matches_recorded_payloads(
            &mut session,
            &recorded[rewind_idx],
            "seek back to replay cursor",
        )?;

        let repeat_cursor_jump = session.goto_cl(recorded[rewind_idx].id)?;
        assert_eq!(
            repeat_cursor_jump.replayed, 0,
            "repeat seek to the replay cursor should not advance runtime state"
        );
        assert_runtime_matches_recorded_payloads(
            &mut session,
            &recorded[rewind_idx],
            "repeat seek to replay cursor",
        )?;

        for (idx, expected) in recorded.iter().enumerate().skip(rewind_idx + 1) {
            let step = session.step(1)?;
            assert_eq!(
                step.culistid, expected.id,
                "step forward landed on wrong copperlist at idx {idx}"
            );
            assert_runtime_matches_recorded_payloads(
                &mut session,
                expected,
                format!("step forward after rewind idx={idx} cl={}", expected.id).as_str(),
            )?;
        }

        remove_log_family(&replay_log_base)?;
        Ok(())
    }
}
