#![cfg(all(feature = "std", feature = "telemetry", not(feature = "async-cl-io")))]
use bincode::{Decode, Encode};
use cu29::logstream::TelemetryBufferConfig;
use cu29::logstream::protocol::{CuTelemetryRecord, CuTelemetryTx, TxMode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex as StdMutex};
#[derive(Clone, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
struct StreamMsg(u64);

#[derive(Default, Reflect)]
struct StreamSource {
    next: u64,
}

impl Freezable for StreamSource {}

impl CuSrcTask for StreamSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(StreamMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        info!(ctx, "Logstream source iteration {}", self.next);
        output.set_payload(StreamMsg(self.next));
        self.next += 1;
        Ok(())
    }
}

#[copper_runtime(config = "tests/manual_telemetry.ron")]
struct ManualApp {}

mod ground {
    use super::{StreamMsg, StreamSource};
    use cu29::prelude::*;
    #[copper_runtime(config = "tests/manual_telemetry.ron", sim_mode = true)]
    struct Ground {}
    pub type Twin = default::Ground;
}
mod missions {
    use super::{StreamMsg, StreamSource};
    use cu29::prelude::*;
    #[copper_runtime(config = "tests/manual_telemetry_missions.ron")]
    struct Robot {}
    use super::{Arc, CuTelemetryRecord, CuTelemetryTx, TelemetryBufferConfig};
    pub(super) fn check_managed_loop() -> CuResult<()> {
        use std::sync::atomic::{AtomicUsize, Ordering};
        struct ManagedProbe(Arc<AtomicUsize>, u8);
        impl CuTelemetryTx for ManagedProbe {
            fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
                if let CuTelemetryRecord::Session(session) = record {
                    assert_eq!(session.identity.session_id, [self.1; 16]);
                }
                Ok(())
            }
            fn poll(&mut self, mode: super::TxMode) -> CuResult<Option<CuTime>> {
                if mode == super::TxMode::Running {
                    self.0.fetch_add(1, Ordering::Relaxed);
                }
                Ok(None)
            }
        }
        let received = Arc::new(AtomicUsize::new(0));
        let mut activation = 0;
        let mut runner = RobotRunner::builder()
            .with_telemetry_tx(
                || {
                    activation += 1;
                    Ok(([activation; 16], ManagedProbe(received.clone(), activation)))
                },
                TelemetryBufferConfig::new(1024, 1024),
            )
            .build()?;
        runner.start(RobotMission::First)?;
        let mut iterations = 0;
        runner.run_until(|| {
            iterations += 1;
            iterations > 6
        })?;
        assert_eq!(received.load(Ordering::Relaxed), 6);
        runner.transition_to(RobotMission::Second)?;
        let before = received.load(Ordering::Relaxed);
        let mut iterations = 0;
        runner.run_until(|| {
            iterations += 1;
            iterations > 3
        })?;
        assert_eq!(received.load(Ordering::Relaxed) - before, 3);
        runner.stop()?;
        Ok(())
    }
}

#[derive(Default)]
struct Observations {
    captures: Vec<Vec<u8>>,
    keyframes: Vec<Vec<u8>>,
    lists: Vec<u64>,
    frames: Vec<u64>,
    logs: Vec<u64>,
    sessions: usize,
    draining: bool,
}
struct Probe(Arc<StdMutex<Observations>>);
impl CuTelemetryTx for Probe {
    fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
        let mut seen = self.0.lock().unwrap();
        match record {
            CuTelemetryRecord::Session(session) => {
                assert_eq!(session.identity.session_id, [9; 16]);
                seen.sessions += 1;
            }
            CuTelemetryRecord::CopperList { id, capture } => {
                assert_eq!(seen.sessions, 1);
                let (decoded, used): (CopperList<default::CuStampedDataSet>, _) =
                    bincode::decode_from_slice(capture, bincode::config::standard()).unwrap();
                assert_eq!(used, capture.len());
                assert_eq!(decoded.id, id);
                assert_eq!(
                    decoded.msgs.get_source_output().payload(),
                    Some(&StreamMsg(id))
                );
                seen.captures.push(capture.to_vec());
                seen.lists.push(id);
            }
            CuTelemetryRecord::KeyFrame {
                copperlist_id,
                state,
            } => {
                let (frame, used): (KeyFrame, _) =
                    bincode::decode_from_slice(state, bincode::config::standard()).unwrap();
                assert_eq!(frame.culistid, copperlist_id);
                assert_eq!(used, state.len());
                seen.keyframes.push(state.to_vec());
                seen.frames.push(copperlist_id);
            }
            CuTelemetryRecord::StructuredLog { sequence, entry } => {
                assert!(!entry.is_empty());
                seen.logs.push(sequence);
            }
        }
        Ok(())
    }
    fn poll(&mut self, mode: TxMode) -> CuResult<Option<CuTime>> {
        self.0.lock().unwrap().draining |= mode == TxMode::Draining;
        Ok(None)
    }
}
#[test]
fn generated_iteration_services_telemetry_and_twin_recovers_from_loss() -> CuResult<()> {
    let seen = Arc::new(StdMutex::new(Observations::default()));
    let tx = Probe(seen.clone());
    let app = ManualApp::builder()
        .with_telemetry_tx(
            [9; 16],
            move |_| Ok(tx),
            TelemetryBufferConfig {
                capture_count: 1,
                keyframe_count: 1,
                structured_log_count: 1,
                ..TelemetryBufferConfig::new(1024, 1024)
            },
        )
        .build()?;
    let mut app = app.start()?;
    for id in 0..6 {
        app.run_one_iteration()?;
        let s = seen.lock().unwrap();
        assert_eq!(s.sessions, 1);
        assert_eq!(s.lists.last(), Some(&id));
    }
    drop(app.stop()?);
    let s = seen.lock().unwrap();
    assert_eq!(s.sessions, 1);
    assert_eq!(s.lists, [0, 1, 2, 3, 4, 5]);
    assert_eq!(s.frames.first(), Some(&0));
    assert!(!s.logs.is_empty());
    assert!(s.draining);
    use cu29::logstream::twin::LiveReplay;
    let (mut replay, clock) = ground::Twin::build_twin()?;
    assert_eq!(s.frames, [0, 5]);
    for (bytes, frame) in [&s.captures[0], &s.captures[5]]
        .into_iter()
        .zip(&s.keyframes)
    {
        let (mut list, _) = bincode::decode_from_slice(bytes, bincode::config::standard()).unwrap();
        let (frame, _) = bincode::decode_from_slice(frame, bincode::config::standard()).unwrap();
        replay.replay_capture(&clock, &mut list, Some(&frame))?;
        assert_eq!(
            list.msgs.get_source_output().payload(),
            Some(&StreamMsg(list.id))
        );
    }
    replay.stop_twin()?;
    drop(replay);
    drop(s);
    missions::check_managed_loop()?;

    Ok(())
}

#[cfg(feature = "memory_monitoring")]
#[test]
fn manual_handoff_and_service_do_not_allocate_after_setup() -> CuResult<()> {
    use cu29::curuntime::{KeyFramesManager, SyncCopperListsManager};
    use cu29::logstream::{manual::ManualDestination, structured::StructuredLogOutput};
    use cu29::monitoring::ScopedAllocCounter;
    struct Discard;
    impl CuTelemetryTx for Discard {
        fn send(&mut self, record: CuTelemetryRecord<'_>) -> CuResult<()> {
            core::hint::black_box(record);
            Ok(())
        }
    }
    let counter = ScopedAllocCounter::new();
    core::hint::black_box(vec![0u8; 128]);
    assert!(counter.allocated() >= 128);

    let buffers = TelemetryBufferConfig {
        capture_count: 1,
        keyframe_count: 1,
        structured_log_count: 1,
        ..TelemetryBufferConfig::new(1024, 1024)
    };
    let (clock, _) = RobotClock::mock();
    let (mut tx, mut logs) = ManualDestination {
        session_id: [1; 16],
        buffers,
        make_tx: Box::new(|_| Ok(Box::new(Discard))),
    }
    .start::<default::CuStampedDataSet>(clock.clone(), Default::default())?;
    let mut lists = SyncCopperListsManager::<default::CuStampedDataSet, 2>::new(None)?;
    lists.enable_manual_output(1, 1)?;
    let mut frames = KeyFramesManager::new(Some(Box::new(cu29::curuntime::NullWriteStream)), 1)?;
    frames.finish_capture_preallocation()?;
    lists.prepare_manual_keyframes(&frames)?;
    let counter = ScopedAllocCounter::new();
    for id in 0..16 {
        let list = lists.create()?;
        list.change_state(cu29::copperlist::CopperListState::Processing);
        frames.try_reset(id, &clock)?;
        logs.begin();
        logs.write(&[1, 2, 3, 4]);
        logs.finish(true);
        lists.end_of_processing_with_keyframes(id, &mut frames)?;
        if id % 4 == 3 {
            for _ in 0..5 {
                tx.service(&mut lists)?;
            }
        }
    }
    assert_eq!(counter.allocated(), 0);
    assert_eq!(counter.deallocated(), 0);
    assert!(lists.dropped_copperlists_total() > 0);
    assert!(tx.dropped_logs() > 0);
    assert!(lists.dropped_keyframes_total() > 0);
    Ok(())
}
