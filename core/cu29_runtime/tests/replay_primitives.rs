// Replay-primitive tests: drive the raw (app-deprecated) lifecycle API on
// purpose, because replay interleaves recorded-copperlist injection with a
// started app in ways the lifecycle typestate deliberately does not expose.
#![allow(deprecated)]
#![cfg(all(test, feature = "std"))]

use bincode::{Decode, Encode, config::standard, encode_to_vec};
use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::prelude::copper_runtime;
use cu29::prelude::*;
use cu29::simulation::recorded_copperlist_timestamp;
use cu29_export::{copperlists_reader, keyframes_reader};
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, LazyLock, Mutex};

/// The logger runtime is a process-wide singleton: serialize the tests that
/// each build a full application so they do not race its initialization.
static TEST_MUTEX: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));

#[derive(Default, Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
struct CounterMsg {
    value: u32,
}

#[derive(Default, Debug, Clone, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect)]
struct AccumMsg {
    sum: u32,
}

static CUSTOM_CODEC_ENCODE_CALLS: AtomicUsize = AtomicUsize::new(0);

struct CountingCodec;

impl cu29::logcodec::CuLogCodec<CounterMsg> for CountingCodec {
    type Config = ();

    fn new(_config: Self::Config) -> CuResult<Self> {
        Ok(Self)
    }

    fn source_payload_handle_bytes(&self, _payload: &CounterMsg) -> usize {
        0
    }

    fn encode_payload<E: Encoder>(
        &mut self,
        payload: &CounterMsg,
        encoder: &mut E,
    ) -> Result<(), EncodeError> {
        CUSTOM_CODEC_ENCODE_CALLS.fetch_add(1, Ordering::Relaxed);
        payload.encode(encoder)
    }

    fn decode_payload<D: Decoder<Context = ()>>(
        &mut self,
        decoder: &mut D,
    ) -> Result<CounterMsg, DecodeError> {
        CounterMsg::decode(decoder)
    }
}

#[derive(Reflect)]
struct CounterSrc {
    next: u32,
}

impl Freezable for CounterSrc {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.next, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.next = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSrcTask for CounterSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(CounterMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { next: 0 })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.next += 1;
        output.set_payload(CounterMsg { value: self.next });
        Ok(())
    }
}

#[derive(Reflect)]
struct Accumulator {
    sum: u32,
}

impl Freezable for Accumulator {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.sum, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.sum = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuTask for Accumulator {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CounterMsg);
    type Output<'m> = output_msg!(AccumMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { sum: 0 })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(msg) = input.payload() {
            self.sum += msg.value;
            output.set_payload(AccumMsg { sum: self.sum });
        } else {
            output.clear_payload();
        }
        Ok(())
    }
}

#[derive(Reflect)]
struct SpySink {
    last: Option<u32>,
}

impl Freezable for SpySink {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.last, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.last = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSinkTask for SpySink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(AccumMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { last: None })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        self.last = input.payload().map(|payload| payload.sum);
        Ok(())
    }
}

#[copper_runtime(config = "tests/replay_primitives_config.ron", sim_mode = true)]
struct ReplayApp {}

fn build_logger(path: &Path) -> CuResult<Arc<Mutex<MmapUnifiedLoggerWrite>>> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .map_err(|e| cu29::CuError::new_with_cause("create log dir failed", e))?;
    }
    let UnifiedLogger::Write(writer) = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .preallocated_size(16 * 1024 * 1024)
        .file_base_name(path)
        .build()
        .map_err(|e| cu29::CuError::new_with_cause("logger init failed", e))?
    else {
        return Err(cu29::CuError::from("logger builder did not return writer"));
    };
    Ok(Arc::new(Mutex::new(writer)))
}

fn read_first_copperlist<P: CopperListTuple>(path: &Path) -> CuResult<CopperList<P>> {
    let UnifiedLogger::Read(read_logger) = UnifiedLoggerBuilder::new()
        .file_base_name(path)
        .build()
        .map_err(|e| cu29::CuError::new_with_cause("open copperlist log failed", e))?
    else {
        return Err(cu29::CuError::from("logger builder did not return reader"));
    };
    let mut reader = UnifiedLoggerIOReader::new(read_logger, UnifiedLogType::CopperList);
    copperlists_reader::<P>(&mut reader)
        .next()
        .ok_or_else(|| cu29::CuError::from("recorded log did not contain a copperlist"))
}

fn read_first_keyframe(path: &Path) -> CuResult<KeyFrame> {
    let UnifiedLogger::Read(read_logger) = UnifiedLoggerBuilder::new()
        .file_base_name(path)
        .build()
        .map_err(|e| cu29::CuError::new_with_cause("open keyframe log failed", e))?
    else {
        return Err(cu29::CuError::from("logger builder did not return reader"));
    };
    let mut reader = UnifiedLoggerIOReader::new(read_logger, UnifiedLogType::FrozenTasks);
    keyframes_reader(&mut reader)
        .next()
        .ok_or_else(|| cu29::CuError::from("recorded log did not contain a keyframe"))
}

fn encode_bytes<T: Encode>(value: &T) -> Vec<u8> {
    encode_to_vec(value, standard()).expect("encode value for deterministic comparison")
}

fn assert_msg_metadata_eq<T>(expected: &CuMsg<T>, actual: &CuMsg<T>)
where
    T: CuMsgPayload + PartialEq,
{
    assert_eq!(expected.payload(), actual.payload());
    assert_eq!(expected.tov, actual.tov);
    let expected_start: Option<CuTime> = expected.metadata.process_time.start.into();
    let actual_start: Option<CuTime> = actual.metadata.process_time.start.into();
    let expected_end: Option<CuTime> = expected.metadata.process_time.end.into();
    let actual_end: Option<CuTime> = actual.metadata.process_time.end.into();
    assert_eq!(expected_start, actual_start);
    assert_eq!(expected_end, actual_end);
    assert_eq!(expected.metadata.status_txt, actual.metadata.status_txt);
    assert_eq!(expected.metadata.origin, actual.metadata.origin);
}

#[test]
fn generated_copperlist_codec_restores_every_message_field() {
    let _guard = TEST_MUTEX
        .lock()
        .unwrap_or_else(|poison| poison.into_inner());
    let mut expected = default::CuStampedDataSet::default();
    let repeated_origin = CuMsgOrigin {
        subsystem_code: u16::MAX,
        instance_id: u32::MAX,
        cl_id: u64::MAX,
    };

    expected.0.0.set_payload(CounterMsg { value: u32::MAX });
    expected.0.0.tov = Tov::Range(CuTimeRange {
        start: CuTime::MAX,
        end: CuTime::MIN,
    });
    expected.0.0.metadata.process_time = PartialCuTimeRange {
        start: Some(CuTime::MAX).into(),
        end: Some(CuTime::MIN).into(),
    };
    expected.0.0.metadata.set_status("repeated");
    expected.0.0.metadata.origin = Some(repeated_origin.clone());

    expected.0.1.clear_payload();
    expected.0.1.tov = Tov::Time(CuTime(42));
    expected.0.1.metadata.process_time = PartialCuTimeRange {
        start: None.into(),
        end: Some(CuTime(17)).into(),
    };
    expected.0.1.metadata.set_status("repeated");
    expected.0.1.metadata.origin = Some(repeated_origin);

    CUSTOM_CODEC_ENCODE_CALLS.store(0, Ordering::Relaxed);
    let mut encoded = vec![0u8; 4096];
    let encoded_len = bincode::encode_into_slice(&expected, &mut encoded, standard())
        .expect("encode generated CopperList");
    encoded.truncate(encoded_len);
    assert_eq!(CUSTOM_CODEC_ENCODE_CALLS.load(Ordering::Relaxed), 1);
    let (decoded, used): (default::CuStampedDataSet, usize) =
        bincode::decode_from_slice(&encoded, standard()).expect("decode generated CopperList");

    assert_eq!(used, encoded.len());
    assert_msg_metadata_eq(&expected.0.0, &decoded.0.0);
    assert_msg_metadata_eq(&expected.0.1, &decoded.0.1);
}

fn record_reference_run(
    log_path: &Path,
) -> CuResult<(CopperList<default::CuStampedDataSet>, KeyFrame)> {
    let logger = build_logger(log_path)?;
    let (clock, _clock_mock) = RobotClock::mock();
    let mut noop = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;

    let mut app = ReplayApp::builder()
        .with_clock(clock)
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
        .with_sim_callback(&mut noop)
        .build()?
        .into_inner();

    app.start_all_tasks(&mut noop)?;
    app.run_one_iteration(&mut noop)?;
    app.stop_all_tasks(&mut noop)?;

    drop(app);

    Ok((
        read_first_copperlist::<default::CuStampedDataSet>(log_path)?,
        read_first_keyframe(log_path)?,
    ))
}

fn replay_run(
    log_path: &Path,
    recorded_cl: &CopperList<default::CuStampedDataSet>,
    keyframe: Option<&KeyFrame>,
) -> CuResult<(CopperList<default::CuStampedDataSet>, KeyFrame, u64)> {
    let logger = build_logger(log_path)?;
    let (clock, clock_mock) = RobotClock::mock();
    let mut noop = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;

    let mut app = ReplayApp::builder()
        .with_clock(clock)
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
        .with_sim_callback(&mut noop)
        .build()?
        .into_inner();

    app.start_all_tasks(&mut noop)?;
    app.replay_recorded_copperlist(&clock_mock, recorded_cl, keyframe)?;
    app.stop_all_tasks(&mut noop)?;

    let clock_value = clock_mock.value();
    drop(app);

    Ok((
        read_first_copperlist::<default::CuStampedDataSet>(log_path)?,
        read_first_keyframe(log_path)?,
        clock_value,
    ))
}

#[test]
fn replay_recorded_copperlist_reproduces_copperlist_and_keyframe() -> CuResult<()> {
    let _guard = TEST_MUTEX
        .lock()
        .unwrap_or_else(|poison| poison.into_inner());
    let temp_dir = tempfile::tempdir()
        .map_err(|e| cu29::CuError::new_with_cause("create temp dir failed", e))?;
    let record_path = temp_dir.path().join("recorded.copper");
    let replay_path = temp_dir.path().join("replayed.copper");

    let (recorded_cl, recorded_kf) = record_reference_run(&record_path)?;
    let (replayed_cl, replayed_kf, replay_clock) =
        replay_run(&replay_path, &recorded_cl, Some(&recorded_kf))?;

    assert_eq!(encode_bytes(&replayed_cl), encode_bytes(&recorded_cl));
    assert_eq!(encode_bytes(&replayed_kf), encode_bytes(&recorded_kf));
    assert_eq!(replay_clock, recorded_kf.timestamp.as_nanos());

    Ok(())
}

#[test]
fn replay_recorded_copperlist_without_keyframe_uses_recorded_timestamp() -> CuResult<()> {
    let _guard = TEST_MUTEX
        .lock()
        .unwrap_or_else(|poison| poison.into_inner());
    let temp_dir = tempfile::tempdir()
        .map_err(|e| cu29::CuError::new_with_cause("create temp dir failed", e))?;
    let record_path = temp_dir.path().join("recorded_no_kf.copper");
    let replay_path = temp_dir.path().join("replayed_no_kf.copper");

    let (recorded_cl, _recorded_kf) = record_reference_run(&record_path)?;
    let expected_timestamp = recorded_copperlist_timestamp(&recorded_cl)
        .expect("reference copperlist should contain a process timestamp")
        .as_nanos();

    let (replayed_cl, _replayed_kf, replay_clock) = replay_run(&replay_path, &recorded_cl, None)?;

    assert_eq!(encode_bytes(&replayed_cl), encode_bytes(&recorded_cl));
    assert_eq!(replay_clock, expected_timestamp);

    Ok(())
}

#[test]
fn builder_propagates_instance_id_into_runtime() -> CuResult<()> {
    let _guard = TEST_MUTEX
        .lock()
        .unwrap_or_else(|poison| poison.into_inner());
    let temp_dir = tempfile::tempdir()
        .map_err(|e| cu29::CuError::new_with_cause("create temp dir failed", e))?;
    let log_path = temp_dir.path().join("instance_id_runtime.copper");
    let logger = build_logger(&log_path)?;
    let (clock, _clock_mock) = RobotClock::mock();
    let mut noop = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;

    let mut app = ReplayApp::builder()
        .with_clock(clock)
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
        .with_instance_id(42)
        .with_sim_callback(&mut noop)
        .build()?
        .into_inner();

    assert_eq!(app.copper_runtime_mut().instance_id(), 42);
    Ok(())
}

#[test]
fn recorded_replay_rejects_missing_history_before_changing_clock_or_state() -> CuResult<()> {
    let _guard = TEST_MUTEX
        .lock()
        .unwrap_or_else(|poison| poison.into_inner());
    let temp = tempfile::tempdir().unwrap();
    let (mut recorded, mut keyframe) = record_reference_run(&temp.path().join("source.copper"))?;
    recorded.id = 100;
    keyframe.culistid = 100;
    let logger = build_logger(&temp.path().join("replay.copper"))?;
    let (clock, mock) = RobotClock::mock();
    let mut noop = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let mut app = ReplayApp::builder()
        .with_clock(clock)
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
        .with_sim_callback(&mut noop)
        .build()?
        .into_inner();
    app.start_all_tasks(&mut noop)?;
    let before = mock.value();
    let error = app
        .replay_recorded_copperlist(&mock, &recorded, None)
        .unwrap_err();
    assert!(error.to_string().contains("gap"));
    assert_eq!(mock.value(), before);
    assert_eq!(app.copper_runtime_mut().copperlists_manager.next_cl_id(), 0);
    // A validated received keyframe makes this boundary a legal restart.
    app.replay_recorded_copperlist(&mock, &recorded, Some(&keyframe))?;
    assert_eq!(
        app.copper_runtime_mut().copperlists_manager.next_cl_id(),
        101
    );
    recorded.id = 101;
    app.replay_recorded_copperlist(&mock, &recorded, None)?;
    assert_eq!(
        app.copper_runtime_mut().copperlists_manager.next_cl_id(),
        102
    );
    app.stop_all_tasks(&mut noop)?;
    drop(app);
    let replayed: Vec<_> =
        copperlists_reader::<default::CuStampedDataSet>(UnifiedLoggerIOReader::new(
            cu29::prelude::UnifiedLoggerRead::new(temp.path().join("replay.copper").as_path())
                .map_err(|error| CuError::new_with_cause("Open replay archive", error))?,
            UnifiedLogType::CopperList,
        ))
        .collect();
    assert_eq!(
        replayed.iter().map(|list| list.id).collect::<Vec<_>>(),
        [100, 101]
    );
    Ok(())
}
