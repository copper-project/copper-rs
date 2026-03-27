#![cfg(all(test, feature = "std"))]

use bincode::{Decode, Encode, config::standard, encode_to_vec};
use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::prelude::copper_runtime;
use cu29::prelude::*;
use cu29::simulation::recorded_copperlist_timestamp;
use cu29_export::{copperlists_reader, keyframes_reader};
use cu29_unifiedlog::memmap::MmapUnifiedLoggerWrite;
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;
use std::sync::{Arc, Mutex};

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
struct CounterMsg {
    value: u32,
}

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
struct AccumMsg {
    sum: u32,
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

fn record_reference_run(
    log_path: &Path,
) -> CuResult<(CopperList<default::CuStampedDataSet>, KeyFrame)> {
    let logger = build_logger(log_path)?;
    let (clock, _clock_mock) = RobotClock::mock();
    let mut noop = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;

    let mut app = ReplayAppBuilder::new()
        .with_clock(clock)
        .with_unified_logger(logger)
        .with_sim_callback(&mut noop)
        .build()?;

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

    let mut app = ReplayAppBuilder::new()
        .with_clock(clock)
        .with_unified_logger(logger)
        .with_sim_callback(&mut noop)
        .build()?;

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
