#![cfg(all(test, feature = "std"))]

//! Exact-output replay test for the anytime interface. A deterministic in-tree
//! anytime task is wrapped by `AnytimeTask<A>` and run once under record. The
//! existing exact-output path must reproduce the recorded bytes.

use bincode::{Decode, Encode, config::standard, encode_to_vec};
use cu29::cutask_anytime::{Anytime, AnytimeOutput, Progress, Step};
use cu29::prelude::copper_runtime;
use cu29::prelude::*;
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;
use std::sync::{Arc, Mutex};

static REPLAY_TEST_LOCK: Mutex<()> = Mutex::new(());

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
struct CounterMsg {
    value: u32,
}

#[derive(Reflect)]
struct CounterSrc {
    next: u32,
}

impl Freezable for CounterSrc {
    fn freeze<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Encode::encode(&self.next, encoder)
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
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

// Non-converging anytime task: live execution stops on its physical refinement
// budget even though the RobotClock is frozen.
#[derive(Reflect)]
struct TestPlanner {
    best_val: u32,
    pass_count: u32,
}

impl Freezable for TestPlanner {
    fn freeze<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Encode::encode(&self.best_val, encoder)?;
        Encode::encode(&self.pass_count, encoder)?;
        Ok(())
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        self.best_val = Decode::decode(decoder)?;
        self.pass_count = Decode::decode(decoder)?;
        Ok(())
    }
}

impl Anytime for TestPlanner {
    type Input = CounterMsg;
    type Output = CounterMsg;
    type Resources<'r> = ();

    fn new(_cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            best_val: 0,
            pass_count: 0,
        })
    }

    fn base(&mut self, input: &Self::Input, _ctx: &CuContext) -> CuResult<Step> {
        self.best_val = input.value;
        self.pass_count = 0;
        Ok(Step::Continue)
    }

    fn refine(&mut self, _input: &Self::Input) -> CuResult<Step> {
        self.pass_count += 1;
        self.best_val += 1;
        Ok(Step::Continue)
    }

    fn write_best(&mut self, out: &mut Self::Output) -> CuResult<()> {
        out.value = self.best_val;
        Ok(())
    }
}

#[derive(Reflect)]
struct AnytimeSpySink {
    last_value: Option<u32>,
    last_progress: Option<Progress>,
}

impl Freezable for AnytimeSpySink {
    fn freeze<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        Encode::encode(&self.last_value, encoder)?;
        Encode::encode(&self.last_progress, encoder)
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        self.last_value = Decode::decode(decoder)?;
        self.last_progress = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSinkTask for AnytimeSpySink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(AnytimeOutput<CounterMsg>);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            last_value: None,
            last_progress: None,
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(payload) = input.payload() {
            self.last_value = Some(payload.value.value);
            self.last_progress = Some(payload.progress);
        }
        Ok(())
    }
}

#[copper_runtime(config = "tests/anytime_replay_config.ron", sim_mode = true)]
struct AnytimeReplayApp {}

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
    cu29_export::copperlists_reader::<P>(&mut reader)
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
    cu29_export::keyframes_reader(&mut reader)
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

    let mut app = AnytimeReplayApp::builder()
        .with_clock(clock)
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
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

fn exact_replay_run(
    log_path: &Path,
    recorded_cl: &CopperList<default::CuStampedDataSet>,
    keyframe: Option<&KeyFrame>,
) -> CuResult<(CopperList<default::CuStampedDataSet>, KeyFrame)> {
    let logger = build_logger(log_path)?;
    let (clock, clock_mock) = RobotClock::mock();
    let mut noop = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;

    let mut app = AnytimeReplayApp::builder()
        .with_clock(clock)
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
        .with_sim_callback(&mut noop)
        .build()?;

    app.start_all_tasks(&mut noop)?;
    app.replay_recorded_copperlist(&clock_mock, recorded_cl, keyframe)?;
    app.stop_all_tasks(&mut noop)?;

    drop(app);

    Ok((
        read_first_copperlist::<default::CuStampedDataSet>(log_path)?,
        read_first_keyframe(log_path)?,
    ))
}

#[test]
fn anytime_task_exact_replay_reproduces_recorded_bytes() -> CuResult<()> {
    let _guard = REPLAY_TEST_LOCK
        .lock()
        .map_err(|_| cu29::CuError::from("anytime replay test lock poisoned"))?;
    let temp_dir = tempfile::tempdir()
        .map_err(|e| cu29::CuError::new_with_cause("create temp dir failed", e))?;
    let record_path = temp_dir.path().join("anytime_recorded.copper");
    let replay_path = temp_dir.path().join("anytime_replayed.copper");

    let (recorded_cl, recorded_kf) = record_reference_run(&record_path)?;
    let (replayed_cl, replayed_kf) =
        exact_replay_run(&replay_path, &recorded_cl, Some(&recorded_kf))?;

    // Same pass count, same best value, same tov, byte-for-byte.
    assert_eq!(encode_bytes(&replayed_cl), encode_bytes(&recorded_cl));
    assert_eq!(encode_bytes(&replayed_kf), encode_bytes(&recorded_kf));

    Ok(())
}
