//! Deterministic-replay non-regression test for foreground anytime nodes.
//!
//! An anytime node compiles to a base step plus `max_refines` refine steps, and
//! each step reads the clock, stamps status text and freezes the task. Replaying
//! a recorded copperlist must reproduce all of it byte for byte.

#![cfg(all(test, feature = "std"))]

use bincode::{Decode, Encode, config::standard, encode_to_vec};
use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::cutask_anytime::{AnytimeStatus, CuAnytimeTask, Quality, quality_from_f32};
use cu29::prelude::copper_runtime;
use cu29::prelude::*;
use cu29_export::{copperlists_reader, keyframes_reader};
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use std::fs;
use std::path::Path;
use std::sync::{Arc, Mutex};

/// What the sink saw, one entry per copperlist: the payload and the status text
/// the anytime runner stamped on it.
static OBSERVED: Mutex<Vec<(u32, String)>> = Mutex::new(Vec::new());

fn take_observed() -> Vec<(u32, String)> {
    let mut observed = OBSERVED.lock().expect("observer poisoned");
    std::mem::take(&mut *observed)
}

#[derive(Reflect)]
struct TargetSrc;

impl Freezable for TargetSrc {}

impl CuSrcTask for TargetSrc {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(3);
        // A fresh Tov: the planner's max_age check anchors on it.
        output.tov = Tov::Time(ctx.clock.now());
        Ok(())
    }
}

/// Scored anytime node: `base()` publishes 0 and each quantum commits one more
/// increment toward the target, so quality climbs until `quality_target` stops
/// refinement early.
#[derive(Reflect)]
struct IncrementalPlanner {
    target: u32,
    acc: u32,
}

impl Freezable for IncrementalPlanner {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&(self.target, self.acc), encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        (self.target, self.acc) = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuAnytimeTask for IncrementalPlanner {
    type Input<'m> = input_msg!(u32);
    type Output<'m> = output_msg!(u32);
    type Resources<'r> = ();
    type Quality = Quality;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { target: 0, acc: 0 })
    }

    fn base(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        self.target = input.payload().copied().ok_or("planner: no input")?;
        self.acc = 0;
        output.set_payload(self.acc);
        Ok(AnytimeStatus::Improved(quality_from_f32(0.0)))
    }

    fn refine(
        &mut self,
        _ctx: &CuContext,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        if self.acc >= self.target {
            return Ok(AnytimeStatus::Converged(quality_from_f32(1.0)));
        }
        self.acc += 1;
        output.set_payload(self.acc);
        Ok(AnytimeStatus::Improved(quality_from_f32(
            self.acc as f32 / self.target as f32,
        )))
    }
}

/// Quality-less anytime node: every quantum bumps the output, so it runs its
/// whole emitted refine budget and stops by position (`MaxRefines`).
#[derive(Reflect)]
struct CountingSmoother {
    bumps: u32,
}

impl Freezable for CountingSmoother {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.bumps, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.bumps = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuAnytimeTask for CountingSmoother {
    type Input<'m> = input_msg!(u32);
    type Output<'m> = output_msg!(u32);
    type Resources<'r> = ();
    type Quality = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { bumps: 0 })
    }

    fn base(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<()>> {
        output.set_payload(input.payload().copied().ok_or("smoother: no input")?);
        Ok(AnytimeStatus::Improved(()))
    }

    fn refine(
        &mut self,
        _ctx: &CuContext,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<()>> {
        self.bumps += 1;
        let bumped = output.payload().copied().unwrap_or(0) + 1;
        output.set_payload(bumped);
        Ok(AnytimeStatus::Improved(()))
    }
}

#[derive(Reflect)]
struct SpySink;

impl Freezable for SpySink {}

impl CuSinkTask for SpySink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let payload = input.payload().copied().ok_or("sink: no input")?;
        OBSERVED
            .lock()
            .expect("observer poisoned")
            .push((payload, input.metadata.status_txt.0.to_string()));
        Ok(())
    }
}

#[copper_runtime(config = "tests/replay_anytime_config.ron", sim_mode = true)]
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

fn read_first_copperlist(path: &Path) -> CuResult<RecordedCl> {
    let UnifiedLogger::Read(read_logger) = UnifiedLoggerBuilder::new()
        .file_base_name(path)
        .build()
        .map_err(|e| cu29::CuError::new_with_cause("open copperlist log failed", e))?
    else {
        return Err(cu29::CuError::from("logger builder did not return reader"));
    };
    let mut reader = UnifiedLoggerIOReader::new(read_logger, UnifiedLogType::CopperList);
    copperlists_reader::<default::CuStampedDataSet>(&mut reader)
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

type RecordedCl = CopperList<default::CuStampedDataSet>;
/// One recorded copperlist, its keyframe, and what the sink saw while producing it.
type RecordedRun = (RecordedCl, KeyFrame, Vec<(u32, String)>);

/// Records one copperlist and returns it with its keyframe and what the sink saw.
fn record_reference_run(log_path: &Path) -> CuResult<RecordedRun> {
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
        read_first_copperlist(log_path)?,
        read_first_keyframe(log_path)?,
        take_observed(),
    ))
}

/// Replays `recorded_cl` on a fresh application and returns what it produced.
///
/// Replay injects the recorded messages instead of running the tasks
/// (`SimOverride::ExecutedBySim`), so nothing here depends on the tasks
/// re-executing — it checks that the anytime steps still line up with the
/// recorded copperlist slot by slot.
fn replay_run(
    log_path: &Path,
    recorded_cl: &RecordedCl,
    keyframe: &KeyFrame,
) -> CuResult<(RecordedCl, KeyFrame)> {
    let logger = build_logger(log_path)?;
    let (clock, clock_mock) = RobotClock::mock();
    let mut noop = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;

    let mut app = AnytimeReplayApp::builder()
        .with_clock(clock)
        .with_logger::<MmapSectionStorage, MmapUnifiedLoggerWrite>(logger)
        .with_sim_callback(&mut noop)
        .build()?;

    app.start_all_tasks(&mut noop)?;
    app.replay_recorded_copperlist(&clock_mock, recorded_cl, Some(keyframe))?;
    app.stop_all_tasks(&mut noop)?;

    drop(app);
    let _ = take_observed();

    Ok((
        read_first_copperlist(log_path)?,
        read_first_keyframe(log_path)?,
    ))
}

/// One test function on purpose: the log runtime is a process-wide `OnceLock`,
/// so several tests building applications in the same binary race on it.
#[test]
fn anytime_nodes_run_and_replay_deterministically() -> CuResult<()> {
    let temp_dir = tempfile::tempdir()
        .map_err(|e| cu29::CuError::new_with_cause("create temp dir failed", e))?;
    let first_path = temp_dir.path().join("first.copper");
    let second_path = temp_dir.path().join("second.copper");
    let replay_path = temp_dir.path().join("replayed.copper");

    let (first_cl, first_kf, first_observed) = record_reference_run(&first_path)?;

    // Pins the schedule the plan expanded to, so losing refinement quanta fails
    // here and not only as a byte mismatch: the planner reaches its target of 3
    // in three quanta, then the smoother bumps twice and stops by position.
    assert_eq!(
        first_observed,
        vec![(5u32, "any:2it max".to_string())],
        "reference run did not run the anytime schedule"
    );

    // Same input, fresh application: the base/refine steps must produce the same
    // payloads, the same status stamps and the same timings.
    let (second_cl, second_kf, second_observed) = record_reference_run(&second_path)?;
    assert_eq!(second_observed, first_observed);
    assert_eq!(encode_bytes(&second_cl), encode_bytes(&first_cl));
    assert_eq!(encode_bytes(&second_kf), encode_bytes(&first_kf));

    // Replaying the recorded copperlist reproduces it byte for byte, keyframe
    // included: one anytime node is still one replay step and one freeze.
    let (replayed_cl, replayed_kf) = replay_run(&replay_path, &first_cl, &first_kf)?;
    assert_eq!(encode_bytes(&replayed_cl), encode_bytes(&first_cl));
    assert_eq!(encode_bytes(&replayed_kf), encode_bytes(&first_kf));

    Ok(())
}
