//! Deterministic-replay non-regression test for foreground anytime nodes.
//!
//! An anytime node compiles to a base step plus `max_refines` refine steps, and
//! each step reads the clock, stamps status text and (for the base step) freezes
//! the task. This test records a run, then resims it: the source is injected
//! from the recording and the sink is suppressed, but the two anytime nodes
//! really re-execute their base and every refine quantum. The resim must
//! reproduce the recorded copperlists and keyframes byte for byte.

#![cfg(all(test, feature = "std"))]

use bincode::{Decode, Encode, config::standard, encode_to_vec};
use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::cutask_anytime::{AnytimeStatus, CuAnytimeTask, Quality, quality_from_f32};
use cu29::prelude::copper_runtime;
use cu29::prelude::*;
use cu29::simulation::recorded_copperlist_timestamp;
use cu29_export::{copperlists_reader, keyframes_reader};
use std::collections::BTreeSet;
use std::path::Path;
use std::sync::atomic::{AtomicU32, Ordering};

/// Copperlists recorded and resimmed: enough to cycle the source targets
/// several times and to put a few keyframes in the log.
const ITERATIONS: u64 = 64;
/// Mock-clock step between two copperlists (1 ms).
const DT_NANOS: u64 = 1_000_000;
const LOG_SLAB_SIZE: Option<usize> = Some(16 * 1024 * 1024);

/// Targets the source cycles through, each held for `TARGET_HOLD` copperlists
/// so the planner can close in on one before it moves.
const TARGETS: [u32; 5] = [0, 12, 4, 20, 8];
const TARGET_HOLD: u64 = 8;
/// Copperlists whose input carries a timestamp older than the planner's
/// `max_age_ms`, so the planner skips the job.
const STALE_PHASE: u64 = 9;
const STALE_PERIOD: u64 = 16;
/// How much older than `now` a stale input is stamped: above the 2 ms
/// `max_age_ms` in the config.
const STALE_AGE_NANOS: u64 = 3 * DT_NANOS;

/// `base()` and `refine()` calls the two anytime nodes actually made, counted
/// across both of them. A replay that only copies recorded bytes back leaves
/// these at zero, so the test compares them between the recording and the resim.
static BASE_CALLS: AtomicU32 = AtomicU32::new(0);
static REFINE_QUANTA: AtomicU32 = AtomicU32::new(0);

/// Reads and clears the two counters.
fn take_anytime_calls() -> (u32, u32) {
    (
        BASE_CALLS.swap(0, Ordering::Relaxed),
        REFINE_QUANTA.swap(0, Ordering::Relaxed),
    )
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
        // The mock clock doubles as the copperlist counter: the test moves it by
        // DT_NANOS per iteration, so the source needs no state of its own.
        let now = ctx.clock.now();
        let index = now.as_nanos() / DT_NANOS;
        output.set_payload(TARGETS[(index / TARGET_HOLD) as usize % TARGETS.len()]);
        output.tov = if index % STALE_PERIOD == STALE_PHASE {
            Tov::Time(now - CuDuration(STALE_AGE_NANOS))
        } else {
            Tov::Time(now)
        };
        Ok(())
    }
}

/// Distance at which the planner quality reaches 0. `quality_target: 0.9` is met
/// at a distance of 1 (0.9375) but not at 2 (0.875), so the target stops the job
/// one step before it lands.
const PLANNER_SCALE: f32 = 16.0;

fn planner_quality(target: u32, acc: u32) -> Quality {
    quality_from_f32((1.0 - target.abs_diff(acc) as f32 / PLANNER_SCALE).max(0.0))
}

/// Scored anytime node: each quantum walks `acc` one step toward the target the
/// source asks for. `base` does not reset `acc`, so the solution carries across
/// copperlists and a single lost quantum shifts every later copperlist.
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
        BASE_CALLS.fetch_add(1, Ordering::Relaxed);
        self.target = input.payload().copied().ok_or("planner: no input")?;
        output.set_payload(self.acc);
        Ok(AnytimeStatus::Improved(planner_quality(
            self.target,
            self.acc,
        )))
    }

    fn refine(
        &mut self,
        _ctx: &CuContext,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        REFINE_QUANTA.fetch_add(1, Ordering::Relaxed);
        if self.acc == self.target {
            return Ok(AnytimeStatus::Converged(quality_from_f32(1.0)));
        }
        if self.acc < self.target {
            self.acc += 1;
        } else {
            self.acc -= 1;
        }
        output.set_payload(self.acc);
        Ok(AnytimeStatus::Improved(planner_quality(
            self.target,
            self.acc,
        )))
    }
}

/// Quality-less anytime node: it runs its whole emitted refine budget unless it
/// converges. `bumps` counts every quantum it ever ran and is added to the
/// payload, so a lost quantum stays visible in every later copperlist.
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
        BASE_CALLS.fetch_add(1, Ordering::Relaxed);
        let Some(value) = input.payload().copied() else {
            // The planner skipped a stale job: there is nothing to smooth and no
            // result this task can vouch for.
            output.clear_payload();
            return Ok(AnytimeStatus::Aborted);
        };
        output.set_payload(value + self.bumps);
        Ok(AnytimeStatus::Improved(()))
    }

    fn refine(
        &mut self,
        _ctx: &CuContext,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<()>> {
        REFINE_QUANTA.fetch_add(1, Ordering::Relaxed);
        let value = output.payload().copied().ok_or("smoother: no output")?;
        if value.is_multiple_of(5) {
            return Ok(AnytimeStatus::Converged(()));
        }
        self.bumps += 1;
        output.set_payload(value + 1);
        Ok(AnytimeStatus::Improved(()))
    }
}

/// Terminal consumer of the graph. It collects nothing: the resim suppresses
/// sinks, so anything gathered here would be missing on the replay side. The
/// test reads what the run produced from the log instead.
#[derive(Reflect)]
struct SpySink;

impl Freezable for SpySink {}

impl CuSinkTask for SpySink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(config = "tests/replay_anytime_config.ron", sim_mode = true)]
struct AnytimeReplayApp {}

type RecordedCl = CopperList<default::CuStampedDataSet>;

fn open_reader(path: &Path, log_type: UnifiedLogType) -> CuResult<UnifiedLoggerIOReader> {
    let UnifiedLogger::Read(read_logger) = UnifiedLoggerBuilder::new()
        .file_base_name(path)
        .build()
        .map_err(|e| cu29::CuError::new_with_cause("open log failed", e))?
    else {
        return Err(cu29::CuError::from(
            "logger builder did not return a reader",
        ));
    };
    Ok(UnifiedLoggerIOReader::new(read_logger, log_type))
}

fn read_copperlists(path: &Path) -> CuResult<Vec<RecordedCl>> {
    let mut reader = open_reader(path, UnifiedLogType::CopperList)?;
    Ok(copperlists_reader::<default::CuStampedDataSet>(&mut reader).collect())
}

fn read_keyframes(path: &Path) -> CuResult<Vec<KeyFrame>> {
    let mut reader = open_reader(path, UnifiedLogType::FrozenTasks)?;
    Ok(keyframes_reader(&mut reader).collect())
}

fn encode<T: Encode>(value: &T) -> Vec<u8> {
    encode_to_vec(value, standard()).expect("bincode encode for byte comparison")
}

/// One-line summary of what the anytime nodes left in a copperlist, for the
/// failure message: a byte dump of 64 copperlists says nothing useful.
fn describe(copperlist: &RecordedCl) -> String {
    let planner = copperlist.msgs.get_planner_output();
    let smoother = copperlist.msgs.get_smoother_output();
    format!(
        "planner={:?} [{}], smoother={:?} [{}]",
        planner.payload(),
        planner.metadata.status_txt.0,
        smoother.payload(),
        smoother.metadata.status_txt.0,
    )
}

/// Compares what two runs produced and names the first copperlist that differs.
/// The whole CopperList is compared, ids and message metadata included, like
/// `examples/cu_caterpillar/src/determinism.rs` does.
fn assert_copperlists_eq(actual: &[RecordedCl], expected: &[RecordedCl], what: &str) {
    assert_eq!(
        actual.len(),
        expected.len(),
        "{what}: copperlist count differs"
    );
    for (index, (actual_cl, expected_cl)) in actual.iter().zip(expected).enumerate() {
        assert!(
            encode(actual_cl) == encode(expected_cl),
            "{what}: copperlist {index} differs\n  got:      id {} {}\n  recorded: id {} {}",
            actual_cl.id,
            describe(actual_cl),
            expected_cl.id,
            describe(expected_cl)
        );
    }
}

fn assert_keyframes_eq(actual: &[KeyFrame], expected: &[KeyFrame], what: &str) {
    assert_eq!(
        actual.len(),
        expected.len(),
        "{what}: keyframe count differs"
    );
    for (index, (actual_kf, expected_kf)) in actual.iter().zip(expected).enumerate() {
        assert!(
            encode(actual_kf) == encode(expected_kf),
            "{what}: keyframe {index} differs (copperlist {} vs {})",
            actual_kf.culistid,
            expected_kf.culistid
        );
    }
}

/// Last word of the `"any:{N}it [q=X.XX ]{label}[!]"` stamp an anytime node
/// leaves on its output, without the not-published marker.
fn stop_cause(status: &str) -> &str {
    status
        .rsplit(' ')
        .next()
        .unwrap_or_default()
        .trim_end_matches('!')
}

fn planner_stop_causes(copperlists: &[RecordedCl]) -> BTreeSet<&str> {
    copperlists
        .iter()
        .map(|cl| stop_cause(cl.msgs.get_planner_output().metadata.status_txt.0.as_str()))
        .collect()
}

fn smoother_stop_causes(copperlists: &[RecordedCl]) -> BTreeSet<&str> {
    copperlists
        .iter()
        .map(|cl| stop_cause(cl.msgs.get_smoother_output().metadata.status_txt.0.as_str()))
        .collect()
}

fn record_run(log_path: &Path) -> CuResult<()> {
    let (clock, clock_mock) = RobotClock::mock();
    let mut noop = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;

    let mut app = AnytimeReplayApp::builder()
        .with_clock(clock)
        .with_log_path(log_path, LOG_SLAB_SIZE)?
        .with_sim_callback(&mut noop)
        .build()?;

    app.start_all_tasks(&mut noop)?;
    for index in 0..ITERATIONS {
        clock_mock.set_value(index * DT_NANOS);
        app.run_one_iteration(&mut noop)?;
    }
    app.stop_all_tasks(&mut noop)?;
    drop(app);
    Ok(())
}

/// Runs one recorded copperlist through the runtime. `recorded_debug_replay_step`
/// injects the recorded source output and suppresses the sink, but leaves the
/// two anytime nodes to the runtime, so their base and refine quanta execute for
/// real against the recorded input.
fn resim_one(
    app: &mut AnytimeReplayApp,
    clock_mock: &RobotClockMock,
    recorded: &RecordedCl,
) -> CuResult<()> {
    let timestamp = recorded_copperlist_timestamp(recorded).ok_or_else(|| {
        cu29::CuError::from(format!(
            "recorded copperlist {} has no process_time.start",
            recorded.id
        ))
    })?;
    clock_mock.set_value(timestamp.as_nanos());
    let mut callback =
        |step: default::SimStep<'_>| default::recorded_debug_replay_step(step, recorded);
    app.run_one_iteration(&mut callback)
}

/// Resims `recorded` on a fresh application: the tasks start from `new()` and
/// the resim covers the whole recording.
fn resim_run(recorded: &[RecordedCl], log_path: &Path) -> CuResult<()> {
    let (clock, clock_mock) = RobotClock::mock();
    let mut noop = |_step: default::SimStep<'_>| SimOverride::ExecuteByRuntime;

    let mut app = AnytimeReplayApp::builder()
        .with_clock(clock)
        .with_log_path(log_path, LOG_SLAB_SIZE)?
        .with_sim_callback(&mut noop)
        .build()?;

    app.start_all_tasks(&mut noop)?;
    for copperlist in recorded {
        resim_one(&mut app, &clock_mock, copperlist)?;
    }
    app.stop_all_tasks(&mut noop)?;
    drop(app);
    Ok(())
}

/// One test function on purpose: the log runtime is a process-wide `OnceLock`,
/// so several tests building applications in the same binary race on it.
#[test]
fn anytime_nodes_record_and_resim_deterministically() -> CuResult<()> {
    let temp_dir = tempfile::tempdir()
        .map_err(|e| cu29::CuError::new_with_cause("create temp dir failed", e))?;
    let record_a = temp_dir.path().join("record_a.copper");
    let record_b = temp_dir.path().join("record_b.copper");
    let resim_a = temp_dir.path().join("resim_a.copper");

    record_run(&record_a)?;
    let recorded_calls = take_anytime_calls();
    record_run(&record_b)?;
    let _ = take_anytime_calls();

    let a_copperlists = read_copperlists(&record_a)?;
    let a_keyframes = read_keyframes(&record_a)?;
    assert_eq!(
        a_copperlists.len(),
        ITERATIONS as usize,
        "the recording lost copperlists"
    );
    assert!(
        !a_keyframes.is_empty(),
        "the recording captured no keyframe"
    );

    // The recording has to exercise the anytime schedule, not one path through
    // it: a run that only ever stopped one way would compare equal for the
    // wrong reason.
    assert_eq!(
        planner_stop_causes(&a_copperlists),
        BTreeSet::from(["max", "stale", "tgt"]),
        "the scored planner did not cover the quality target, the plan bound and the age check"
    );
    assert_eq!(
        smoother_stop_causes(&a_copperlists),
        BTreeSet::from(["abort", "conv", "max"]),
        "the quality-less smoother did not cover convergence, the plan bound and the abort path"
    );

    // The anytime state has to cross copperlists and reach the output, or a lost
    // quantum would stay invisible to the comparisons below.
    let planner_payloads: BTreeSet<Option<u32>> = a_copperlists
        .iter()
        .map(|cl| cl.msgs.get_planner_output().payload().copied())
        .collect();
    assert!(
        planner_payloads.len() > 4,
        "the planner published {} distinct payloads: its state does not carry across copperlists",
        planner_payloads.len()
    );

    // Two recordings of the same application are identical.
    assert_copperlists_eq(
        &read_copperlists(&record_b)?,
        &a_copperlists,
        "second recording",
    );
    assert_keyframes_eq(
        &read_keyframes(&record_b)?,
        &a_keyframes,
        "second recording",
    );

    // Resim: every base and refine quantum runs again and must reproduce the log
    // byte for byte, keyframes included.
    resim_run(&a_copperlists, &resim_a)?;
    assert_eq!(
        take_anytime_calls(),
        recorded_calls,
        "the resim did not run the same anytime quanta as the recording"
    );
    assert_copperlists_eq(&read_copperlists(&resim_a)?, &a_copperlists, "resim");
    assert_keyframes_eq(&read_keyframes(&resim_a)?, &a_keyframes, "resim");

    Ok(())
}
