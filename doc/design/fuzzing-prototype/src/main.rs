//! Standalone LibAFL/Copper integration probe, not a generated robot application.
//! Follow Imu → Scenario → EngineInput → TemporalMutator → run_scenario → main.
//!
//! LANDING comments describe proposed production homes, not existing crates:
//! - Schema: core/cu29_fuzz_schema (public data/RPC types; no LibAFL dependency).
//! - Engine: core/cu29_fuzz (open host-only engine; private LibAFL implementation).
//! - Runtime: cu29_runtime/cu29_derive (generic host injection access only).
//! - CLI/API: thin clients and worker service over the same engine.
//! - Paid GUI: ../cu-timetraveler (authoring, progress, investigation).

use std::borrow::Cow;
use std::cell::Cell;
use std::rc::Rc;

use bevy_reflect::{Reflect, ReflectMut};
use bincode::{Decode, Encode};
use cu29_clock::{CuTime, RobotClock, Tov};
use cu29_runtime::cutask::CuMsg;
use libafl::corpus::{Corpus, CorpusId, InMemoryCorpus, Testcase};
use libafl::events::NopEventManager;
use libafl::executors::{ExitKind, InProcessExecutor};
use libafl::feedbacks::{ConstFeedback, Feedback, StateInitializer};
use libafl::fuzzer::Fuzzer;
use libafl::inputs::Input;
use libafl::mutators::{HavocScheduledMutator, MutationResult, Mutator};
use libafl::schedulers::RandScheduler;
use libafl::stages::StdMutationalStage;
use libafl::state::{HasExecutions, HasSolutions, StdState};
use libafl::{Error, StdFuzzer};
use libafl_bolts::Named;
use libafl_bolts::rands::StdRand;
use libafl_bolts::tuples::tuple_list;
use serde::{Deserialize, Serialize};

// LANDING: user's application, or components/payloads for a reusable sensor type.
// Ordinary Copper payload: no Hash, Input, Mutator, or other LibAFL traits.
// The fuzzer discovers its existing schema; users do not write this for fuzzing.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
struct Imu {
    accel_z: f32,
}

// LANDING: public core/cu29_fuzz_schema::scenario, with the split noted below.
// Shared serialized data for GUI/CLI/CI; no LibAFL traits on this contract.
// This toy combines a portable scenario with a resolved execution plan.
#[derive(Clone, Debug, Hash, Serialize, Deserialize)]
struct Scenario {
    // Public mutation data; production also needs port, event, and timing scope.
    hold_ms: u64,
    // Engine-internal access plan ONLY. Public cases name fields/ports and schema
    // identity; resolve to indices once per campaign. Never export this index as
    // the stable field identifier: a rebuilt payload can reorder fields.
    field_index: usize,
    // Public scalar-value representation can preserve float bits, including NaNs.
    replacement_bits: u32,
}

// LANDING: private core/cu29_fuzz::backend::libafl, even though the engine is open.
// Private means Rust implementation detail, not commercially restricted source.
// Changing fuzz engines must not change the public campaign/case schema.
#[derive(Clone, Debug, Hash, Serialize, Deserialize)]
#[serde(transparent)]
struct EngineInput(Scenario);

impl Input for EngineInput {}

// LANDING: core/cu29_fuzz mutation engine + private LibAFL adapter.
// Generic campaign-driven operators replace this hard-coded duration sweep;
// no implementation is required on individual application payloads.
struct TemporalMutator {
    name: Cow<'static, str>,
    next: usize,
}

impl Named for TemporalMutator {
    fn name(&self) -> &Cow<'static, str> {
        &self.name
    }
}

impl<State> Mutator<EngineInput, State> for TemporalMutator {
    fn mutate(
        &mut self,
        _state: &mut State,
        input: &mut EngineInput,
    ) -> Result<MutationResult, Error> {
        // A deterministic boundary sweep, not a throughput/discovery benchmark.
        input.0.hold_ms = [0, 90, 100, 110, 140, 250][self.next % 6];
        input.0.replacement_bits = 9.81_f32.to_bits();
        self.next += 1;
        Ok(MutationResult::Mutated)
    }

    fn post_exec(
        &mut self,
        _state: &mut State,
        _new_corpus_id: Option<CorpusId>,
    ) -> Result<(), Error> {
        Ok(())
    }
}

// LANDING: private core/cu29_fuzz::backend::libafl objective adapter.
// Actual check evaluation lives in the engine's properties module; public schema
// contains the GUI-defined expressions/deadlines, not this Feedback trait.
struct PropertyFeedback {
    name: Cow<'static, str>,
    // Prototype-local result handoff; not a public API or checkpoint format.
    failed: Rc<Cell<bool>>,
}

impl Named for PropertyFeedback {
    fn name(&self) -> &Cow<'static, str> {
        &self.name
    }
}

impl<State> StateInitializer<State> for PropertyFeedback {}

impl<EventManager, InputType, Observers, State> Feedback<EventManager, InputType, Observers, State>
    for PropertyFeedback
{
    fn is_interesting(
        &mut self,
        _state: &mut State,
        _manager: &mut EventManager,
        _input: &InputType,
        _observers: &Observers,
        _exit_kind: &ExitKind,
    ) -> Result<bool, Error> {
        Ok(self.failed.get())
    }
}

// LANDING: core/cu29_fuzz::injection (host-only reflection interpreter).
// cu29_runtime/cu29_derive supply generic mutable message/ToV access at simulation
// callbacks. They must not know about campaigns, mutation operators, or LibAFL.
// Resolve/validate paths up front; production failures return CuResult, not panic.
fn inject_scalar(payload: &mut dyn Reflect, index: usize, bits: u32) {
    let ReflectMut::Struct(fields) = payload.reflect_mut() else {
        panic!("probe expects a struct");
    };
    let field = fields.field_at_mut(index).expect("resolved field exists");
    *field
        .try_downcast_mut::<f32>()
        .expect("resolved scalar type") = f32::from_bits(bits);
}

// LANDING: core/cu29_fuzz::runner around a generated application harness.
// This function mixes runner, fake controller, and check logic only for the demo.
// Production uses a generated typed app shim; users write no equivalent function.
fn run_scenario(input: &Scenario, broken_controller: bool) -> bool {
    // RUNNER: reset application + resources/RNG/clock for each candidate.
    // RobotClock and CuMsg are existing Copper APIs, reused unchanged.
    let (clock, clock_mock) = RobotClock::mock();
    let mut msg = CuMsg::<Imu>::new(Some(Imu::default()));
    // INJECTION: seed from the log and mutate the actual output slot before its
    // consumers run. No remote calls, JSON conversion, or original-output replay.
    let payload = msg.payload_mut().as_mut().expect("seeded payload exists");
    inject_scalar(payload, input.field_index, input.replacement_bits);
    assert_eq!(payload.accel_z.to_bits(), input.replacement_bits);
    msg.tov = Tov::Time(CuTime::from_millis(0));
    // PROPERTIES: per-candidate temporal obligation state; never stored in tasks.
    let mut deadline = None;

    // RUNNER: virtual schedule + source delivery; replace this toy loop with
    // generated app iterations and SimStep callbacks at supported boundaries.
    for cycle in 0..=30 {
        let now_ms = cycle * 10;
        clock_mock.set_value(now_ms * 1_000_000);
        if now_ms > input.hold_ms {
            msg.tov = Tov::Time(clock.now());
        }
        let Tov::Time(sample_time) = msg.tov else {
            panic!("probe seed has a point ToV");
        };
        let stale = clock.now().as_millis() - sample_time.as_millis() > 100;
        // APPLICATION: planted defect, not Copper/fuzzer logic. In production
        // the real controller task runs and its motor output is observed.
        let motors_enabled = msg.payload().is_some() && (broken_controller || !stale);
        // PROPERTIES: generic expression/deadline evaluator replaces this
        // IMU-specific check. GUI config supplies selectors, limits, and response.
        if stale {
            let due = *deadline.get_or_insert(cycle + 2);
            if cycle >= due && motors_enabled {
                return true;
            }
        } else {
            deadline = None;
        }
    }
    false
}

// LANDING: split CLI startup, worker service, backend setup, and artifact export.
// The CLI/GUI eventually call the same worker API; neither owns the search loop.
// The worker links engine + app harness in-process. All names below are private
// wiring except the future schema/config/status contracts.
fn main() -> Result<(), Error> {
    // DEMO ONLY: baseline + planted-bug/fixed-model assertions become integration
    // fixtures. Production gets seed/config/checks from public campaign data.
    let seed = Scenario {
        hold_ms: 0,
        field_index: 0,
        replacement_bits: 0.0_f32.to_bits(),
    };
    assert!(!run_scenario(&seed, true));
    let failing = Scenario {
        hold_ms: 140,
        ..seed.clone()
    };
    assert!(run_scenario(&failing, true));
    assert!(!run_scenario(&failing, false));

    // BACKEND: LibAFL corpus, RNG, scheduler, mutation stages, and objectives.
    // Open engine internals, not public API and not part of cu29_runtime.
    let failed = Rc::new(Cell::new(false));
    // DEMO ONLY: replace constant acceptance with real edge/state feedback.
    let mut feedback = ConstFeedback::new(true);
    let mut objective = PropertyFeedback {
        name: Cow::Borrowed("temporal_property"),
        failed: failed.clone(),
    };
    let mut corpus = InMemoryCorpus::<EngineInput>::new();
    corpus.add(Testcase::new(EngineInput(seed)))?;
    let mut state = StdState::new(
        StdRand::with_seed(42),
        corpus,
        InMemoryCorpus::<EngineInput>::new(),
        &mut feedback,
        &mut objective,
    )?;
    let mut fuzzer = StdFuzzer::new(RandScheduler::new(), feedback, objective);
    let mut manager = NopEventManager::new();
    // RUNNER ADAPTER: invoke the generated app shim locally and publish check
    // results to the objective. No per-field or per-cycle RPC.
    let mut harness = |input: &EngineInput| {
        failed.set(run_scenario(&input.0, true));
        ExitKind::Ok // A property failure is not reported as an OS crash.
    };
    let mut executor = InProcessExecutor::builder()
        .harness(&mut harness)
        .observers(tuple_list!())
        .fuzzer(&mut fuzzer)
        .state(&mut state)
        .event_mgr(&mut manager)
        .build()?;
    let mutator = HavocScheduledMutator::new(tuple_list!(TemporalMutator {
        name: Cow::Borrowed("temporal_mutator"),
        next: 0,
    }));
    let mut stages = tuple_list!(StdMutationalStage::new(mutator));
    // WORKER SERVICE: campaign budget/pause/checkpoints wrap this engine loop.
    // Batched public status goes to CLI/GUI. An external watchdog covers hangs.
    for _ in 0..100 {
        fuzzer.fuzz_one(&mut stages, &mut executor, &mut state, &mut manager)?;
        if state.solutions().count() > 0 {
            break;
        }
    }
    assert!(
        state.solutions().count() > 0,
        "LibAFL saved the temporal failure"
    );
    let solution_id = state.solutions().first().expect("solution exists");
    let solution = state
        .solutions()
        .get(solution_id)?
        .borrow_mut()
        .load_input(state.solutions())?
        .clone();
    assert!(run_scenario(&solution.0, true), "saved case reproduces");
    assert!(!run_scenario(&solution.0, false), "fixed controller passes");
    // ARTIFACTS: public versioned case export belongs to core/cu29_fuzz_schema
    // plus engine I/O. LibAFL Input::to_file is used only to prove persistence;
    // its backend format is not the public portable case format.
    let output_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../../target/fuzz-design/fuzzing-prototype");
    std::fs::create_dir_all(&output_dir)?;
    let case_path = output_dir.join("scenario.case");
    solution.to_file(&case_path)?;
    let reloaded = EngineInput::from_file(&case_path)?;
    assert!(run_scenario(&reloaded.0, true), "persisted case reproduces");
    // CLI PRESENTATION: real service emits a public summary DTO; CLI prints it,
    // while the paid TimeTraveler GUI displays progress/findings and opens replay.
    eprintln!(
        "probe passed: LibAFL 0.16.1; executions={}; findings={}; hold_ms={}; local CuMsg<f32-payload> reflection + virtual clock; saved case replayed; fixed control passed",
        state.executions(),
        state.solutions().count(),
        reloaded.0.hold_ms
    );
    Ok(())
}
