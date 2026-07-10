//! Anytime tasks: interruptible computations that produce a usable base result
//! and monotonically refine it until a per-cycle refinement budget expires.
//!
//! The [`Anytime`] trait is the user-facing interface; [`AnytimeTask`] adapts
//! any `A: Anytime` into a plain [`CuTask`] the runtime can schedule. Each
//! emitted [`AnytimeOutput`] carries a [`Progress`] marker.

use crate::config::ComponentConfig;
use crate::context::CuContext;
use crate::cutask::{CuMsg, CuMsgPayload, CuTask, Freezable};
use crate::reflect::{GetTypeRegistration, Reflect, TypePath, TypeRegistry};
use crate::{input_msg, output_msg};

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29_clock::{CuDuration, CuInstant, Tov};
use cu29_traits::{CuError, CuResult};
use serde::{Deserialize, Serialize};

use alloc::format;

/// Outcome of a single refinement step.
///
/// `Continue` means the task can still improve if given more time. The two
/// early-exit variants are semantically distinct and are logged separately in
/// [`Progress`] so operators can tell them apart:
///
/// - `Converged` — no further improvement is *possible* (algorithmic fixpoint).
/// - `Satisfied` — no further improvement is *needed* (task-defined quality
///   target reached). Improvement may still be possible, but the task judges
///   it not worth the cycles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Reflect)]
pub enum Step {
    Continue,
    Converged,
    Satisfied,
}

/// Quality marker emitted alongside every [`AnytimeOutput`].
///
/// The adapter owns the refinement count, so it always reflects the number of
/// calls made to [`Anytime::refine`]. Downstream tasks can gate on both that
/// count and the reason refinement stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub enum Progress {
    /// The configured refinement budget expired.
    RefinementBudgetExhausted { refinements: u32 },
    /// The task reported that no further improvement is possible.
    Converged { refinements: u32 },
    /// The task reported that its quality target was reached.
    Satisfied { refinements: u32 },
}

impl Default for Progress {
    fn default() -> Self {
        Self::RefinementBudgetExhausted { refinements: 0 }
    }
}

/// Payload wrapping the task's current best value together with the
/// [`Progress`] marker the runtime observed.
///
/// This is a plain `CuMsgPayload`, so exact-output replay handles it via the
/// normal recorded-log path with no anytime-specific code.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(opaque, from_reflect = false, no_field_bounds)]
#[serde(bound(
    serialize = "O: Serialize",
    deserialize = "O: serde::de::DeserializeOwned"
))]
pub struct AnytimeOutput<O: CuMsgPayload> {
    pub value: O,
    pub progress: Progress,
}

/// User-facing trait for anytime tasks.
///
/// Implementors provide a cheap `base` computation that always fits within any
/// plausible budget, a bounded `refine` step that monotonically improves
/// quality. The runtime wraps the
/// implementor in an [`AnytimeTask`] adapter which handles the refinement
/// budget clock.
///
/// Large or accelerator-produced outputs can use
/// [`CuHandle<T>`](crate::pool::CuHandle) as `type Output` so `write_best` can
/// commit into a pooled cell without copying the payload.
pub trait Anytime: Freezable + Reflect {
    type Input: CuMsgPayload;
    type Output: CuMsgPayload;
    type Resources<'r>;

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized;

    /// Base computation: the basic result for the anytime algorithm.
    ///
    /// Cheap, MUST always fit within any plausible budget, produces a safe
    /// rough result. Reads `input` and resets per-cycle scratch on `self`.
    /// Return [`Step::Continue`] when refinement can proceed, or an early-exit
    /// status when the base result already converged or met its quality target.
    /// After this returns, [`Self::write_best`] is valid.
    fn base(&mut self, input: &Self::Input, ctx: &CuContext) -> CuResult<Step>;

    /// One bounded refinement increment, sized so overshoot is at most one
    /// step. MUST NOT allocate on the RT path. Signature deliberately takes no
    /// `ctx` — `refine` must be pure w.r.t. wall time; the adapter checks the
    /// deadline between calls.
    ///
    /// `input` is re-passed each call so implementors never need to cache it
    /// on `self` between the adapter's `base`/`refine` calls — the copperlist
    /// already owns it for the whole cycle.
    fn refine(&mut self, input: &Self::Input) -> CuResult<Step>;

    /// Write the best-so-far into `out` in place. `out` is the runtime-owned
    /// slot, preserved across cycles, so implementations should reuse its
    /// allocations (for example, clear a `Vec` and extend it in place).
    ///
    /// The mutable receiver and fallible result also cover bounded accelerator
    /// readback without adding an anytime-specific lifecycle hook. This method
    /// runs after the refinement loop ends and must complete promptly.
    fn write_best(&mut self, out: &mut Self::Output) -> CuResult<()>;

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }
    fn preprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }
    fn postprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }
    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }
}

/// Adapter that turns any `A: Anytime` into a `CuTask` visible to
/// `#[copper_runtime]` and `cu29_derive`.
///
/// RON names it directly, e.g.:
///
/// ```ron
/// (
///     id: "planner",
///     type: "cu29::AnytimeTask<my_crate::MyPlanner>",
///     config: { "refinement_budget_us": 8000 },
/// )
/// ```
///
/// `refinement_budget_us` bounds only the window in which new refinement
/// steps may start. The base computation runs before that window, and
/// [`Anytime::write_best`] runs after it. A refinement that starts before the
/// deadline is allowed to finish after it because synchronous task code cannot
/// be safely preempted.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct AnytimeTask<A: Anytime + TypePath> {
    #[reflect(ignore)]
    inner: A,
    #[reflect(ignore)]
    refinement_budget: CuDuration,
}

impl<A: Anytime + TypePath> AnytimeTask<A> {
    fn read_refinement_budget_us(cfg: Option<&ComponentConfig>) -> CuResult<CuDuration> {
        let cfg = cfg.ok_or_else(|| {
            CuError::from(
                "AnytimeTask requires a config with 'refinement_budget_us' (u64 microseconds, > 0)",
            )
        })?;
        let raw: u64 = cfg
            .get::<u64>("refinement_budget_us")
            .map_err(|e| CuError::from(format!("AnytimeTask config 'refinement_budget_us': {e}")))?
            .ok_or_else(|| {
                CuError::from("AnytimeTask config missing required 'refinement_budget_us'")
            })?;
        if raw == 0 {
            return Err(CuError::from(
                "AnytimeTask config 'refinement_budget_us' must be > 0",
            ));
        }
        Ok(CuDuration::from_micros(raw))
    }

    fn live_progress(
        &mut self,
        input: &A::Input,
        base_step: Step,
        mut now_nanos: impl FnMut() -> u64,
    ) -> CuResult<Progress> {
        match base_step {
            Step::Converged => Ok(Progress::Converged { refinements: 0 }),
            Step::Satisfied => Ok(Progress::Satisfied { refinements: 0 }),
            Step::Continue => {
                let deadline = now_nanos().saturating_add(self.refinement_budget.as_nanos());
                let mut refinements = 0u32;
                loop {
                    if now_nanos() >= deadline {
                        return Ok(Progress::RefinementBudgetExhausted { refinements });
                    }
                    let step = self.inner.refine(input)?;
                    refinements = refinements.saturating_add(1);
                    match step {
                        Step::Continue => {}
                        Step::Converged => {
                            return Ok(Progress::Converged { refinements });
                        }
                        Step::Satisfied => {
                            return Ok(Progress::Satisfied { refinements });
                        }
                    }
                }
            }
        }
    }

    fn process_with_now(
        &mut self,
        ctx: &CuContext,
        input: &CuMsg<A::Input>,
        out: &mut CuMsg<AnytimeOutput<A::Output>>,
        now_nanos: impl FnMut() -> u64,
    ) -> CuResult<()> {
        let Some(in_payload) = input.payload() else {
            out.clear_payload();
            out.tov = Tov::None;
            return Ok(());
        };

        let base_step = self.inner.base(in_payload, ctx)?;
        let progress = self.live_progress(in_payload, base_step, now_nanos)?;

        let payload = out.payload_mut().get_or_insert_default();
        payload.progress = progress;
        self.inner.write_best(&mut payload.value)?;
        out.tov = input.tov;
        Ok(())
    }
}

impl<A: Anytime + TypePath> Freezable for AnytimeTask<A> {
    fn freeze<E: Encoder>(&self, e: &mut E) -> Result<(), EncodeError> {
        self.inner.freeze(e)
    }

    fn thaw<D: Decoder>(&mut self, d: &mut D) -> Result<(), DecodeError> {
        self.inner.thaw(d)
    }
}

impl<A: Anytime + TypePath + 'static> TypePath for AnytimeTask<A> {
    fn type_path() -> &'static str {
        #[cfg(feature = "reflect")]
        {
            use bevy_reflect::utility::GenericTypePathCell;
            static CELL: GenericTypePathCell = GenericTypePathCell::new();
            CELL.get_or_insert::<Self, _>(|| {
                format!(
                    "cu29_runtime::cutask_anytime::AnytimeTask<{}>",
                    A::type_path()
                )
            })
        }
        #[cfg(not(feature = "reflect"))]
        {
            // Fallback path: `type_name` already carries `A`, so different
            // monomorphizations return distinct static strings.
            core::any::type_name::<Self>()
        }
    }

    fn short_type_path() -> &'static str {
        #[cfg(feature = "reflect")]
        {
            use bevy_reflect::utility::GenericTypePathCell;
            static CELL: GenericTypePathCell = GenericTypePathCell::new();
            CELL.get_or_insert::<Self, _>(|| format!("AnytimeTask<{}>", A::short_type_path()))
        }
        #[cfg(not(feature = "reflect"))]
        {
            core::any::type_name::<Self>()
        }
    }

    fn type_ident() -> Option<&'static str> {
        Some("AnytimeTask")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu29_runtime")
    }

    fn module_path() -> Option<&'static str> {
        Some("cutask_anytime")
    }
}

impl<A: Anytime + GetTypeRegistration + TypePath + 'static> CuTask for AnytimeTask<A> {
    type Input<'m> = input_msg!(A::Input);
    type Output<'m> = output_msg!(AnytimeOutput<A::Output>);
    type Resources<'r> = A::Resources<'r>;

    fn register_debug_state_types(registry: &mut TypeRegistry)
    where
        Self: GetTypeRegistration + Sized,
    {
        // Debug view surfaces `&self.inner: A`, so register `A` (not the
        // opaque wrapper). In non-reflect builds `register` is a no-op.
        registry.register::<A>();
    }

    fn debug_state_type_path() -> &'static str
    where
        Self: TypePath + Sized,
    {
        // Must match what `with_debug_state` yields — `A`, not the wrapper.
        <A as TypePath>::type_path()
    }

    fn with_debug_state<R>(&self, f: impl FnOnce(&dyn Reflect) -> R) -> R
    where
        Self: Sized,
    {
        f(&self.inner)
    }

    fn new(cfg: Option<&ComponentConfig>, res: Self::Resources<'_>) -> CuResult<Self> {
        let refinement_budget = Self::read_refinement_budget_us(cfg)?;
        Ok(Self {
            inner: A::new(cfg, res)?,
            refinement_budget,
        })
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.inner.start(ctx)
    }

    fn preprocess(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.inner.preprocess(ctx)
    }

    fn postprocess(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.inner.postprocess(ctx)
    }

    fn stop(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.inner.stop(ctx)
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        out: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        self.process_with_now(ctx, input, out, || CuInstant::now().as_nanos())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;
    use core::mem::size_of;
    use core::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
    use cu29_clock::{CuTime, RobotClock};
    use spin::Mutex;

    #[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
    struct Counter {
        value: u32,
    }

    #[derive(Reflect)]
    struct TestPlanner {
        best: u32,
        refinements: u32,
    }

    impl Freezable for TestPlanner {}

    impl Anytime for TestPlanner {
        type Input = Counter;
        type Output = Counter;
        type Resources<'r> = ();

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self {
                best: 0,
                refinements: 0,
            })
        }

        fn base(&mut self, input: &Self::Input, _ctx: &CuContext) -> CuResult<Step> {
            self.best = input.value;
            self.refinements = 0;
            Ok(Step::Continue)
        }

        fn refine(&mut self, _input: &Self::Input) -> CuResult<Step> {
            self.refinements += 1;
            self.best += 1;
            Ok(Step::Continue)
        }

        fn write_best(&mut self, out: &mut Self::Output) -> CuResult<()> {
            out.value = self.best;
            Ok(())
        }
    }

    fn config(refinement_budget_us: u64) -> ComponentConfig {
        let mut config = ComponentConfig::new();
        config.set("refinement_budget_us", refinement_budget_us);
        config
    }

    fn context() -> CuContext {
        CuContext::from_clock(RobotClock::default())
    }

    fn physical_now_nanos() -> u64 {
        CuInstant::now().as_nanos()
    }

    fn expect_error<T>(result: CuResult<T>) -> CuError {
        match result {
            Ok(_) => panic!("expected an error"),
            Err(error) => error,
        }
    }

    #[test]
    fn refinement_budget_is_required() {
        let error = expect_error(AnytimeTask::<TestPlanner>::new(None, ()));
        assert!(
            format!("{error:?}").contains("refinement_budget_us"),
            "unexpected error: {error:?}"
        );
    }

    #[test]
    fn refinement_budget_must_be_positive() {
        let config = config(0);
        let error = expect_error(AnytimeTask::<TestPlanner>::new(Some(&config), ()));
        assert!(
            format!("{error:?}").contains("> 0"),
            "unexpected error: {error:?}"
        );
    }

    #[test]
    fn old_budget_name_has_no_compatibility_alias() {
        let mut config = ComponentConfig::new();
        config.set("budget_us", 1_000u64);
        let error = expect_error(AnytimeTask::<TestPlanner>::new(Some(&config), ()));
        assert!(
            format!("{error:?}").contains("missing required 'refinement_budget_us'"),
            "unexpected error: {error:?}"
        );
    }

    #[test]
    fn old_missing_input_policy_is_ignored_and_output_is_cleared() {
        let mut config = config(1_000);
        config.set("on_missing_input", String::from("fail"));
        let mut task = AnytimeTask::<TestPlanner>::new(Some(&config), ()).unwrap();
        let mut output = CuMsg::new(Some(AnytimeOutput {
            value: Counter { value: 42 },
            progress: Progress::Converged { refinements: 3 },
        }));
        output.tov = Tov::Time(CuTime::from_nanos(99));

        task.process_with_now(
            &context(),
            &CuMsg::new(None),
            &mut output,
            physical_now_nanos,
        )
        .unwrap();

        assert!(output.payload().is_none());
        assert_eq!(output.tov, Tov::None);
    }

    #[test]
    fn missing_input_always_clears_payload_and_tov() {
        let config = config(1_000);
        let mut task = AnytimeTask::<TestPlanner>::new(Some(&config), ()).unwrap();
        let mut output = CuMsg::new(Some(AnytimeOutput {
            value: Counter { value: 7 },
            progress: Progress::Satisfied { refinements: 2 },
        }));
        output.tov = Tov::Time(CuTime::from_nanos(123));

        task.process_with_now(
            &context(),
            &CuMsg::new(None),
            &mut output,
            physical_now_nanos,
        )
        .unwrap();

        assert!(output.payload().is_none());
        assert_eq!(output.tov, Tov::None);
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Phase {
        Base,
        Clock,
        Refine,
        WriteBest,
    }

    #[derive(Reflect)]
    #[reflect(from_reflect = false)]
    struct PhasePlanner {
        #[reflect(ignore)]
        phases: Arc<Mutex<Vec<Phase>>>,
        #[reflect(ignore)]
        physical_nanos: Arc<AtomicU64>,
        refinement_nanos: u64,
        refinements: u32,
        base_step: Step,
    }

    impl Freezable for PhasePlanner {}

    impl Anytime for PhasePlanner {
        type Input = Counter;
        type Output = Counter;
        type Resources<'r> = (Arc<Mutex<Vec<Phase>>>, Arc<AtomicU64>, u64, Step);

        fn new(
            _config: Option<&ComponentConfig>,
            resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            let (phases, physical_nanos, refinement_nanos, base_step) = resources;
            Ok(Self {
                phases,
                physical_nanos,
                refinement_nanos,
                refinements: 0,
                base_step,
            })
        }

        fn base(&mut self, _input: &Self::Input, _ctx: &CuContext) -> CuResult<Step> {
            self.phases.lock().push(Phase::Base);
            self.refinements = 0;
            Ok(self.base_step)
        }

        fn refine(&mut self, _input: &Self::Input) -> CuResult<Step> {
            self.phases.lock().push(Phase::Refine);
            self.refinements += 1;
            self.physical_nanos
                .fetch_add(self.refinement_nanos, Ordering::Relaxed);
            Ok(Step::Continue)
        }

        fn write_best(&mut self, out: &mut Self::Output) -> CuResult<()> {
            self.phases.lock().push(Phase::WriteBest);
            out.value = self.refinements;
            Ok(())
        }
    }

    #[test]
    fn refinement_window_starts_after_base_and_ends_before_write_best() {
        let config = config(10);
        let phases = Arc::new(Mutex::new(Vec::new()));
        let physical_nanos = Arc::new(AtomicU64::new(0));
        let mut task = AnytimeTask::<PhasePlanner>::new(
            Some(&config),
            (
                Arc::clone(&phases),
                Arc::clone(&physical_nanos),
                10_000,
                Step::Continue,
            ),
        )
        .unwrap();
        let clock_phases = Arc::clone(&phases);
        let clock_nanos = Arc::clone(&physical_nanos);
        let mut output = CuMsg::new(None);

        task.process_with_now(
            &context(),
            &CuMsg::new(Some(Counter::default())),
            &mut output,
            move || {
                clock_phases.lock().push(Phase::Clock);
                clock_nanos.load(Ordering::Relaxed)
            },
        )
        .unwrap();

        assert_eq!(
            phases.lock().as_slice(),
            [
                Phase::Base,
                Phase::Clock,
                Phase::Clock,
                Phase::Refine,
                Phase::Clock,
                Phase::WriteBest,
            ]
        );
        assert_eq!(physical_nanos.load(Ordering::Relaxed), 10_000);
        let payload = output.payload().unwrap();
        assert_eq!(
            payload.progress,
            Progress::RefinementBudgetExhausted { refinements: 1 }
        );
        assert_eq!(payload.value.value, 1);
    }

    #[test]
    fn refinement_started_before_deadline_may_finish_after_it() {
        let config = config(10);
        let phases = Arc::new(Mutex::new(Vec::new()));
        let physical_nanos = Arc::new(AtomicU64::new(0));
        let mut task = AnytimeTask::<PhasePlanner>::new(
            Some(&config),
            (
                Arc::clone(&phases),
                Arc::clone(&physical_nanos),
                50_000,
                Step::Continue,
            ),
        )
        .unwrap();
        let clock_nanos = Arc::clone(&physical_nanos);
        let mut output = CuMsg::new(None);

        task.process_with_now(
            &context(),
            &CuMsg::new(Some(Counter::default())),
            &mut output,
            move || clock_nanos.load(Ordering::Relaxed),
        )
        .unwrap();

        assert_eq!(physical_nanos.load(Ordering::Relaxed), 50_000);
        assert_eq!(
            output.payload().unwrap().progress,
            Progress::RefinementBudgetExhausted { refinements: 1 }
        );
    }

    #[test]
    fn terminal_base_does_not_start_refinement_window() {
        let config = config(10);
        let phases = Arc::new(Mutex::new(Vec::new()));
        let physical_nanos = Arc::new(AtomicU64::new(0));
        let mut task = AnytimeTask::<PhasePlanner>::new(
            Some(&config),
            (Arc::clone(&phases), physical_nanos, 0, Step::Converged),
        )
        .unwrap();
        let mut output = CuMsg::new(None);

        task.process_with_now(
            &context(),
            &CuMsg::new(Some(Counter::default())),
            &mut output,
            || panic!("terminal base must not read the physical clock"),
        )
        .unwrap();

        assert_eq!(phases.lock().as_slice(), [Phase::Base, Phase::WriteBest]);
        assert_eq!(
            output.payload().unwrap().progress,
            Progress::Converged { refinements: 0 }
        );
    }

    static OUTPUT_CLONES: AtomicUsize = AtomicUsize::new(0);

    #[derive(Default, Debug, Encode, Decode, Serialize, Deserialize, Reflect)]
    struct CloneTrackedOutput {
        value: u32,
    }

    impl Clone for CloneTrackedOutput {
        fn clone(&self) -> Self {
            OUTPUT_CLONES.fetch_add(1, Ordering::Relaxed);
            Self { value: self.value }
        }
    }

    #[derive(Reflect)]
    struct ClonePlanner;

    impl Freezable for ClonePlanner {}

    impl Anytime for ClonePlanner {
        type Input = Counter;
        type Output = CloneTrackedOutput;
        type Resources<'r> = ();

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self)
        }

        fn base(&mut self, _input: &Self::Input, _ctx: &CuContext) -> CuResult<Step> {
            Ok(Step::Converged)
        }

        fn refine(&mut self, _input: &Self::Input) -> CuResult<Step> {
            Ok(Step::Converged)
        }

        fn write_best(&mut self, out: &mut Self::Output) -> CuResult<()> {
            out.value = 7;
            Ok(())
        }
    }

    #[test]
    fn successful_processing_does_not_clone_output() {
        let config = config(1);
        let mut task = AnytimeTask::<ClonePlanner>::new(Some(&config), ()).unwrap();
        let mut output = CuMsg::new(None);
        OUTPUT_CLONES.store(0, Ordering::Relaxed);

        task.process_with_now(
            &context(),
            &CuMsg::new(Some(Counter::default())),
            &mut output,
            physical_now_nanos,
        )
        .unwrap();

        assert_eq!(OUTPUT_CLONES.load(Ordering::Relaxed), 0);
        assert_eq!(output.payload().unwrap().value.value, 7);
    }

    #[test]
    fn adapter_size_does_not_include_output_storage() {
        assert_eq!(
            size_of::<AnytimeTask<ClonePlanner>>(),
            size_of::<CuDuration>()
        );
    }

    #[derive(Reflect)]
    struct StatefulPlanner {
        state: u32,
    }

    impl Freezable for StatefulPlanner {
        fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
            self.state.encode(encoder)
        }

        fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
            self.state = Decode::decode(decoder)?;
            Ok(())
        }
    }

    impl Anytime for StatefulPlanner {
        type Input = Counter;
        type Output = Counter;
        type Resources<'r> = ();

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self { state: 0 })
        }

        fn base(&mut self, _input: &Self::Input, _ctx: &CuContext) -> CuResult<Step> {
            Ok(Step::Converged)
        }

        fn refine(&mut self, _input: &Self::Input) -> CuResult<Step> {
            Ok(Step::Converged)
        }

        fn write_best(&mut self, out: &mut Self::Output) -> CuResult<()> {
            out.value = self.state;
            Ok(())
        }
    }

    #[test]
    fn freeze_and_thaw_only_forward_to_inner_task() {
        use crate::cutask::BincodeAdapter;
        use bincode::de::DecoderImpl;
        use bincode::de::read::SliceReader;

        let config = config(1_000);
        let mut source = AnytimeTask::<StatefulPlanner>::new(Some(&config), ()).unwrap();
        source.inner.state = 91;
        let adapter_bytes =
            bincode::encode_to_vec(BincodeAdapter(&source), bincode::config::standard()).unwrap();
        let inner_bytes =
            bincode::encode_to_vec(BincodeAdapter(&source.inner), bincode::config::standard())
                .unwrap();
        assert_eq!(adapter_bytes, inner_bytes);

        let mut destination = AnytimeTask::<StatefulPlanner>::new(Some(&config), ()).unwrap();
        let mut decoder = DecoderImpl::new(
            SliceReader::new(&adapter_bytes),
            bincode::config::standard(),
            (),
        );
        destination.thaw(&mut decoder).unwrap();
        assert_eq!(destination.inner.state, 91);
    }

    #[cfg(feature = "reflect")]
    #[test]
    fn type_path_is_per_inner_monomorphization() {
        let first = <AnytimeTask<TestPlanner> as TypePath>::type_path();
        let second = <AnytimeTask<ClonePlanner> as TypePath>::type_path();
        assert_ne!(first, second);
        assert!(first.contains("TestPlanner"), "unexpected: {first}");
        assert!(second.contains("ClonePlanner"), "unexpected: {second}");
        assert_eq!(
            <AnytimeTask<TestPlanner> as CuTask>::debug_state_type_path(),
            <TestPlanner as TypePath>::type_path()
        );
    }
}
