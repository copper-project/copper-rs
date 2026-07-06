//! Anytime tasks: interruptible computations that produce a usable base result
//! and monotonically refine it until a per-cycle budget expires.
//!
//! The [`Anytime`] trait is the user-facing interface; [`AnytimeTask`] adapts
//! any `A: Anytime` into a plain [`CuTask`] the runtime can schedule. Each
//! emitted [`AnytimeOutput`] carries a [`Progress`] marker, and the
//! [`OnOverload`] policy governs cycles that cannot produce a fresh output.

use crate::config::ComponentConfig;
use crate::context::CuContext;
use crate::cutask::{CuMsg, CuMsgPayload, CuTask, Freezable};
use crate::reflect::{GetTypeRegistration, Reflect, TypePath, TypeRegistry};
use crate::{input_msg, output_msg};

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29_clock::{CuDuration, Tov};
use cu29_traits::{CuError, CuResult};
use serde::{Deserialize, Serialize};

use alloc::format;
use alloc::vec::Vec;

/// Outcome of a single refinement step.
///
/// `Continue` means the task can still improve if given more time. The two
/// early-exit variants are semantically distinct and are logged separately in
/// [`Progress`] so operators can tell them apart:
///
/// - `Converged` — no further improvement is *possible* (algorithmic fixpoint).
/// - `Satisfied` — no further improvement is *needed* (task-defined quality
///   target reached). Improvement may still be possible, but the task judges
///   it not worth the cycles. See also [`Anytime::is_satisfied`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    Continue,
    Converged,
    Satisfied,
}

/// Quality marker emitted alongside every `AnytimeOutput`.
///
/// Downstream tasks can gate on this. `Skipped` carries the reason the adapter
/// did not run this cycle, so consumers can distinguish "no input" from
/// "reused prior best" from a normally converged result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub enum Progress {
    /// Adapter ran `base` and then `refine` `done` times before the budget
    /// expired.
    Passes { done: u32 },
    /// `refine` returned [`Step::Converged`]; no further improvement is
    /// available.
    Converged,
    /// `refine` returned [`Step::Satisfied`], or [`Anytime::is_satisfied`]
    /// reported the quality target had been met. Distinct from `Converged`
    /// because improvement may still be possible — the task chose to stop.
    Satisfied,
    /// Adapter did not run this cycle; see `SkipReason` for why.
    Skipped { reason: SkipReason },
}

impl Default for Progress {
    fn default() -> Self {
        Self::Passes { done: 0 }
    }
}

/// Reason the adapter skipped a cycle (see [`OnOverload::ReuseLast`]).
#[derive(
    Debug, Default, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub enum SkipReason {
    /// No input payload was present on the current cycle.
    #[default]
    NoInput,
    /// Adapter re-emitted the previous cycle's `best()` because the current
    /// cycle would have overshot the budget or had no input to run against.
    ReusedLast,
}

/// Payload wrapping the task's current `best()` value together with the
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

/// Policy applied when the current cycle cannot produce a fresh output:
/// either the previous output is reused, or the adapter returns an error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OnOverload {
    /// Re-emit the previous cycle's `best()` with the prior `tov` so downstream
    /// can detect staleness. `progress` becomes
    /// `Progress::Skipped { reason: ReusedLast }` (or `NoInput` if there is
    /// also no cached previous output).
    #[default]
    ReuseLast,
    /// Return a `CuError` from `process`; the existing copper-rs error path
    /// takes over.
    Fail,
}

impl OnOverload {
    fn parse(s: &str) -> CuResult<Self> {
        match s {
            "reuse_last" | "ReuseLast" => Ok(Self::ReuseLast),
            "fail" | "Fail" => Ok(Self::Fail),
            other => Err(CuError::from(format!(
                "AnytimeTask: invalid on_overload value '{other}' (expected 'reuse_last' or 'fail')"
            ))),
        }
    }
}

/// User-facing trait for anytime tasks.
///
/// Implementors provide a cheap `base` computation that always fits within any
/// plausible budget, a bounded `refine` step that monotonically improves
/// quality, plus cheap `best`/`progress` accessors. The runtime wraps the
/// implementor in an [`AnytimeTask`] adapter which handles the budget clock
/// and the overload/skip policy.
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
    /// After this returns, `best()` and `progress()` are valid.
    fn base(&mut self, input: &Self::Input, ctx: &CuContext) -> CuResult<()>;

    /// One bounded refinement increment, sized so overshoot is at most one
    /// step. MUST NOT allocate on the RT path. Signature deliberately takes no
    /// `ctx` — `refine` must be pure w.r.t. wall time; the adapter checks the
    /// deadline between calls.
    fn refine(&mut self) -> CuResult<Step>;

    /// Best-so-far. Cheap. Valid after `base` and after every `refine`.
    fn best(&self) -> Self::Output;

    /// Quality marker logged alongside the output. Cheap.
    fn progress(&self) -> Progress;

    /// Task-defined "quality target reached" predicate, checked by the
    /// adapter between refines. Return `true` to exit the refine loop with
    /// [`Progress::Satisfied`] without running another `refine`.
    ///
    /// MUST be cheap and MUST NOT allocate. Default: never satisfied.
    ///
    /// Use this when the satisfaction condition is cheaper to check than a
    /// full refine step (e.g. a cost delta cached by the last `refine`). If
    /// checking is only meaningful *after* another refine ran, return
    /// [`Step::Satisfied`] from `refine` itself instead.
    fn is_satisfied(&self) -> bool {
        false
    }

    /// Called by the adapter once per cycle after the refine loop exits (via
    /// deadline, `Converged`, `Satisfied`, or `is_satisfied`) and before
    /// `best()` is read. Default no-op.
    ///
    /// This is the drain point for tasks that dispatch asynchronous work
    /// inside `refine` (e.g. accelerator kernels): `finalize` waits for the
    /// last in-flight stage and commits its readback so `best()` is coherent.
    /// The wait must be bounded — the caller has already spent the cycle
    /// budget, so `finalize` should complete promptly (typically one
    /// last-dispatched stage's worth of work).
    ///
    /// Not called on skipped cycles (no input, or overload reuse) — nothing
    /// was dispatched, so there is nothing to drain.
    fn finalize(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }

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

/// Cached copy of the last emitted output, used to serve
/// [`OnOverload::ReuseLast`] on skipped cycles.
struct CachedOutput<O: CuMsgPayload> {
    value: O,
    tov: Tov,
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
///     config: { "budget_us": 8000, "on_overload": "reuse_last" },
/// )
/// ```
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct AnytimeTask<A: Anytime + TypePath> {
    #[reflect(ignore)]
    inner: A,
    #[reflect(ignore)]
    budget: CuDuration,
    #[reflect(ignore)]
    on_overload: OnOverload,
    #[reflect(ignore)]
    last: Option<CachedOutput<A::Output>>,
}

impl<A: Anytime + TypePath> AnytimeTask<A> {
    // Cross-field validation against `runtime.rate_target_hz` runs one layer
    // up in `CuConfig::validate_anytime_budgets` — this only enforces the
    // per-node invariants.
    fn read_budget_us(cfg: Option<&ComponentConfig>) -> CuResult<CuDuration> {
        let cfg = cfg.ok_or_else(|| {
            CuError::from("AnytimeTask requires a config with 'budget_us' (u64 microseconds, > 0)")
        })?;
        let raw: u64 = cfg
            .get::<u64>("budget_us")
            .map_err(|e| CuError::from(format!("AnytimeTask config 'budget_us': {e}")))?
            .ok_or_else(|| CuError::from("AnytimeTask config missing required 'budget_us'"))?;
        if raw == 0 {
            return Err(CuError::from("AnytimeTask config 'budget_us' must be > 0"));
        }
        Ok(CuDuration::from_micros(raw))
    }

    fn read_on_overload(cfg: Option<&ComponentConfig>) -> CuResult<OnOverload> {
        let Some(cfg) = cfg else {
            return Ok(OnOverload::default());
        };
        match cfg
            .get::<String>("on_overload")
            .map_err(|e| CuError::from(format!("AnytimeTask config 'on_overload': {e}")))?
        {
            Some(s) => OnOverload::parse(&s),
            None => Ok(OnOverload::default()),
        }
    }

    fn cache_last(&mut self, value: &A::Output, tov: Tov) {
        self.last = Some(CachedOutput {
            value: value.clone(),
            tov,
        });
    }

    fn emit_skip(
        &mut self,
        out: &mut CuMsg<AnytimeOutput<A::Output>>,
        reason: SkipReason,
    ) -> CuResult<()> {
        match (self.on_overload, self.last.as_ref()) {
            (OnOverload::Fail, _) => Err(CuError::from(format!(
                "AnytimeTask overload with on_overload=fail (reason: {reason:?})"
            ))),
            (OnOverload::ReuseLast, Some(cached)) => {
                out.set_payload(AnytimeOutput {
                    value: cached.value.clone(),
                    progress: Progress::Skipped {
                        reason: SkipReason::ReusedLast,
                    },
                });
                out.tov = cached.tov;
                Ok(())
            }
            (OnOverload::ReuseLast, None) => {
                out.set_payload(AnytimeOutput {
                    value: A::Output::default(),
                    progress: Progress::Skipped { reason },
                });
                out.tov = Tov::None;
                Ok(())
            }
        }
    }
}

impl<A: Anytime + TypePath> Freezable for AnytimeTask<A> {
    fn freeze<E: Encoder>(&self, e: &mut E) -> Result<(), EncodeError> {
        self.inner.freeze(e)?;
        // Persist `last` so exact-output replay reproduces the `ReuseLast`
        // payload when the recorded cycle skipped. The outer `Decoder::Context`
        // is unconstrained by `Freezable`, so `A::Output` goes through as a
        // length-prefixed blob under the standard bincode config, decoupling
        // from the caller's Decoder context.
        match &self.last {
            None => Encode::encode(&false, e),
            Some(cached) => {
                Encode::encode(&true, e)?;
                let value_bytes =
                    bincode::encode_to_vec(&cached.value, bincode::config::standard()).map_err(
                        |_| EncodeError::Other("AnytimeTask: failed to encode cached output"),
                    )?;
                Encode::encode(&value_bytes, e)?;
                Encode::encode(&cached.tov, e)
            }
        }
    }

    fn thaw<D: Decoder>(&mut self, d: &mut D) -> Result<(), DecodeError> {
        self.inner.thaw(d)?;
        let has_cache: bool = Decode::decode(d)?;
        self.last = if has_cache {
            let value_bytes: Vec<u8> = Decode::decode(d)?;
            let (value, _read) = bincode::decode_from_slice::<A::Output, _>(
                &value_bytes,
                bincode::config::standard(),
            )
            .map_err(|_| DecodeError::Other("AnytimeTask: failed to decode cached output"))?;
            let tov: Tov = Decode::decode(d)?;
            Some(CachedOutput { value, tov })
        } else {
            None
        };
        Ok(())
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
        let budget = Self::read_budget_us(cfg)?;
        let on_overload = Self::read_on_overload(cfg)?;
        Ok(Self {
            inner: A::new(cfg, res)?,
            budget,
            on_overload,
            last: None,
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
        let Some(in_payload) = input.payload() else {
            return self.emit_skip(out, SkipReason::NoInput);
        };

        let start = ctx.clock.now();
        let deadline = start + self.budget;

        self.inner.base(in_payload, ctx)?;
        let progress = loop {
            if ctx.clock.now() >= deadline {
                break self.inner.progress();
            }
            if self.inner.is_satisfied() {
                break Progress::Satisfied;
            }
            match self.inner.refine()? {
                Step::Continue => continue,
                Step::Converged => break Progress::Converged,
                Step::Satisfied => break Progress::Satisfied,
            }
        };

        self.inner.finalize(ctx)?;
        let value = self.inner.best();
        let tov = Tov::Time(ctx.clock.now());
        self.cache_last(&value, tov);
        out.set_payload(AnytimeOutput { value, progress });
        out.tov = tov;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Value;
    use cu29_clock::RobotClock;

    // Deterministic anytime task used to exercise the adapter: `base` seeds
    // `best_val` from the input; each `refine` bumps the counter up to
    // `max_passes`, then reports `Converged`.
    #[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
    struct Counter {
        value: u32,
    }

    #[derive(Reflect)]
    struct TestPlanner {
        best_val: u32,
        pass_count: u32,
        max_passes: u32,
        // When Some(n), `refine` returns `Step::Satisfied` once `pass_count >= n`.
        satisfy_after: Option<u32>,
        // When Some(n), `is_satisfied` returns true once `pass_count >= n`.
        // Checked by the adapter between refines.
        is_satisfied_after: Option<u32>,
        // Incremented every time `finalize` is called.
        finalize_calls: u32,
    }

    impl Freezable for TestPlanner {}

    impl Anytime for TestPlanner {
        type Input = Counter;
        type Output = Counter;
        type Resources<'r> = ();

        fn new(_cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self> {
            Ok(Self {
                best_val: 0,
                pass_count: 0,
                max_passes: 1_000,
                satisfy_after: None,
                is_satisfied_after: None,
                finalize_calls: 0,
            })
        }

        fn base(&mut self, input: &Self::Input, _ctx: &CuContext) -> CuResult<()> {
            self.best_val = input.value;
            self.pass_count = 0;
            Ok(())
        }

        fn refine(&mut self) -> CuResult<Step> {
            if self.pass_count >= self.max_passes {
                return Ok(Step::Converged);
            }
            self.pass_count += 1;
            self.best_val += 1;
            if let Some(n) = self.satisfy_after
                && self.pass_count >= n
            {
                return Ok(Step::Satisfied);
            }
            Ok(Step::Continue)
        }

        fn best(&self) -> Self::Output {
            Counter {
                value: self.best_val,
            }
        }

        fn progress(&self) -> Progress {
            Progress::Passes {
                done: self.pass_count,
            }
        }

        fn is_satisfied(&self) -> bool {
            match self.is_satisfied_after {
                Some(n) => self.pass_count >= n,
                None => false,
            }
        }

        fn finalize(&mut self, _ctx: &CuContext) -> CuResult<()> {
            self.finalize_calls += 1;
            Ok(())
        }
    }

    fn cfg_with_budget(budget_us: u64) -> ComponentConfig {
        let mut c = ComponentConfig::new();
        c.set("budget_us", budget_us);
        c
    }

    fn expect_err<T>(r: CuResult<T>) -> CuError {
        match r {
            Ok(_) => panic!("expected an error"),
            Err(e) => e,
        }
    }

    #[test]
    fn rejects_missing_budget() {
        let err = expect_err(AnytimeTask::<TestPlanner>::new(None, ()));
        let msg = format!("{err:?}");
        assert!(msg.contains("budget_us"), "unexpected error: {msg}");
    }

    #[test]
    fn rejects_zero_budget() {
        let cfg = cfg_with_budget(0);
        let err = expect_err(AnytimeTask::<TestPlanner>::new(Some(&cfg), ()));
        let msg = format!("{err:?}");
        assert!(msg.contains("> 0"), "unexpected error: {msg}");
    }

    #[test]
    fn parses_on_overload() {
        let mut cfg = cfg_with_budget(1_000);
        cfg.set("on_overload", "fail".to_string());
        let t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        assert_eq!(t.on_overload, OnOverload::Fail);

        let mut cfg = cfg_with_budget(1_000);
        cfg.set("on_overload", "reuse_last".to_string());
        let t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        assert_eq!(t.on_overload, OnOverload::ReuseLast);
    }

    #[test]
    fn on_overload_defaults_to_reuse_last() {
        let cfg = cfg_with_budget(1_000);
        let t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        assert_eq!(t.on_overload, OnOverload::ReuseLast);
    }

    #[test]
    fn rejects_unknown_on_overload() {
        let mut cfg = cfg_with_budget(1_000);
        cfg.set("on_overload", "wat".to_string());
        let err = expect_err(AnytimeTask::<TestPlanner>::new(Some(&cfg), ()));
        let msg = format!("{err:?}");
        assert!(msg.contains("wat"), "unexpected error: {msg}");
    }

    #[test]
    fn adapter_runs_base_then_refines_until_deadline() {
        let cfg = cfg_with_budget(1_000);
        let mut t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();

        let (clock, mock) = RobotClock::mock();
        mock.set_value(0);
        let ctx = CuContext::from_clock(clock);

        let input: CuMsg<Counter> = CuMsg::new(Some(Counter { value: 5 }));
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);

        // Mock clock never advances, so the loop terminates on Converged when
        // `pass_count == max_passes`.
        t.inner.max_passes = 10;
        t.process(&ctx, &input, &mut out).unwrap();

        let payload = out.payload().expect("output must be set");
        assert_eq!(payload.progress, Progress::Converged);
        assert_eq!(payload.value.value, 5 + 10);
    }

    #[test]
    fn adapter_emits_skip_no_input() {
        let cfg = cfg_with_budget(1_000);
        let mut t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, _mock) = RobotClock::mock();
        let ctx = CuContext::from_clock(clock);

        let input: CuMsg<Counter> = CuMsg::new(None);
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input, &mut out).unwrap();

        let payload = out.payload().expect("output must be set");
        assert_eq!(
            payload.progress,
            Progress::Skipped {
                reason: SkipReason::NoInput
            }
        );
    }

    #[test]
    fn adapter_reuses_last_on_second_skip() {
        let cfg = cfg_with_budget(1_000);
        let mut t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, mock) = RobotClock::mock();
        mock.set_value(100);
        let ctx = CuContext::from_clock(clock);

        t.inner.max_passes = 3;
        // First cycle: produces a real output, populates the cache.
        let input1: CuMsg<Counter> = CuMsg::new(Some(Counter { value: 10 }));
        let mut out1: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input1, &mut out1).unwrap();
        let first_tov = out1.tov;
        let first_value = out1.payload().unwrap().value.value;
        assert_eq!(first_value, 13);

        // Second cycle: no input → adapter re-emits cached best with prior tov.
        mock.set_value(500);
        let input2: CuMsg<Counter> = CuMsg::new(None);
        let mut out2: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input2, &mut out2).unwrap();

        let p2 = out2.payload().expect("reuse_last must emit a payload");
        assert_eq!(
            p2.progress,
            Progress::Skipped {
                reason: SkipReason::ReusedLast
            }
        );
        assert_eq!(p2.value.value, first_value);
        assert_eq!(out2.tov, first_tov, "reused output must carry prior tov");
    }

    #[test]
    fn adapter_fails_on_overload_when_configured() {
        let mut cfg = cfg_with_budget(1_000);
        cfg.set("on_overload", "fail".to_string());
        let mut t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, _mock) = RobotClock::mock();
        let ctx = CuContext::from_clock(clock);

        let input: CuMsg<Counter> = CuMsg::new(None);
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        let err = t.process(&ctx, &input, &mut out).unwrap_err();
        let msg = format!("{err:?}");
        assert!(msg.contains("fail"), "unexpected error: {msg}");
    }

    // `Step::Satisfied` returned from `refine` exits the loop and is logged as
    // `Progress::Satisfied` — distinct from `Converged`, so operators can
    // tell "quality target reached" apart from "algorithmic fixpoint".
    #[test]
    fn adapter_reports_satisfied_when_refine_returns_satisfied() {
        let cfg = cfg_with_budget(1_000);
        let mut t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, mock) = RobotClock::mock();
        mock.set_value(0);
        let ctx = CuContext::from_clock(clock);

        // Clock frozen so the deadline never expires; convergence disabled;
        // task asks the adapter to stop after 4 refines via Step::Satisfied.
        t.inner.max_passes = 1_000_000;
        t.inner.satisfy_after = Some(4);

        let input: CuMsg<Counter> = CuMsg::new(Some(Counter { value: 10 }));
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input, &mut out).unwrap();

        let payload = out.payload().expect("output must be set");
        assert_eq!(payload.progress, Progress::Satisfied);
        assert_eq!(payload.value.value, 10 + 4);
    }

    // `is_satisfied` returning true between refines exits the loop with
    // `Progress::Satisfied` without running another refine — the "cheap
    // predicate" path documented on the trait.
    #[test]
    fn adapter_reports_satisfied_when_is_satisfied_predicate_fires() {
        let cfg = cfg_with_budget(1_000);
        let mut t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, mock) = RobotClock::mock();
        mock.set_value(0);
        let ctx = CuContext::from_clock(clock);

        t.inner.max_passes = 1_000_000;
        // After 3 refines the adapter's next top-of-loop check sees
        // is_satisfied() == true and exits — pass_count must be exactly 3.
        t.inner.is_satisfied_after = Some(3);

        let input: CuMsg<Counter> = CuMsg::new(Some(Counter { value: 100 }));
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input, &mut out).unwrap();

        let payload = out.payload().expect("output must be set");
        assert_eq!(payload.progress, Progress::Satisfied);
        assert_eq!(payload.value.value, 103);
    }

    // `finalize` is called exactly once per non-skipped cycle, before `best()`
    // is read. This is the async-drain contract accelerator tasks rely on.
    #[test]
    fn adapter_calls_finalize_exactly_once_per_processed_cycle() {
        let cfg = cfg_with_budget(1_000);
        let mut t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, mock) = RobotClock::mock();
        mock.set_value(0);
        let ctx = CuContext::from_clock(clock);
        t.inner.max_passes = 5;

        let input: CuMsg<Counter> = CuMsg::new(Some(Counter { value: 0 }));
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input, &mut out).unwrap();
        assert_eq!(t.inner.finalize_calls, 1);

        // A second normal cycle bumps the counter to 2 — one call per cycle.
        let input: CuMsg<Counter> = CuMsg::new(Some(Counter { value: 0 }));
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input, &mut out).unwrap();
        assert_eq!(t.inner.finalize_calls, 2);
    }

    // `finalize` must NOT be called on skip paths — nothing was dispatched, so
    // there is nothing to drain. Documented on the trait.
    #[test]
    fn adapter_does_not_call_finalize_on_skip_paths() {
        let cfg = cfg_with_budget(1_000);
        let mut t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, _mock) = RobotClock::mock();
        let ctx = CuContext::from_clock(clock);

        // NoInput skip.
        let input: CuMsg<Counter> = CuMsg::new(None);
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input, &mut out).unwrap();
        assert_eq!(t.inner.finalize_calls, 0);

        // ReuseLast skip after a real cycle: the real cycle bumps finalize
        // once; the skipped cycle must not.
        let input: CuMsg<Counter> = CuMsg::new(Some(Counter { value: 1 }));
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input, &mut out).unwrap();
        assert_eq!(t.inner.finalize_calls, 1);

        let input: CuMsg<Counter> = CuMsg::new(None);
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input, &mut out).unwrap();
        assert_eq!(t.inner.finalize_calls, 1, "skip must not call finalize");
    }

    // Cached `last` must survive freeze/thaw so a keyframe-based replay can
    // reproduce a `Progress::Skipped { reason: ReusedLast }` payload.
    #[test]
    fn freeze_thaw_round_trips_cached_last() {
        use crate::cutask::BincodeAdapter;
        use bincode::de::DecoderImpl;
        use bincode::de::read::SliceReader;

        let cfg = cfg_with_budget(1_000);
        let mut src = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, mock) = RobotClock::mock();
        mock.set_value(42);
        let ctx = CuContext::from_clock(clock);

        src.inner.max_passes = 3;
        let input: CuMsg<Counter> = CuMsg::new(Some(Counter { value: 7 }));
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        src.process(&ctx, &input, &mut out).unwrap();
        let expected = out.payload().unwrap().value.value;
        let expected_tov = out.tov;
        assert!(src.last.is_some(), "process must populate the cache");

        let bytes = bincode::encode_to_vec(BincodeAdapter(&src), bincode::config::standard())
            .expect("freeze must succeed");

        let mut dst = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        assert!(dst.last.is_none());
        let mut d = DecoderImpl::new(SliceReader::new(&bytes), bincode::config::standard(), ());
        Freezable::thaw(&mut dst, &mut d).expect("thaw must succeed");
        let cached = dst.last.as_ref().expect("cache must be restored");
        assert_eq!(cached.value.value, expected);
        assert_eq!(cached.tov, expected_tov);

        // And after thaw, a skipped cycle re-emits the cached payload.
        let no_input: CuMsg<Counter> = CuMsg::new(None);
        let mut skip_out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        dst.process(&ctx, &no_input, &mut skip_out).unwrap();
        let p = skip_out.payload().unwrap();
        assert_eq!(
            p.progress,
            Progress::Skipped {
                reason: SkipReason::ReusedLast
            }
        );
        assert_eq!(p.value.value, expected);
        assert_eq!(skip_out.tov, expected_tov);
    }

    // Distinct `A` monomorphizations must yield distinct `TypePath::type_path`
    // strings, otherwise the type registry would collide them.
    #[cfg(feature = "reflect")]
    #[test]
    fn type_path_is_per_inner_monomorphization() {
        #[derive(Reflect)]
        struct OtherPlanner {
            best_val: u32,
        }
        impl Freezable for OtherPlanner {}
        impl Anytime for OtherPlanner {
            type Input = Counter;
            type Output = Counter;
            type Resources<'r> = ();
            fn new(_c: Option<&ComponentConfig>, _r: Self::Resources<'_>) -> CuResult<Self> {
                Ok(Self { best_val: 0 })
            }
            fn base(&mut self, i: &Self::Input, _c: &CuContext) -> CuResult<()> {
                self.best_val = i.value;
                Ok(())
            }
            fn refine(&mut self) -> CuResult<Step> {
                Ok(Step::Converged)
            }
            fn best(&self) -> Self::Output {
                Counter {
                    value: self.best_val,
                }
            }
            fn progress(&self) -> Progress {
                Progress::Converged
            }
        }

        let a = <AnytimeTask<TestPlanner> as TypePath>::type_path();
        let b = <AnytimeTask<OtherPlanner> as TypePath>::type_path();
        assert_ne!(a, b, "TypePath::type_path must differ per inner A");
        assert!(a.contains("TestPlanner"), "unexpected: {a}");
        assert!(b.contains("OtherPlanner"), "unexpected: {b}");

        // `debug_state_type_path()` must point at the inner type — that's
        // what `with_debug_state` yields.
        assert_eq!(
            <AnytimeTask<TestPlanner> as CuTask>::debug_state_type_path(),
            <TestPlanner as TypePath>::type_path()
        );
    }

    // Silences unused imports on non-reflect builds.
    #[allow(dead_code)]
    fn _touch_value(_: Value) {}
}
