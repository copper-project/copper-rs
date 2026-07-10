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
///   it not worth the cycles.
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
    /// `refine` returned [`Step::Satisfied`], or the task's [`Anytime::progress`]
    /// reported [`Progress::Satisfied`] between refines. Distinct from
    /// `Converged` because improvement may still be possible — the task chose
    /// to stop.
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
///
/// Serialised form in RON is snake_case (`"reuse_last"`, `"fail"`) — one
/// canonical spelling, no aliases.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OnOverload {
    /// Re-emit the previous cycle's best with the prior `tov` so downstream
    /// can detect staleness. `progress` becomes
    /// [`Progress::Skipped`] `{ reason: ReusedLast }`.
    ///
    /// Cold start: if the adapter has never produced a real output yet
    /// (typically the very first cycle takes the skip path), there is no
    /// cached value to reuse. In that case the adapter emits
    /// `A::Output::default()` with `Tov::None` and preserves the *original*
    /// skip reason on `Progress::Skipped` (e.g. `NoInput`), so downstream
    /// contract code can distinguish "warm-cache reuse" (`ReusedLast`) from
    /// "cold default" (the original reason with `Tov::None`).
    #[default]
    ReuseLast,
    /// Return a `CuError` from `process`; the existing copper-rs error path
    /// takes over.
    Fail,
}

/// User-facing trait for anytime tasks.
///
/// Implementors provide a cheap `base` computation that always fits within any
/// plausible budget, a bounded `refine` step that monotonically improves
/// quality, plus cheap `best`/`progress` accessors. The runtime wraps the
/// implementor in an [`AnytimeTask`] adapter which handles the budget clock
/// and the overload/skip policy.
///
/// For large or accelerator-produced outputs, use
/// [`CuHandle<T>`](crate::pool::CuHandle) as `type Output`: `write_best`
/// becomes an Arc clone (no payload copy) and readback in `finalize` lands
/// directly in the pooled cell.
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
    /// After this returns, `write_best` and `progress` are valid.
    fn base(&mut self, input: &Self::Input, ctx: &CuContext) -> CuResult<()>;

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
    /// slot, preserved across cycles — reuse its allocations (e.g.
    /// `out.clear(); out.extend_from_slice(&self.buf)`).
    fn write_best(&self, out: &mut Self::Output);

    /// Quality marker logged alongside the output, and checked at the top of
    /// every refine iteration: any non-[`Progress::Passes`] value short-circuits
    /// the loop. Set it from `base`/`refine` for the cheap-predicate early-out;
    /// otherwise return [`Step::Satisfied`] from `refine`. Cheap, MUST NOT allocate.
    fn progress(&self) -> Progress;

    /// Called by the adapter once per cycle after the refine loop exits (via
    /// deadline, `Converged`, or `Satisfied`) and before `best()` is read.
    /// Default no-op.
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

pub(crate) struct CachedOutput<O: CuMsgPayload> {
    value: O,
    tov: Tov,
}

/// Overload policy + its cache. `ReuseLast(None)` is cold; becomes
/// `Some(cached)` after the first real cycle. `Fail` cannot cache.
pub(crate) enum OverloadState<O: CuMsgPayload> {
    Fail,
    ReuseLast(Option<CachedOutput<O>>),
}

impl<O: CuMsgPayload> OverloadState<O> {
    fn from_policy(policy: OnOverload) -> Self {
        match policy {
            OnOverload::Fail => Self::Fail,
            OnOverload::ReuseLast => Self::ReuseLast(None),
        }
    }

    /// Test-only: the configured policy, ignoring cache warmth.
    #[cfg(test)]
    pub(crate) fn policy(&self) -> OnOverload {
        match self {
            Self::Fail => OnOverload::Fail,
            Self::ReuseLast(_) => OnOverload::ReuseLast,
        }
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
    // `budget` and the policy tag inside `overload` are re-derived by `new()`
    // from config on replay, not persisted through freeze/thaw.
    #[reflect(ignore)]
    pub(crate) overload: OverloadState<A::Output>,
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
        Ok(cfg
            .get_value::<OnOverload>("on_overload")
            .map_err(|e| CuError::from(format!("AnytimeTask config 'on_overload': {e}")))?
            .unwrap_or_default())
    }

    fn emit_skip(
        &mut self,
        out: &mut CuMsg<AnytimeOutput<A::Output>>,
        reason: SkipReason,
    ) -> CuResult<()> {
        match &self.overload {
            OverloadState::Fail => Err(CuError::from(format!(
                "AnytimeTask overload with on_overload=fail (reason: {reason:?})"
            ))),
            OverloadState::ReuseLast(Some(cached)) => {
                if out.payload().is_none() {
                    out.set_payload(AnytimeOutput::default());
                }
                let payload = out
                    .payload_mut()
                    .as_mut()
                    .expect("payload set above if missing");
                payload.value.clone_from(&cached.value);
                payload.progress = Progress::Skipped {
                    reason: SkipReason::ReusedLast,
                };
                out.tov = cached.tov;
                Ok(())
            }
            OverloadState::ReuseLast(None) => {
                // Cold start: preserve original reason so downstream can
                // distinguish this from a warm-cache reuse.
                if out.payload().is_none() {
                    out.set_payload(AnytimeOutput::default());
                }
                let payload = out
                    .payload_mut()
                    .as_mut()
                    .expect("payload set above if missing");
                payload.value = A::Output::default();
                payload.progress = Progress::Skipped { reason };
                out.tov = Tov::None;
                Ok(())
            }
        }
    }
}

impl<A: Anytime + TypePath> Freezable for AnytimeTask<A> {
    // Wire: inner | reuse:bool [ has_cache:bool [ value:Vec<u8> tov ] ].
    // `value` goes through a `Vec<u8>` blob so the inner `A::Output: Decode<()>`
    // bound is satisfied regardless of the outer decoder's context.
    fn freeze<E: Encoder>(&self, e: &mut E) -> Result<(), EncodeError> {
        self.inner.freeze(e)?;
        match &self.overload {
            OverloadState::Fail => Encode::encode(&false, e),
            OverloadState::ReuseLast(None) => {
                Encode::encode(&true, e)?;
                Encode::encode(&false, e)
            }
            OverloadState::ReuseLast(Some(cached)) => {
                Encode::encode(&true, e)?;
                Encode::encode(&true, e)?;
                let value_bytes =
                    bincode::encode_to_vec(&cached.value, bincode::config::standard()).map_err(
                        |_| EncodeError::Other("AnytimeTask: failed to encode cached output"),
                    )?;
                Encode::encode(&value_bytes, e)?;
                cached.tov.encode(e)
            }
        }
    }

    fn thaw<D: Decoder>(&mut self, d: &mut D) -> Result<(), DecodeError> {
        self.inner.thaw(d)?;
        let reuse: bool = Decode::decode(d)?;
        if !reuse {
            self.overload = OverloadState::Fail;
            return Ok(());
        }
        let has_cache: bool = Decode::decode(d)?;
        self.overload = if has_cache {
            let value_bytes: Vec<u8> = Decode::decode(d)?;
            let (value, _read) = bincode::decode_from_slice::<A::Output, _>(
                &value_bytes,
                bincode::config::standard(),
            )
            .map_err(|_| DecodeError::Other("AnytimeTask: failed to decode cached output"))?;
            let tov: Tov = Decode::decode(d)?;
            OverloadState::ReuseLast(Some(CachedOutput { value, tov }))
        } else {
            OverloadState::ReuseLast(None)
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
        let policy = Self::read_on_overload(cfg)?;
        Ok(Self {
            inner: A::new(cfg, res)?,
            budget,
            overload: OverloadState::from_policy(policy),
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
            let p = self.inner.progress();
            if !matches!(p, Progress::Passes { .. }) {
                break p;
            }
            if ctx.clock.now() >= deadline {
                break p;
            }
            match self.inner.refine(in_payload)? {
                Step::Continue => continue,
                Step::Converged => break Progress::Converged,
                Step::Satisfied => break Progress::Satisfied,
            }
        };

        self.inner.finalize(ctx)?;
        let tov = Tov::Time(ctx.clock.now());

        // Mutate the slot in place so `write_best` can reuse its allocation.
        if out.payload().is_none() {
            out.set_payload(AnytimeOutput::default());
        }
        {
            let payload = out
                .payload_mut()
                .as_mut()
                .expect("payload set above if missing");
            payload.progress = progress;
            self.inner.write_best(&mut payload.value);
        }
        out.tov = tov;

        // Cache only under ReuseLast; `clone_from` reuses capacity.
        if let OverloadState::ReuseLast(slot) = &mut self.overload {
            let payload = out.payload().expect("payload written just above");
            match slot {
                Some(cached) => {
                    cached.value.clone_from(&payload.value);
                    cached.tov = tov;
                }
                None => {
                    *slot = Some(CachedOutput {
                        value: payload.value.clone(),
                        tov,
                    });
                }
            }
        }
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
        // When Some(n), `progress()` returns `Progress::Satisfied` once
        // `pass_count >= n`. The adapter checks this at the top of the refine
        // loop and short-circuits without running another refine.
        progress_satisfied_after: Option<u32>,
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
                progress_satisfied_after: None,
                finalize_calls: 0,
            })
        }

        fn base(&mut self, input: &Self::Input, _ctx: &CuContext) -> CuResult<()> {
            self.best_val = input.value;
            self.pass_count = 0;
            Ok(())
        }

        fn refine(&mut self, _input: &Self::Input) -> CuResult<Step> {
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

        fn write_best(&self, out: &mut Self::Output) {
            out.value = self.best_val;
        }

        fn progress(&self) -> Progress {
            if let Some(n) = self.progress_satisfied_after
                && self.pass_count >= n
            {
                return Progress::Satisfied;
            }
            Progress::Passes {
                done: self.pass_count,
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
        assert_eq!(t.overload.policy(), OnOverload::Fail);
        assert!(matches!(t.overload, OverloadState::Fail));

        let mut cfg = cfg_with_budget(1_000);
        cfg.set("on_overload", "reuse_last".to_string());
        let t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        assert_eq!(t.overload.policy(), OnOverload::ReuseLast);
        assert!(matches!(t.overload, OverloadState::ReuseLast(None)));
    }

    #[test]
    fn on_overload_defaults_to_reuse_last() {
        let cfg = cfg_with_budget(1_000);
        let t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        assert_eq!(t.overload.policy(), OnOverload::ReuseLast);
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

    // Cold start: on a skip cycle before any real cycle has populated the
    // cache, `ReuseLast` emits `A::Output::default()` with `Tov::None` and
    // keeps the original skip reason (here `NoInput`). This is the contract
    // downstream code relies on to tell "warm-cache reuse" apart from a
    // "cold default" emission.
    #[test]
    fn adapter_cold_start_reuse_last_emits_default_with_none_tov() {
        let cfg = cfg_with_budget(1_000);
        let mut t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, _mock) = RobotClock::mock();
        let ctx = CuContext::from_clock(clock);

        let input: CuMsg<Counter> = CuMsg::new(None);
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input, &mut out).unwrap();

        let payload = out.payload().expect("cold-start reuse_last still emits");
        assert_eq!(
            payload.progress,
            Progress::Skipped {
                reason: SkipReason::NoInput
            },
            "cold-start skip must preserve the original SkipReason, not ReusedLast"
        );
        assert_eq!(payload.value.value, u32::default());
        assert_eq!(out.tov, Tov::None);
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

    // `progress()` returning `Satisfied` between refines exits the loop
    // without running another refine — the "cheap predicate" path documented
    // on the trait.
    #[test]
    fn adapter_reports_satisfied_when_progress_reports_satisfied() {
        let cfg = cfg_with_budget(1_000);
        let mut t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, mock) = RobotClock::mock();
        mock.set_value(0);
        let ctx = CuContext::from_clock(clock);

        t.inner.max_passes = 1_000_000;
        // After 3 refines the adapter's next top-of-loop check sees
        // progress() == Satisfied and exits — pass_count must be exactly 3.
        t.inner.progress_satisfied_after = Some(3);

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
        assert!(
            matches!(&src.overload, OverloadState::ReuseLast(Some(_))),
            "process must populate the cache"
        );

        let bytes = bincode::encode_to_vec(BincodeAdapter(&src), bincode::config::standard())
            .expect("freeze must succeed");

        let mut dst = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        assert!(matches!(&dst.overload, OverloadState::ReuseLast(None)));
        let mut d = DecoderImpl::new(SliceReader::new(&bytes), bincode::config::standard(), ());
        Freezable::thaw(&mut dst, &mut d).expect("thaw must succeed");
        let cached = match &dst.overload {
            OverloadState::ReuseLast(Some(cached)) => cached,
            _ => panic!("cache must be restored"),
        };
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

    // Slot `Vec` capacity must survive across cycles (`write_best` reuse).
    #[test]
    fn process_preserves_slot_vec_capacity_across_cycles() {
        #[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
        struct BufMsg {
            data: Vec<u32>,
        }

        #[derive(Default, Reflect)]
        struct BufPlanner {
            n: u32,
        }

        impl Freezable for BufPlanner {}

        impl Anytime for BufPlanner {
            type Input = Counter;
            type Output = BufMsg;
            type Resources<'r> = ();

            fn new(_c: Option<&ComponentConfig>, _r: Self::Resources<'_>) -> CuResult<Self> {
                Ok(Self { n: 0 })
            }
            fn base(&mut self, input: &Self::Input, _ctx: &CuContext) -> CuResult<()> {
                self.n = input.value;
                Ok(())
            }
            fn refine(&mut self, _i: &Self::Input) -> CuResult<Step> {
                Ok(Step::Converged)
            }
            fn write_best(&self, out: &mut Self::Output) {
                out.data.clear();
                out.data.push(self.n);
            }
            fn progress(&self) -> Progress {
                Progress::Converged
            }
        }

        let cfg = cfg_with_budget(1_000);
        let mut t = AnytimeTask::<BufPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, _mock) = RobotClock::mock();
        let ctx = CuContext::from_clock(clock);

        // Prime with high capacity — collapses to 0 if the adapter resets.
        let mut out: CuMsg<AnytimeOutput<BufMsg>> = CuMsg::new(None);
        out.set_payload(AnytimeOutput {
            value: BufMsg {
                data: Vec::with_capacity(1024),
            },
            progress: Progress::default(),
        });

        let input: CuMsg<Counter> = CuMsg::new(Some(Counter { value: 7 }));
        t.process(&ctx, &input, &mut out).unwrap();

        let payload = out.payload().expect("output set");
        assert_eq!(payload.value.data, [7]);
        assert!(
            payload.value.data.capacity() >= 1024,
            "primed capacity must survive — got {}",
            payload.value.data.capacity()
        );
    }

    #[test]
    fn fail_mode_carries_no_cache_after_real_cycle() {
        let mut cfg = cfg_with_budget(1_000);
        cfg.set("on_overload", "fail".to_string());
        let mut t = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let (clock, mock) = RobotClock::mock();
        mock.set_value(0);
        let ctx = CuContext::from_clock(clock);
        t.inner.max_passes = 3;

        let input: CuMsg<Counter> = CuMsg::new(Some(Counter { value: 1 }));
        let mut out: CuMsg<AnytimeOutput<Counter>> = CuMsg::new(None);
        t.process(&ctx, &input, &mut out).unwrap();

        assert!(
            matches!(t.overload, OverloadState::Fail),
            "Fail mode must not switch to ReuseLast after a real cycle"
        );
    }

    #[test]
    fn freeze_thaw_round_trips_fail_mode() {
        use crate::cutask::BincodeAdapter;
        use bincode::de::DecoderImpl;
        use bincode::de::read::SliceReader;

        let mut cfg = cfg_with_budget(1_000);
        cfg.set("on_overload", "fail".to_string());
        let src = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        assert!(matches!(src.overload, OverloadState::Fail));

        let bytes = bincode::encode_to_vec(BincodeAdapter(&src), bincode::config::standard())
            .expect("freeze must succeed");

        let mut dst = AnytimeTask::<TestPlanner>::new(Some(&cfg), ()).unwrap();
        let mut d = DecoderImpl::new(SliceReader::new(&bytes), bincode::config::standard(), ());
        Freezable::thaw(&mut dst, &mut d).expect("thaw must succeed");
        assert!(matches!(dst.overload, OverloadState::Fail));
    }

    // Distinct `A` monomorphizations must yield distinct `TypePath::type_path`
    // strings, otherwise the type registry would collide them.
    #[cfg(feature = "reflect")]
    #[test]
    fn type_path_is_per_inner_monomorphization() {
        // Second distinct `A` for the type_path comparison; bodies unused.
        #[derive(Reflect)]
        struct OtherPlanner;
        impl Freezable for OtherPlanner {}
        impl Anytime for OtherPlanner {
            type Input = Counter;
            type Output = Counter;
            type Resources<'r> = ();
            fn new(_c: Option<&ComponentConfig>, _r: Self::Resources<'_>) -> CuResult<Self> {
                Ok(Self)
            }
            fn base(&mut self, _i: &Self::Input, _c: &CuContext) -> CuResult<()> {
                Ok(())
            }
            fn refine(&mut self, _i: &Self::Input) -> CuResult<Step> {
                Ok(Step::Converged)
            }
            fn write_best(&self, _out: &mut Self::Output) {}
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
