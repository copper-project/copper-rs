//! Trait and types to implement an anytime Copper task.
//!
//! An anytime task splits its work into a mandatory minimum ([`CuAnytimeTask::base`])
//! plus optional bounded improvements ([`CuAnytimeTask::refine`]). The task *reports*
//! what each quantum achieved through [`AnytimeStatus`]; the runtime *decides* whether
//! to schedule another quantum from that status stream and its configured time budget
//! and quality target. The task never sees the policy, so implementations stay
//! reusable under any policy.

use crate::config::ComponentConfig;
use crate::context::CuContext;
use crate::cutask::{CuMsg, CuMsgPack, CuMsgPayload, Freezable};
use crate::reflect::{GetTypeRegistration, Reflect, TypePath, TypeRegistry};
use compact_str::format_compact;
use core::fmt::{Debug, Formatter, Result as FmtResult};
use core::marker::PhantomData;
use cu29_clock::{CuDuration, CuTime};
use cu29_traits::{CuCompactString, CuResult};
use cu29_units::si::f32::Ratio;
use cu29_units::si::ratio::ratio;

/// Normalized quality of a published result: a dimensionless [`Ratio`] in
/// `0.0..=1.0`, higher is better and `1.0` means no further improvement is
/// meaningful. Sharing one scale across tasks keeps a configured quality target
/// portable.
pub type Quality = Ratio;

/// Returned by [`CuAnytimeTask::base`] and [`CuAnytimeTask::refine`]; drives the
/// runtime's refinement scheduling.
///
/// `Q` is [`CuAnytimeTask::Quality`]: [`Quality`] for tasks that can score their
/// result, `()` for tasks that cannot.
///
/// After any `Ok` return, the output must be valid and hold the best result produced
/// so far for the current job: a quantum that regresses or plateaus keeps its
/// candidate in task-local state and leaves the output untouched. The runtime never
/// buffers or rolls back the output, so it can publish it at any stop point, and the
/// published quality is monotone even when the algorithm internally is not.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AnytimeStatus<Q> {
    /// The output holds the best result so far; further refinement may help.
    Improved(Q),
    /// Proactive yield: no further improvement is possible for this job. The output
    /// holds the final result, published unless the quality floor rejects it.
    Converged(Q),
    /// Proactive give-up: the algorithm diverged or reached an unrecoverable state
    /// for this job. Refinement stops; the output is published as-is — subject to
    /// the quality floor once a quality has been reported — so a task that can no
    /// longer vouch even for its base result must clear the payload before
    /// returning this. The next copperlist starts a fresh job.
    Aborted,
}

/// A task producing a valid result from the bare-minimum compute, then improving it
/// in bounded quanta for as long as the runtime allows.
///
/// Per job: `preprocess` → `base` → N × `refine` → `postprocess`, where N is chosen
/// by the runtime (possibly 0: a time budget may suppress every refinement, but
/// never the base computation).
pub trait CuAnytimeTask: Freezable + Reflect {
    type Input<'m>: CuMsgPack;
    type Output<'m>: CuMsgPayload;
    /// Resources required by the task.
    type Resources<'r>;
    /// Measure reported through [`AnytimeStatus`]: [`Quality`] for tasks that can
    /// score their result, `()` for tasks that cannot. A quality target can only be
    /// configured for tasks whose `Quality` is comparable to it, so a target on a
    /// `()` task is rejected at compile time.
    type Quality: Copy + PartialOrd;

    /// Registers the reflected type used as this task's debug-state contract.
    ///
    /// The default exposes the task struct itself. Override this when the task
    /// contains ignored, third-party, hardware, or otherwise non-inspectable
    /// internals and should expose a purpose-built debug-state view instead.
    fn register_debug_state_types(registry: &mut TypeRegistry)
    where
        Self: GetTypeRegistration + Sized,
    {
        registry.register::<Self>();
    }

    /// Returns the reflected type path used as this task's debug-state schema.
    fn debug_state_type_path() -> &'static str
    where
        Self: TypePath + Sized,
    {
        Self::type_path()
    }

    /// Borrows this task's current debug-state view.
    ///
    /// Override this together with [`debug_state_type_path`](Self::debug_state_type_path)
    /// when the debug state is a projected view rather than the task struct.
    fn with_debug_state<R>(&self, f: impl FnOnce(&dyn Reflect) -> R) -> R
    where
        Self: Sized,
    {
        f(self)
    }

    /// Here you need to initialize everything your task will need for the duration
    /// of its lifetime. The config allows you to access the configuration of the task.
    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized;

    /// Start is called between the creation of the task and the first call to
    /// pre/base.
    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }

    /// This is a method called by the runtime before "base". This is a kind of best
    /// effort, as soon as possible call to give a chance for the task to do some work
    /// before to prepare to make "base" as short as possible.
    fn preprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }

    /// Starts a new job and writes its minimum valid result into `output`.
    ///
    /// On `Ok`: `output` is valid and safe to publish, refinement state from the
    /// preceding job has been reset, and later `refine()` calls improve this job.
    /// The task must capture into its own per-job state everything refinement will
    /// need from `input`: `refine()` does not receive the input (in background
    /// placements refinement outlives the copperlist that carried it), and the task
    /// knows the cheapest representation to retain.
    ///
    /// On `Err`: the job produced no valid output and the error propagates like a
    /// `CuTask::process` failure.
    fn base<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<AnytimeStatus<Self::Quality>>;

    /// Performs exactly one bounded refinement quantum.
    ///
    /// On `Ok` (any status), `output` is valid and holds the best result produced so
    /// far for this job; see [`AnytimeStatus`] for the commit-only-improvements
    /// contract.
    ///
    /// Anything a quantum could want to know about its own job the task already has:
    /// it can count its quanta, read the clock through `ctx`, and remembers the last
    /// quality it reported.
    ///
    /// This method must not contain an unbounded refinement loop: the runtime can
    /// only observe time and quality *between* calls, so one call must be one
    /// bounded quantum.
    fn refine<'o>(
        &mut self,
        ctx: &CuContext,
        output: &mut Self::Output<'o>,
    ) -> CuResult<AnytimeStatus<Self::Quality>>;

    /// This is a method called by the runtime after the job's refinement window has
    /// closed. It is best effort a chance for the task to update some state out of
    /// the critical path, for example to release scratch memory or maintain
    /// statistics that are not time-critical for the robot.
    fn postprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }

    /// Called to stop the task. It signals that `base`/`refine` won't be called
    /// until start is called again.
    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }
}

/// Converts a normalized `f32` (e.g. a RON policy knob) into a [`Quality`].
#[inline(always)]
pub fn quality_from_f32(value: f32) -> Quality {
    Quality::new::<ratio>(value)
}

/// Reads a [`Quality`] back as a normalized `f32`.
#[inline(always)]
pub fn quality_to_f32(quality: Quality) -> f32 {
    quality.get::<ratio>()
}

/// A node's `anytime:` RON policy, carried as compile-time constants.
///
/// Codegen emits one zero-sized impl per anytime node; `Q` is the task's
/// [`CuAnytimeTask::Quality`]. An unset knob is `None` and its check in
/// [`AnytimeJob::check`] const-folds away. `max_refines` does not appear here:
/// it is consumed while emitting the execution plan and never read at run time.
pub trait AnytimePolicy<Q> {
    /// Wall-clock refinement window per job, from job start.
    const TIME_BUDGET: Option<CuDuration>;
    /// Validity horizon, from the input's Tov anchor.
    const MAX_AGE: Option<CuDuration>;
    /// Stop after this many quanta without the best quality improving.
    const MAX_STALL: Option<u32>;

    /// Codegen override: `q >= target` (never satisfied by NaN). Default false.
    #[inline(always)]
    fn target_met(_q: Q) -> bool {
        false
    }
    /// Codegen override: `q < floor`, NaN counting as below the floor
    /// (emitted as `q.partial_cmp(&floor).is_none_or(Ordering::is_lt)`).
    /// Default false.
    #[inline(always)]
    fn below_floor(_q: Q) -> bool {
        false
    }
    /// Normalized quality for the status stamp; `None` when `Quality = ()`.
    #[inline(always)]
    fn quality_ratio(_q: Q) -> Option<f32> {
        None
    }
}

/// Why a job stopped refining (or never started).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnytimeStopCause {
    /// The task reported no further improvement is possible.
    Converged,
    /// The configured quality target was reached.
    TargetMet,
    /// The wall-clock time budget elapsed.
    BudgetExhausted,
    /// The input's validity horizon passed between quanta; best-so-far published
    /// (subject to the quality floor).
    AgeExceeded,
    /// The validity horizon had already passed before `base()`; the job never ran.
    SkippedStale,
    /// The last emitted refine step ran; the plan has no more quanta for this job.
    MaxRefines,
    /// Too many quanta without the best quality improving.
    Stalled,
    /// The task gave up on this job; a base-site abort skips the floor gate.
    Aborted,
}

impl AnytimeStopCause {
    /// Short label used in the status stamp and interned logs.
    pub fn label(self) -> &'static str {
        match self {
            AnytimeStopCause::Converged => "conv",
            AnytimeStopCause::TargetMet => "tgt",
            AnytimeStopCause::BudgetExhausted => "bdgt",
            AnytimeStopCause::AgeExceeded => "age",
            AnytimeStopCause::SkippedStale => "stale",
            AnytimeStopCause::MaxRefines => "max",
            AnytimeStopCause::Stalled => "stall",
            AnytimeStopCause::Aborted => "abort",
        }
    }
}

/// What one job amounted to, recorded at its stop point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AnytimeOutcome {
    /// Refinement quanta that ran (the base computation is iteration 0).
    pub iterations: u32,
    /// Wall-clock span from job start to the stop point (zero when nothing ran).
    pub elapsed: CuDuration,
    /// Why the job stopped.
    pub stop: AnytimeStopCause,
    /// False when nothing was published (stale skip, quality floor, or a
    /// task-cleared abort).
    pub published: bool,
}

/// Runtime state of one live job, shared by every step of that job.
///
/// Holds only what genuinely varies at run time; anything positional (which
/// quantum this is, whether more remain) is fixed by the emitted plan.
/// Constructed once `base()` has reported a quality, so `best` needs no
/// `Option` (it is `()` for quality-less tasks).
pub struct AnytimeJob<Q, P> {
    /// Job start: time-budget anchor and elapsed origin.
    t0: CuTime,
    /// Age anchor: the input's Tov (falls back to `t0`).
    anchor: CuTime,
    /// Best quality reported so far (== the published quality).
    best: Q,
    /// Quanta since `best` last improved.
    stall: u32,
    _policy: PhantomData<P>,
}

// Manual impls: derives would demand bounds on the policy ZST it doesn't need.
// No `Copy`: `finish(self)` is a single-use guard.
impl<Q: Copy, P> Clone for AnytimeJob<Q, P> {
    fn clone(&self) -> Self {
        Self {
            t0: self.t0,
            anchor: self.anchor,
            best: self.best,
            stall: self.stall,
            _policy: PhantomData,
        }
    }
}
impl<Q: Debug, P> Debug for AnytimeJob<Q, P> {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        f.debug_struct("AnytimeJob")
            .field("t0", &self.t0)
            .field("anchor", &self.anchor)
            .field("best", &self.best)
            .field("stall", &self.stall)
            .finish()
    }
}

impl<Q: Copy + PartialOrd, P: AnytimePolicy<Q>> AnytimeJob<Q, P> {
    /// Starts a job at `t0` with the quality `base()` reported.
    pub fn new(t0: CuTime, anchor: CuTime, quality: Q) -> Self {
        Self {
            t0,
            anchor,
            best: quality,
            stall: 0,
            _policy: PhantomData,
        }
    }

    /// Records the quality one refine quantum reported.
    ///
    /// An unordered `best` (NaN) is displaced by the next report — NaN never
    /// wins a comparison, so it would otherwise pin `best` for the whole job.
    pub fn record(&mut self, quality: Q) {
        if quality > self.best || self.best.partial_cmp(&self.best).is_none() {
            self.best = quality;
            self.stall = 0;
        } else if P::MAX_STALL.is_some() {
            self.stall += 1;
        }
    }

    /// Checks the configured between-quanta bounds, in stop-cause attribution
    /// order: target → budget → age → stall. All comparisons are `>=`.
    pub fn check(&self, now: CuTime) -> Option<AnytimeStopCause> {
        if P::target_met(self.best) {
            return Some(AnytimeStopCause::TargetMet);
        }
        if let Some(budget) = P::TIME_BUDGET
            && now >= self.t0 + budget
        {
            return Some(AnytimeStopCause::BudgetExhausted);
        }
        if let Some(age) = P::MAX_AGE
            && now >= self.anchor + age
        {
            return Some(AnytimeStopCause::AgeExceeded);
        }
        if let Some(max_stall) = P::MAX_STALL
            && self.stall >= max_stall
        {
            return Some(AnytimeStopCause::Stalled);
        }
        None
    }

    /// Ends the job: applies the quality floor, stamps the status text and
    /// returns the outcome. `iterations` comes from the caller — the plan
    /// knows the quantum count, the job does not track one.
    pub fn finish<O: CuMsgPayload>(
        self,
        now: CuTime,
        cause: AnytimeStopCause,
        iterations: u32,
        output: &mut CuMsg<O>,
    ) -> AnytimeOutcome {
        let published = if P::below_floor(self.best) {
            output.clear_payload();
            false
        } else {
            output.payload().is_some()
        };
        stamp(
            output,
            iterations,
            P::quality_ratio(self.best),
            cause,
            published,
        );
        AnytimeOutcome {
            iterations,
            elapsed: now - self.t0,
            stop: cause,
            published,
        }
    }
}

/// Writes the `"any: {N}it [q=X.XX ]{label}[ !p]"` status stamp shared by every
/// terminal site, moving the built string straight into `status_txt`.
fn stamp<O: CuMsgPayload>(
    output: &mut CuMsg<O>,
    iterations: u32,
    quality: Option<f32>,
    cause: AnytimeStopCause,
    published: bool,
) {
    let not_published = if published { "" } else { " !p" };
    output.metadata.status_txt = CuCompactString(match quality {
        Some(q) => format_compact!(
            "any: {}it q={:.2} {}{}",
            iterations,
            q,
            cause.label(),
            not_published
        ),
        None => format_compact!("any: {}it {}{}", iterations, cause.label(), not_published),
    });
}

/// Terminal outcome when the age limit passed before `base()`: the job is
/// skipped and nothing is published.
pub fn skip_stale<O: CuMsgPayload>(output: &mut CuMsg<O>) -> AnytimeOutcome {
    output.clear_payload();
    stamp(output, 0, None, AnytimeStopCause::SkippedStale, false);
    AnytimeOutcome {
        iterations: 0,
        elapsed: CuDuration::default(),
        stop: AnytimeStopCause::SkippedStale,
        published: false,
    }
}

/// Terminal outcome when `base()` returns `Aborted`: no quality was reported
/// so the floor gate does not apply; `published` reflects whether the task
/// left a payload it still vouches for. `t0`/`now` bracket the base computation.
pub fn abort_at_base<O: CuMsgPayload>(
    t0: CuTime,
    now: CuTime,
    output: &mut CuMsg<O>,
) -> AnytimeOutcome {
    let published = output.payload().is_some();
    stamp(output, 0, None, AnytimeStopCause::Aborted, published);
    AnytimeOutcome {
        iterations: 0,
        elapsed: now - t0,
        stop: AnytimeStopCause::Aborted,
        published,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cutask::CuMsg;
    use crate::input_msg;
    use crate::output_msg;

    fn q(v: f32) -> Quality {
        quality_from_f32(v)
    }

    /// Sums its input one increment per quantum: base publishes 0, each refine
    /// commits one more increment until the captured input is fully consumed.
    #[derive(Reflect)]
    struct IncrementalSum {
        target: u32,
        acc: u32,
    }

    impl Freezable for IncrementalSum {}

    impl CuAnytimeTask for IncrementalSum {
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);
        type Resources<'r> = ();
        type Quality = Quality;

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self { target: 0, acc: 0 })
        }

        fn base<'i, 'o>(
            &mut self,
            _ctx: &CuContext,
            input: &Self::Input<'i>,
            output: &mut Self::Output<'o>,
        ) -> CuResult<AnytimeStatus<Quality>> {
            self.target = *input.payload().ok_or("no input")?;
            self.acc = 0;
            output.set_payload(self.acc);
            Ok(AnytimeStatus::Improved(q(0.0)))
        }

        fn refine<'o>(
            &mut self,
            _ctx: &CuContext,
            output: &mut Self::Output<'o>,
        ) -> CuResult<AnytimeStatus<Quality>> {
            if self.acc == self.target {
                return Ok(AnytimeStatus::Converged(q(1.0)));
            }
            self.acc += 1;
            output.set_payload(self.acc);
            Ok(AnytimeStatus::Improved(q(
                self.acc as f32 / self.target as f32
            )))
        }
    }

    #[test]
    fn base_then_refine_until_converged() {
        let ctx = CuContext::new_with_clock();
        let mut task = IncrementalSum::new(None, ()).unwrap();
        let input = CuMsg::new(Some(3u32));
        let mut output = CuMsg::new(None);

        task.start(&ctx).unwrap();
        task.preprocess(&ctx).unwrap();
        let status = task.base(&ctx, &input, &mut output).unwrap();
        assert!(matches!(status, AnytimeStatus::Improved(_)));
        assert_eq!(output.payload(), Some(&0));

        let mut best_quality = q(0.0);
        for _ in 0..8 {
            match task.refine(&ctx, &mut output).unwrap() {
                AnytimeStatus::Improved(quality) => best_quality = quality,
                AnytimeStatus::Converged(quality) => {
                    best_quality = quality;
                    break;
                }
                status => panic!("unexpected status: {status:?}"),
            }
        }
        assert_eq!(output.payload(), Some(&3));
        assert_eq!(quality_to_f32(best_quality), 1.0);
        task.postprocess(&ctx).unwrap();
        task.stop(&ctx).unwrap();
    }

    /// Mirrors codegen output for a policy with every knob set:
    /// budget 1ms, age 2ms, target 0.9, floor 0.3, stall 2.
    struct FullPolicy;
    impl AnytimePolicy<Quality> for FullPolicy {
        const TIME_BUDGET: Option<CuDuration> = Some(CuDuration(1_000_000));
        const MAX_AGE: Option<CuDuration> = Some(CuDuration(2_000_000));
        const MAX_STALL: Option<u32> = Some(2);

        fn target_met(q: Quality) -> bool {
            q >= quality_from_f32(0.9)
        }
        fn below_floor(q: Quality) -> bool {
            q.partial_cmp(&quality_from_f32(0.3))
                .is_none_or(core::cmp::Ordering::is_lt)
        }
        fn quality_ratio(q: Quality) -> Option<f32> {
            Some(quality_to_f32(q))
        }
    }

    /// Mirrors a quality-less node (`Quality = ()`, no knobs set).
    struct BarePolicy;
    impl AnytimePolicy<()> for BarePolicy {
        const TIME_BUDGET: Option<CuDuration> = None;
        const MAX_AGE: Option<CuDuration> = None;
        const MAX_STALL: Option<u32> = None;
    }

    #[test]
    fn check_attribution_order_is_target_budget_age_stall() {
        let t0 = CuTime::from_millis(10);
        // Target met wins over an elapsed budget.
        let job = AnytimeJob::<Quality, FullPolicy>::new(t0, t0, q(0.95));
        assert_eq!(
            job.check(t0 + CuDuration::from_millis(5)),
            Some(AnytimeStopCause::TargetMet)
        );
        // Budget (>= 1ms from t0) wins over age (>= 2ms from anchor).
        let job = AnytimeJob::<Quality, FullPolicy>::new(t0, t0, q(0.5));
        assert_eq!(
            job.check(t0 + CuDuration::from_millis(5)),
            Some(AnytimeStopCause::BudgetExhausted)
        );
        // Age fires alone when the anchor is older than t0.
        let anchor = t0 - CuDuration::from_millis(2);
        let job = AnytimeJob::<Quality, FullPolicy>::new(t0, anchor, q(0.5));
        assert_eq!(
            job.check(t0 + CuDuration::from_nanos(1)),
            Some(AnytimeStopCause::AgeExceeded)
        );
        // Nothing configured fires within bounds.
        let job = AnytimeJob::<Quality, FullPolicy>::new(t0, t0, q(0.5));
        assert_eq!(job.check(t0), None);
    }

    #[test]
    fn stall_counts_quanta_without_improvement() {
        let t0 = CuTime::from_millis(1);
        let mut job = AnytimeJob::<Quality, FullPolicy>::new(t0, t0, q(0.5));
        job.record(q(0.5)); // no improvement: stall 1
        assert_eq!(job.check(t0), None);
        job.record(q(0.6)); // improvement resets
        assert_eq!(job.check(t0), None);
        job.record(q(0.6));
        job.record(q(0.6)); // stall 2 -> stalled
        assert_eq!(job.check(t0), Some(AnytimeStopCause::Stalled));
    }

    #[test]
    fn finish_gates_on_floor_and_stamps_status() {
        let t0 = CuTime::from_millis(1);
        let now = t0 + CuDuration::from_micros(250);

        // Above the floor: published, stamped with quality and cause.
        let mut output: CuMsg<u32> = CuMsg::new(Some(42));
        let job = AnytimeJob::<Quality, FullPolicy>::new(t0, t0, q(0.5));
        let outcome = job.finish(now, AnytimeStopCause::BudgetExhausted, 3, &mut output);
        assert!(outcome.published);
        assert_eq!(outcome.iterations, 3);
        assert_eq!(outcome.elapsed, CuDuration::from_micros(250));
        assert_eq!(output.payload(), Some(&42));
        assert_eq!(
            output.metadata.status_txt.0.as_str(),
            "any: 3it q=0.50 bdgt"
        );

        // Below the floor: payload cleared, not published.
        let mut output: CuMsg<u32> = CuMsg::new(Some(42));
        let job = AnytimeJob::<Quality, FullPolicy>::new(t0, t0, q(0.1));
        let outcome = job.finish(now, AnytimeStopCause::MaxRefines, 2, &mut output);
        assert!(!outcome.published);
        assert_eq!(output.payload(), None);
        assert_eq!(
            output.metadata.status_txt.0.as_str(),
            "any: 2it q=0.10 max !p"
        );
    }

    #[test]
    fn nan_quality_fails_closed() {
        let t0 = CuTime::from_millis(1);

        // A NaN best is below the floor: payload cleared, not published.
        let mut output: CuMsg<u32> = CuMsg::new(Some(1));
        let job = AnytimeJob::<Quality, FullPolicy>::new(t0, t0, q(f32::NAN));
        let outcome = job.finish(t0, AnytimeStopCause::MaxRefines, 1, &mut output);
        assert!(!outcome.published);
        assert_eq!(output.payload(), None);

        // A NaN best is displaced by the next report.
        let mut job = AnytimeJob::<Quality, FullPolicy>::new(t0, t0, q(f32::NAN));
        job.record(q(0.4));
        let mut output: CuMsg<u32> = CuMsg::new(Some(1));
        let outcome = job.finish(t0, AnytimeStopCause::MaxRefines, 1, &mut output);
        assert!(outcome.published);

        // A NaN refine never displaces a real best.
        let mut job = AnytimeJob::<Quality, FullPolicy>::new(t0, t0, q(0.5));
        job.record(q(f32::NAN));
        let mut output: CuMsg<u32> = CuMsg::new(Some(1));
        let outcome = job.finish(t0, AnytimeStopCause::MaxRefines, 1, &mut output);
        assert!(outcome.published);
        assert_eq!(output.metadata.status_txt.0.as_str(), "any: 1it q=0.50 max");
    }

    #[test]
    fn quality_less_job_has_no_quality_in_stamp() {
        let t0 = CuTime::from_millis(1);
        let mut output: CuMsg<u32> = CuMsg::new(Some(7));
        let job = AnytimeJob::<(), BarePolicy>::new(t0, t0, ());
        assert_eq!(job.check(t0 + CuDuration::from_secs(1)), None); // nothing configured
        let outcome = job.finish(t0, AnytimeStopCause::MaxRefines, 4, &mut output);
        assert!(outcome.published);
        assert_eq!(output.metadata.status_txt.0.as_str(), "any: 4it max");
    }

    #[test]
    fn base_site_terminal_outcomes() {
        let t0 = CuTime::from_millis(1);
        let now = t0 + CuDuration::from_micros(80);

        let mut output: CuMsg<u32> = CuMsg::new(Some(9));
        let outcome = skip_stale(&mut output);
        assert_eq!(outcome.stop, AnytimeStopCause::SkippedStale);
        assert!(!outcome.published);
        assert_eq!(outcome.elapsed, CuDuration::default());
        assert_eq!(output.payload(), None);
        assert_eq!(output.metadata.status_txt.0.as_str(), "any: 0it stale !p");

        // Aborted with a payload the task still vouches for: published.
        let mut output: CuMsg<u32> = CuMsg::new(Some(9));
        let outcome = abort_at_base(t0, now, &mut output);
        assert!(outcome.published);
        assert_eq!(outcome.elapsed, CuDuration::from_micros(80));
        assert_eq!(output.payload(), Some(&9));

        // Task-cleared abort: not published.
        let mut output: CuMsg<u32> = CuMsg::new(None);
        let outcome = abort_at_base(t0, now, &mut output);
        assert!(!outcome.published);
        assert_eq!(output.metadata.status_txt.0.as_str(), "any: 0it abort !p");
    }
}
