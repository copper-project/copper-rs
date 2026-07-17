//! Trait and types to implement an anytime Copper task.
//!
//! An anytime task splits its work into a mandatory minimum ([`CuAnytimeTask::base`])
//! plus optional bounded improvements ([`CuAnytimeTask::refine`]). The task *reports*
//! what each quantum achieved through [`AnytimeStatus`]; the runtime *decides* whether
//! to schedule another quantum from that status stream and its configured time budget
//! and quality target. The task never sees the policy, only the [`AnytimeStep`] hints,
//! so implementations stay reusable under any policy.

use crate::config::ComponentConfig;
use crate::context::CuContext;
use crate::cutask::{CuMsgPack, CuMsgPayload, Freezable};
use crate::reflect::Reflect;
use cu29_clock::CuDuration;
use cu29_traits::{CuError, CuResult};

/// User-defined quality metric; higher is better. The scale is task-defined but must
/// be consistent with the quality target configured for the task.
pub type Quality = f32;

/// Returned by [`CuAnytimeTask::base`] and [`CuAnytimeTask::refine`]; drives the
/// runtime's refinement scheduling.
///
/// After any `Ok` return, the output must be valid and hold the best result produced
/// so far for the current job: a quantum that regresses or plateaus keeps its
/// candidate in task-local state and leaves the output untouched. The runtime never
/// buffers or rolls back the output, so it can publish it at any stop point, and the
/// published quality is monotone even when the algorithm internally is not.
#[derive(Debug, Clone)]
pub enum AnytimeStatus {
    /// The output holds the best result so far; further refinement may help.
    Improved { quality: Option<Quality> },
    /// Proactive yield: no further improvement is possible for this job. The output
    /// holds the final result.
    Converged { quality: Option<Quality> },
    /// Proactive give-up: the algorithm diverged or reached an unrecoverable state
    /// for this job. Refinement stops; the output is published as-is, so a task that
    /// can no longer vouch even for its base result must clear the payload before
    /// returning this. The next copperlist starts a fresh job.
    ///
    /// Return this when retrying with fresh input could plausibly succeed; return
    /// `Err` from `base`/`refine` when the task itself is broken.
    Aborted { error: Option<CuError> },
}

/// Runtime knowledge passed to [`CuAnytimeTask::refine`] — scheduling hints, not
/// commands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnytimeStep {
    /// 0 for the first `refine()` after `base()`.
    pub iteration: u32,
    /// Time since `base()` started for this job.
    pub elapsed: CuDuration,
    /// Time left in the budget; `None` if no time budget is configured. A task can
    /// use this to size its next quantum (e.g. skip a strategy that cannot fit).
    pub remaining: Option<CuDuration>,
    /// Best quality reported so far for this job, including by `base()`.
    pub best_quality: Option<Quality>,
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
    ) -> CuResult<AnytimeStatus>;

    /// Performs exactly one bounded refinement quantum.
    ///
    /// On `Ok` (any status), `output` is valid and holds the best result produced so
    /// far for this job; see [`AnytimeStatus`] for the commit-only-improvements
    /// contract.
    ///
    /// This method must not contain an unbounded refinement loop: the runtime can
    /// only observe time and quality *between* calls, so one call must be one
    /// bounded quantum.
    fn refine<'o>(
        &mut self,
        ctx: &CuContext,
        step: &AnytimeStep,
        output: &mut Self::Output<'o>,
    ) -> CuResult<AnytimeStatus>;

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cutask::CuMsg;
    use crate::input_msg;
    use crate::output_msg;

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
        ) -> CuResult<AnytimeStatus> {
            self.target = *input.payload().ok_or("no input")?;
            self.acc = 0;
            output.set_payload(self.acc);
            Ok(AnytimeStatus::Improved { quality: Some(0.0) })
        }

        fn refine<'o>(
            &mut self,
            _ctx: &CuContext,
            _step: &AnytimeStep,
            output: &mut Self::Output<'o>,
        ) -> CuResult<AnytimeStatus> {
            if self.acc == self.target {
                return Ok(AnytimeStatus::Converged { quality: Some(1.0) });
            }
            self.acc += 1;
            output.set_payload(self.acc);
            Ok(AnytimeStatus::Improved {
                quality: Some(self.acc as Quality / self.target as Quality),
            })
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
        assert!(matches!(status, AnytimeStatus::Improved { .. }));
        assert_eq!(output.payload(), Some(&0));

        let mut best_quality = Some(0.0);
        for iteration in 0..8 {
            let step = AnytimeStep {
                iteration,
                elapsed: CuDuration(0),
                remaining: None,
                best_quality,
            };
            match task.refine(&ctx, &step, &mut output).unwrap() {
                AnytimeStatus::Improved { quality } => best_quality = quality,
                AnytimeStatus::Converged { .. } => break,
                status => panic!("unexpected status: {status:?}"),
            }
        }
        assert_eq!(output.payload(), Some(&3));
        assert_eq!(best_quality, Some(1.0));
        task.postprocess(&ctx).unwrap();
        task.stop(&ctx).unwrap();
    }
}
