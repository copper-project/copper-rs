//! An RRT* path planner as a Copper anytime task.
//!
//! [`RrtStarPlanner`] consumes a [`PlanRequest`] and publishes a [`PlanPath`].
//! `base()` grows the tree until it has a first, crude path; every `refine()`
//! runs one more block of RRT* iterations and republishes only when the path
//! got shorter. The task reports how good the path is; the RON `anytime:`
//! policy decides how long to keep going.

mod rrt;

pub use rrt::{MAX_OBSTACLES, MAX_WAYPOINTS, Obstacle, Point2, World};

use crate::rrt::{RrtParams, RrtStar};
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu_rng::prelude::*;
use cu29::cutask_anytime::{AnytimeStatus, CuAnytimeTask, Quality, quality_from_f32};
use cu29::prelude::*;
use cu29::units::si::f32::Length;
use cu29::units::si::length::meter;
use serde::{Deserialize, Serialize};

/// Quality at which the path matches the straight-line lower bound: there is
/// nothing left to refine.
const CONVERGED_QUALITY: f32 = 0.999;

/// One planning problem. The map travels with the job, so the source owns it
/// and may change it between jobs.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct PlanRequest {
    pub world: World,
    pub start: Point2,
    pub goal: Point2,
}

/// The best path known when the refinement window closed.
///
/// Every consecutive pair of waypoints is collision free, so the path can be
/// driven as published.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct PlanPath {
    pub waypoints: [Point2; MAX_WAYPOINTS],
    /// Waypoints actually used in `waypoints`.
    pub len: u32,
    /// Cost of the RRT* tree path, which is what the anytime quality scores.
    /// The published waypoints are a shortcut of it, so this is an upper bound
    /// on the distance actually driven.
    pub cost: Length,
    /// RRT* iterations spent on this path, base block included.
    pub iterations: u32,
}

/// What a remote debugger sees of a planner node: the progress of the job,
/// not the thousands of tree nodes behind it.
///
/// The fields are read through `Reflect`, which the compiler cannot see.
#[allow(dead_code)]
#[derive(Default, Debug, Reflect)]
pub struct PlannerDebugState {
    pub iterations: u32,
    pub tree_size: u32,
    /// Nodes on the best path before it is shortcut for publication.
    pub tree_path_len: u32,
    /// Cost of the best path in the tree, infinite while there is none.
    pub best_cost: Length,
    /// Cost of the path currently in the output.
    pub published_cost: Length,
    pub published_quality: Quality,
}

mod planner_resources {
    use super::*;
    resources!({ rng => Owned<CuRng> });
}

/// An RRT* planner as an anytime task.
///
/// `base()` runs the first block of iterations and publishes the first path it
/// finds; each `refine()` runs one more block and republishes only when the
/// path got shorter. How many blocks run is the policy's call, not the task's.
///
/// Randomness comes from a `cu_rng::CuRngBundle` resource: job N's stream is
/// a pure function of the resource seed and N, so the same seed replays the
/// same trees.
#[derive(Reflect)]
pub struct RrtStarPlanner {
    params: RrtParams,
    /// Iterations of the base block, aiming at a first path.
    base_iterations: u32,
    /// Iterations of one refinement quantum.
    block_iterations: u32,
    /// One draw from the node's RNG resource, taken at construction; every
    /// job's stream derives from it.
    base_seed: u64,
    /// Jobs started so far, part of the frozen state: replay reruns job N on
    /// the stream job N used live.
    job_counter: u64,
    /// The current job, `None` before the first `base()`.
    #[reflect(ignore)]
    planner: Option<RrtStar>,
    /// Cost of the path currently in the output; infinite while none was
    /// published for this job.
    published_cost: f32,
    published_quality: f32,
}

// The search state is per-job and re-initialized by `base()` at the start of
// every copperlist; what must survive a keyframe is the seeding state.
impl Freezable for RrtStarPlanner {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.base_seed, encoder)?;
        Encode::encode(&self.job_counter, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.base_seed = Decode::decode(decoder)?;
        self.job_counter = Decode::decode(decoder)?;
        Ok(())
    }
}

impl RrtStarPlanner {
    /// Commits the best path of the tree when it beats the published one, and
    /// returns the published quality. Leaving the output alone when nothing
    /// improved is what the anytime contract asks for: the output always holds
    /// the best result so far, so the runtime can publish it at any stop point.
    fn publish(&mut self, output: &mut CuMsg<PlanPath>) -> Quality {
        if let Some(planner) = self.planner.as_ref()
            && planner.has_solution()
            && planner.best_cost() < self.published_cost
        {
            let mut waypoints = [Point2::default(); MAX_WAYPOINTS];
            // A path too long to represent is not published: the output keeps
            // the last valid one and a later quantum tries again.
            if let Some(len) = planner.write_path(&mut waypoints) {
                output.set_payload(PlanPath {
                    waypoints,
                    len,
                    cost: Length::new::<meter>(planner.best_cost()),
                    iterations: planner.iterations(),
                });
                self.published_cost = planner.best_cost();
                self.published_quality = planner.quality();
            }
        }
        quality_from_f32(self.published_quality)
    }

    /// The projected view a debug session gets instead of the whole tree.
    fn debug_state(&self) -> PlannerDebugState {
        let planner = self.planner.as_ref();
        PlannerDebugState {
            iterations: planner.map_or(0, RrtStar::iterations),
            tree_size: planner.map_or(0, RrtStar::tree_size),
            tree_path_len: planner.map_or(0, |p| p.tree_path_len() as u32),
            best_cost: Length::new::<meter>(planner.map_or(f32::INFINITY, RrtStar::best_cost)),
            published_cost: Length::new::<meter>(self.published_cost),
            published_quality: quality_from_f32(self.published_quality),
        }
    }
}

impl CuAnytimeTask for RrtStarPlanner {
    type Input<'m> = input_msg!(PlanRequest);
    type Output<'m> = output_msg!(PlanPath);
    type Resources<'r> = planner_resources::Resources;
    type Quality = Quality;

    // The task struct holds a whole RRT* tree, up to `max_nodes` entries. The
    // default hooks would ship all of it on every debug read, so the node
    // exposes a small view instead.
    fn register_debug_state_types(registry: &mut TypeRegistry) {
        registry.register::<PlannerDebugState>();
    }

    fn debug_state_type_path() -> &'static str {
        PlannerDebugState::type_path()
    }

    fn with_debug_state<R>(&self, f: impl FnOnce(&dyn bevy_reflect::Reflect) -> R) -> R {
        f(&self.debug_state())
    }

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self> {
        let mut params = RrtParams::default();
        let mut base_iterations = 400u32;
        let mut block_iterations = 256u32;
        if let Some(config) = config {
            if let Some(value) = config.get::<f32>("step_size")? {
                params.step_size = value;
            }
            if let Some(value) = config.get::<f32>("goal_bias")? {
                params.goal_bias = value;
            }
            if let Some(value) = config.get::<f32>("goal_threshold")? {
                params.goal_threshold = value;
            }
            if let Some(value) = config.get::<f32>("gamma")? {
                params.gamma = value;
            }
            if let Some(value) = config.get::<u32>("prune_interval")? {
                params.prune_interval = value;
            }
            if let Some(value) = config.get::<u32>("max_nodes")? {
                params.max_nodes = value;
            }
            if let Some(value) = config.get::<u32>("base_iterations")? {
                base_iterations = value;
            }
            if let Some(value) = config.get::<u32>("block_iterations")? {
                block_iterations = value;
            }
        }
        let Owned(mut rng) = resources.rng;
        Ok(Self {
            params,
            base_iterations,
            block_iterations,
            base_seed: rng.random::<u64>(),
            job_counter: 0,
            planner: None,
            published_cost: f32::INFINITY,
            published_quality: 0.0,
        })
    }

    fn base(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        let Some(request) = input.payload() else {
            // A source may publish nothing in a copperlist: skip the job
            // instead of failing the application. The recycled output must
            // not keep the previous job's path.
            output.clear_payload();
            self.published_cost = f32::INFINITY;
            self.published_quality = 0.0;
            return Ok(AnytimeStatus::Aborted);
        };
        self.job_counter = self.job_counter.wrapping_add(1);
        // ChaCha8 seeding decorrelates consecutive seed values, so a plain
        // add is enough to give every job an independent stream.
        let seed = self.base_seed.wrapping_add(self.job_counter);
        let planner = match self.planner.as_mut() {
            // Restart on the previous job's memory instead of a fresh tree.
            Some(planner) => {
                planner.reset(request.world.clone(), request.start, request.goal, seed);
                planner
            }
            None => self.planner.insert(RrtStar::new(
                request.world.clone(),
                self.params,
                request.start,
                request.goal,
                seed,
            )),
        };
        planner.grow(self.base_iterations);
        self.published_cost = f32::INFINITY;
        self.published_quality = 0.0;
        // Output messages are recycled: with no path yet the message must not
        // still carry the previous job's path.
        output.clear_payload();
        let quality = self.publish(output);
        if self.published_quality >= CONVERGED_QUALITY {
            // The base path already matches the straight line: no refinement
            // can beat it.
            return Ok(AnytimeStatus::Converged(quality));
        }
        // Even with no path found the job goes on: refinement is what usually
        // finds one, and a quality of 0.0 stays under any configured floor.
        Ok(AnytimeStatus::Improved(quality))
    }

    fn refine(
        &mut self,
        _ctx: &CuContext,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        let planner = self
            .planner
            .as_mut()
            .ok_or("rrt*: refine() without a job from base()")?;
        if planner.is_exhausted() {
            return Ok(AnytimeStatus::Converged(quality_from_f32(
                self.published_quality,
            )));
        }
        planner.grow(self.block_iterations);
        let quality = self.publish(output);
        if self.published_quality >= CONVERGED_QUALITY {
            // The path matches the straight line: no iteration can beat it.
            return Ok(AnytimeStatus::Converged(quality));
        }
        Ok(AnytimeStatus::Improved(quality))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const START: Point2 = Point2::new(0.5, 0.5);
    const GOAL: Point2 = Point2::new(9.5, 9.5);

    /// The resource binding as the generated runtime would hand it over.
    fn test_resources(seed: u64) -> planner_resources::Resources {
        planner_resources::Resources {
            rng: Owned(CuRng::from_seed(seed)),
        }
    }

    /// The anytime contract driven by hand: `base()` publishes a first path,
    /// every `refine()` leaves the output holding the best path so far, and
    /// the debug state tracks the job instead of exposing the whole tree.
    #[test]
    fn refinement_only_commits_improvements() {
        let ctx = CuContext::new_with_clock();
        let mut task = RrtStarPlanner::new(None, test_resources(42)).unwrap();
        let input = CuMsg::new(Some(PlanRequest {
            world: World::depot(),
            start: START,
            goal: GOAL,
        }));
        let mut output = CuMsg::new(None);

        task.start(&ctx).unwrap();
        let status = task.base(&ctx, &input, &mut output).unwrap();
        assert!(matches!(status, AnytimeStatus::Improved(_)));

        let mut best = f32::INFINITY;
        for _ in 0..24 {
            if let AnytimeStatus::Aborted = task.refine(&ctx, &mut output).unwrap() {
                panic!("the planner should not abort on a solvable map");
            }
            if let Some(path) = output.payload() {
                assert!(
                    path.cost.raw() <= best + 1e-4,
                    "the output regressed: {best} then {}",
                    path.cost.raw()
                );
                best = path.cost.raw();
            }
        }
        assert!(output.payload().is_some(), "no path after 24 quanta");

        let state = task.debug_state();
        assert_eq!(state.published_cost.raw(), best);
        assert!(state.best_cost <= state.published_cost);
        assert!(state.published_quality.raw() > 0.0);
        assert!(state.iterations > 0 && state.tree_size > 0);
    }

    /// A copperlist without a request skips the job instead of failing the
    /// application, and must not leak the previous job's path.
    #[test]
    fn missing_request_skips_the_job() {
        let ctx = CuContext::new_with_clock();
        let mut task = RrtStarPlanner::new(None, test_resources(42)).unwrap();
        let input = CuMsg::new(Some(PlanRequest {
            world: World::depot(),
            start: START,
            goal: GOAL,
        }));
        let mut output = CuMsg::new(None);

        task.start(&ctx).unwrap();
        task.base(&ctx, &input, &mut output).unwrap();
        for _ in 0..24 {
            task.refine(&ctx, &mut output).unwrap();
        }
        assert!(output.payload().is_some(), "no path to leak");

        let empty = CuMsg::new(None);
        let status = task.base(&ctx, &empty, &mut output).unwrap();
        assert!(matches!(status, AnytimeStatus::Aborted));
        assert!(output.payload().is_none(), "the old path leaked");
        assert_eq!(task.debug_state().published_quality.raw(), 0.0);
    }

    /// A request with the start on the goal is solved by definition: `base()`
    /// converges at once with quality 1.0, never NaN.
    #[test]
    fn start_on_goal_converges_immediately() {
        let ctx = CuContext::new_with_clock();
        let mut task = RrtStarPlanner::new(None, test_resources(3)).unwrap();
        let input = CuMsg::new(Some(PlanRequest {
            world: World::depot(),
            start: START,
            goal: START,
        }));
        let mut output = CuMsg::new(None);

        task.start(&ctx).unwrap();
        let status = task.base(&ctx, &input, &mut output).unwrap();
        assert!(matches!(status, AnytimeStatus::Converged(_)));
        assert!(output.payload().is_some(), "a trivial path is still a path");
        assert_eq!(task.debug_state().published_quality.raw(), 1.0);
    }
}
