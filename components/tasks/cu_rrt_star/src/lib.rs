//! An RRT* path planner as a Copper anytime task.
//!
//! [`RrtStarPlanner`] consumes a [`PlanRequest`] and publishes a [`PlanPath`].
//! `base()` grows the tree until it has a first, crude path; every `refine()`
//! runs one more block of RRT* iterations and republishes only when the path
//! got shorter. The task reports how good the path is; the RON `anytime:`
//! policy decides how long to keep going.

mod rrt;

pub use rrt::*;

use bincode::{Decode, Encode};
use cu29::cutask_anytime::{AnytimeStatus, CuAnytimeTask, Quality, quality_from_f32};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

/// Quality at which the path matches the straight-line lower bound: there is
/// nothing left to refine.
const CONVERGED_QUALITY: f32 = 0.999;

/// One planning problem. A different seed per job makes each job a fresh RRT*
/// run rather than a replay of the previous one.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct PlanRequest {
    pub start: Point2,
    pub goal: Point2,
    pub seed: u64,
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
    pub cost: f32,
    /// RRT* iterations spent on this path, base block included.
    pub iterations: u32,
    pub tree_size: u32,
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
    /// Cost of the best path in the tree, `f32::INFINITY` while there is none.
    pub best_cost: f32,
    /// Cost of the path currently in the output.
    pub published_cost: f32,
    pub published_quality: f32,
}

/// An RRT* planner as an anytime task.
///
/// `base()` runs the first block of iterations and publishes the first path it
/// finds; each `refine()` runs one more block and republishes only when the
/// path got shorter. How many blocks run is the policy's call, not the task's.
#[derive(Reflect)]
pub struct RrtStarPlanner {
    params: RrtParams,
    /// Iterations of the base block, aiming at a first path.
    base_iterations: u32,
    /// Iterations of one refinement quantum.
    block_iterations: u32,
    /// The current job, `None` before the first `base()`.
    planner: Option<RrtStar>,
    /// Cost of the path currently in the output; infinite while none was
    /// published for this job.
    published_cost: f32,
    published_quality: f32,
}

// All mutable state is per-job and re-initialized by `base()` at the start of
// every copperlist, so there is nothing to snapshot for a foreground node.
impl Freezable for RrtStarPlanner {}

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
                    cost: planner.best_cost(),
                    iterations: planner.iterations(),
                    tree_size: planner.tree_size(),
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
            best_cost: planner.map_or(f32::INFINITY, RrtStar::best_cost),
            published_cost: self.published_cost,
            published_quality: self.published_quality,
        }
    }
}

impl CuAnytimeTask for RrtStarPlanner {
    type Input<'m> = input_msg!(PlanRequest);
    type Output<'m> = output_msg!(PlanPath);
    type Resources<'r> = ();
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

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
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
        Ok(Self {
            params,
            base_iterations,
            block_iterations,
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
        let request = input.payload().ok_or("rrt*: no plan request")?;
        let planner = match self.planner.as_mut() {
            // Restart on the previous job's memory instead of a fresh tree.
            Some(planner) => {
                planner.reset(request.start, request.goal, request.seed);
                planner
            }
            None => self.planner.insert(RrtStar::new(
                World::depot(),
                self.params,
                request.start,
                request.goal,
                request.seed,
            )),
        };
        planner.grow(self.base_iterations);
        self.published_cost = f32::INFINITY;
        self.published_quality = 0.0;
        // Output messages are recycled: with no path yet the message must not
        // still carry the previous job's path.
        output.clear_payload();
        // Even with no path found the job goes on: refinement is what usually
        // finds one, and a quality of 0.0 stays under any configured floor.
        Ok(AnytimeStatus::Improved(self.publish(output)))
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

    /// The anytime contract driven by hand: `base()` publishes a first path,
    /// every `refine()` leaves the output holding the best path so far, and
    /// the debug state tracks the job instead of exposing the whole tree.
    #[test]
    fn refinement_only_commits_improvements() {
        let ctx = CuContext::new_with_clock();
        let mut task = RrtStarPlanner::new(None, ()).unwrap();
        let input = CuMsg::new(Some(PlanRequest {
            start: START,
            goal: GOAL,
            seed: 42,
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
                    path.cost <= best + 1e-4,
                    "the output regressed: {best} then {}",
                    path.cost
                );
                best = path.cost;
            }
        }
        assert!(output.payload().is_some(), "no path after 24 quanta");

        let state = task.debug_state();
        assert_eq!(state.published_cost, best);
        assert!(state.best_cost <= state.published_cost);
        assert!(state.published_quality > 0.0);
        assert!(state.iterations > 0 && state.tree_size > 0);
    }
}
