//! Seeded RRT* over an [`RrtSpace`]: anything that can sample a candidate
//! point and answer clearance queries. The bundled space is [`World`], a
//! rectangle of round obstacles; the algorithm itself never depends on the
//! obstacle shape.
//!
//! The planner knows nothing about Copper: it only exposes [`RrtStar::grow`],
//! one bounded block of iterations. [`crate::RrtStarPlanner`] calls it once
//! from `base()` and once per anytime refinement quantum.
//!
//! The steps follow Karaman and Frazzoli: sample with goal bias, nearest,
//! steer, choose the cheapest parent, rewire the neighborhood, and prune by
//! branch and bound. Three points are stricter here than in a textbook write-up:
//! the final leg to the goal is collision checked and counted in the path cost,
//! rewiring refuses an ancestor so rounding cannot close a cycle, and the cost
//! shift after a rewire is iterative instead of recursive.

use bincode::{Decode, Encode};
use cu_rng::prelude::*;
use cu_spatial_payloads::{BBox2f, Point2f, Point2fSoa};
use cu29::prelude::*;
use cu29::units::si::area::square_meter;
use cu29::units::si::f32::{Area, Length};
use cu29::units::si::length::meter;
use serde::{Deserialize, Serialize};

/// Waypoints carried by a published path. Kept at 32 because serde derives
/// array impls up to that size.
pub const MAX_WAYPOINTS: usize = 32;

/// Tree nodes one job can hold. This is the capacity of the SoA position set,
/// so it is fixed at compile time; [`RrtParams::max_nodes`] is clamped to it.
pub const MAX_NODES: usize = 4096;

/// Obstacles carried by a [`World`]. Bounded so the map can travel inside a
/// [`crate::PlanRequest`] without allocating; must stay at or under 32 for
/// the same serde reason as [`MAX_WAYPOINTS`].
pub const MAX_OBSTACLES: usize = 16;

/// A round obstacle: anything within `radius` of `center` is occupied.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct Obstacle {
    pub center: Point2f,
    pub radius: Length,
}

impl Obstacle {
    pub const fn new(center: Point2f, radius: Length) -> Self {
        Self { center, radius }
    }
}

/// A space that can answer how far a point is from the nearest obstacle.
///
/// The clearance is a signed distance: positive in free space, zero on a
/// surface, negative inside an obstacle or out of bounds. What counts as free
/// is the caller's predicate - `clearance(p) > robot_radius` - so one space
/// serves robots of any size.
pub trait Clearance {
    /// The point type of the space, which fixes its dimension.
    type Point: Copy;

    /// Signed distance from `p` to the nearest occupied geometry.
    fn clearance(&self, p: Self::Point) -> Length;

    /// Smallest clearance anywhere along the segment `a`-`b`.
    fn clearance_segment(&self, a: Self::Point, b: Self::Point) -> Length;
}

/// A rectangular world and its obstacles.
///
/// The obstacle storage is a fixed array so the map can travel inside a
/// [`crate::PlanRequest`]: the planner has no map of its own, every job
/// carries the one it must solve.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct World {
    pub bounds: BBox2f,
    pub obstacles: [Obstacle; MAX_OBSTACLES],
    /// Obstacles actually used in `obstacles`.
    pub obstacle_count: u32,
}

impl World {
    /// A world from its bounds and a list of obstacles. Errors when the list
    /// exceeds [`MAX_OBSTACLES`].
    pub fn new(bounds: BBox2f, obstacles: &[Obstacle]) -> CuResult<Self> {
        if obstacles.len() > MAX_OBSTACLES {
            return Err(format!(
                "rrt*: {} obstacles, the world holds at most {MAX_OBSTACLES}",
                obstacles.len()
            )
            .into());
        }
        let mut world = Self {
            bounds,
            obstacles: [Obstacle::default(); MAX_OBSTACLES],
            obstacle_count: obstacles.len() as u32,
        };
        world.obstacles[..obstacles.len()].copy_from_slice(obstacles);
        Ok(world)
    }

    /// The map the demo and the tests run on: a 10x10 m depot with five
    /// pillars, placed so the straight line between opposite corners is
    /// blocked. A first path is therefore always a detour, and refinement has
    /// real work to do.
    pub fn depot() -> Self {
        let meters = Length::new::<meter>;
        let point = Point2f::from_meters;
        Self::new(
            BBox2f::new(point(0.0, 0.0), point(10.0, 10.0)),
            &[
                Obstacle::new(point(3.0, 3.0), meters(1.2)),
                Obstacle::new(point(6.0, 6.0), meters(1.5)),
                Obstacle::new(point(7.0, 2.5), meters(1.0)),
                Obstacle::new(point(2.5, 7.0), meters(1.0)),
                Obstacle::new(point(5.0, 1.5), meters(0.8)),
            ],
        )
        .expect("the depot obstacles fit MAX_OBSTACLES")
    }

    /// The obstacles in use, with a count out of range clamped rather than
    /// trusted: a hand-built `World` must not be able to cause a panic here.
    fn obstacle_slice(&self) -> &[Obstacle] {
        &self.obstacles[..(self.obstacle_count as usize).min(MAX_OBSTACLES)]
    }

    /// Area left free by the obstacles. Assumes every obstacle lies inside the
    /// bounds and none overlap, which holds for [`World::depot`].
    pub fn free_area(&self) -> Area {
        let blocked: f32 = self
            .obstacle_slice()
            .iter()
            .map(|o| core::f32::consts::PI * o.radius.raw() * o.radius.raw())
            .sum();
        let width = (self.bounds.max.x - self.bounds.min.x).raw();
        let height = (self.bounds.max.y - self.bounds.min.y).raw();
        Area::new::<square_meter>((width * height - blocked).max(f32::EPSILON))
    }
}

impl Clearance for World {
    type Point = Point2f;

    fn clearance(&self, p: Point2f) -> Length {
        let b = &self.bounds;
        let mut clearance = (p.x - b.min.x)
            .raw()
            .min((b.max.x - p.x).raw())
            .min((p.y - b.min.y).raw())
            .min((b.max.y - p.y).raw());
        for o in self.obstacle_slice() {
            clearance = clearance.min(p.distance(o.center).raw() - o.radius.raw());
        }
        Length::new::<meter>(clearance)
    }

    fn clearance_segment(&self, a: Point2f, b: Point2f) -> Length {
        // The wall terms are linear along the segment, so their minimum sits
        // at an endpoint; the obstacle terms need the true segment distance.
        let mut clearance = self.clearance(a).raw().min(self.clearance(b).raw());
        for o in self.obstacle_slice() {
            clearance = clearance.min(distance_to_segment(a, b, o.center) - o.radius.raw());
        }
        Length::new::<meter>(clearance)
    }
}

/// What RRT* needs from the space it searches: the clearance queries of
/// [`Clearance`], a sampler, and the rewiring constant. The algorithm never
/// depends on the obstacle shape; [`World`] is the bundled implementation.
pub trait RrtSpace: Clearance<Point = Point2f> {
    /// A uniform random point inside the bounds of the space.
    fn sample(&self, rng: &mut CuRng) -> Point2f;

    /// The RRT* radius constant of Karaman and Frazzoli:
    /// `gamma* = 2 * (1 + 1/d)^(1/d) * (free_measure / zeta_d)^(1/d)`.
    ///
    /// A smaller constant shrinks the rewiring neighborhood below what
    /// asymptotic optimality needs, and the planner degrades toward plain RRT
    /// as the tree grows.
    fn rrt_star_gamma(&self) -> Length;
}

impl RrtSpace for World {
    fn sample(&self, rng: &mut CuRng) -> Point2f {
        let b = &self.bounds;
        Point2f::new(
            b.min.x + (b.max.x - b.min.x) * rng.random::<f32>(),
            b.min.y + (b.max.y - b.min.y) * rng.random::<f32>(),
        )
    }

    /// The 2D instance of the constant: `d = 2` and `zeta_2 = pi`.
    fn rrt_star_gamma(&self) -> Length {
        Length::new::<meter>(
            2.0 * 1.5f32.sqrt() * (self.free_area().raw() / core::f32::consts::PI).sqrt(),
        )
    }
}

/// Euclidean distance in meters, the planner's cost metric.
fn dist(a: Point2f, b: Point2f) -> f32 {
    a.distance(b).raw()
}

/// Distance in meters from `point` to the segment `a`-`b`.
fn distance_to_segment(a: Point2f, b: Point2f, point: Point2f) -> f32 {
    let (abx, aby) = ((b.x - a.x).raw(), (b.y - a.y).raw());
    let len_sq = abx * abx + aby * aby;
    if len_sq <= f32::EPSILON {
        return dist(a, point);
    }
    let t = (((point.x - a.x).raw() * abx + (point.y - a.y).raw() * aby) / len_sq).clamp(0.0, 1.0);
    dist(a.lerp(b, t), point)
}

/// Tuning knobs of the planner, all read from the node's RON `config:`.
#[derive(Debug, Clone, Copy, Reflect)]
pub struct RrtParams {
    /// Longest edge the planner adds in one extension, in meters.
    pub step_size: f32,
    /// Probability of sampling the goal instead of a random point.
    pub goal_bias: f32,
    /// A node this close to the goal closes a path.
    pub goal_threshold: f32,
    /// Gamma of the RRT* rewiring radius `gamma * sqrt(ln n / n)`. `0.0`
    /// derives it from the space through [`RrtSpace::rrt_star_gamma`], which is
    /// the value RRT* needs to converge to the optimum.
    pub gamma: f32,
    /// Branch-and-bound prune every N iterations; 0 disables pruning.
    pub prune_interval: u32,
    /// Hard cap on the tree size, so one job cannot grow without bound.
    /// Clamped to [`MAX_NODES`] by [`RrtStar::new`].
    pub max_nodes: u32,
}

impl Default for RrtParams {
    fn default() -> Self {
        Self {
            step_size: 0.8,
            goal_bias: 0.05,
            goal_threshold: 0.5,
            gamma: 0.0,
            prune_interval: 512,
            max_nodes: 4000,
        }
    }
}

/// The topology of one vertex. Its position lives in [`RrtStar::positions`] at
/// the same index.
#[derive(Debug, Clone)]
struct TreeNode {
    /// `None` for the root only.
    parent: Option<u32>,
    /// Path cost from the start to this node.
    cost: f32,
    children: Vec<u32>,
}

/// An RRT* search for one start/goal pair over a space `S`.
///
/// The tree only ever improves: `best_cost` is monotone non-increasing over
/// iterations, which is what makes the algorithm a good anytime task.
pub struct RrtStar<S: RrtSpace = World> {
    space: S,
    params: RrtParams,
    /// The rewiring gamma in effect for the current job, in meters: the
    /// configured one, or the one derived from the job's space when the
    /// config left it at 0.
    gamma: f32,
    start: Point2f,
    goal: Point2f,
    /// Node positions, kept apart from the topology so the two scans every
    /// iteration runs - nearest and the rewiring neighborhood - are one
    /// vectorized pass over packed coordinates.
    positions: Point2fSoa<MAX_NODES>,
    /// Parent, cost and children, indexed like `positions`.
    tree: Vec<TreeNode>,
    /// Node closing the best path found so far.
    best_goal: Option<u32>,
    /// Cost of the best path found so far, infinite until one is found.
    best_cost: f32,
    iterations: u32,
    rng: CuRng,
    /// Reused between iterations to keep the search allocation-free.
    scratch_d2: Vec<Area>,
    scratch_near: Vec<u32>,
    scratch_stack: Vec<u32>,
}

impl<S: RrtSpace> RrtStar<S> {
    /// Starts a search rooted at `start`. An unreachable or blocked `start`
    /// simply never grows a tree; the caller sees "no path" and the anytime
    /// quality floor drops the result.
    pub fn new(space: S, params: RrtParams, start: Point2f, goal: Point2f, seed: u64) -> Self {
        let mut planner = Self {
            space,
            params: RrtParams {
                // The position set has a compile-time capacity, so a config
                // asking for more nodes than it holds is capped, not honored.
                max_nodes: params.max_nodes.min(MAX_NODES as u32),
                ..params
            },
            gamma: 0.0,
            start,
            goal,
            positions: Point2fSoa::default(),
            tree: Vec::new(),
            best_goal: None,
            best_cost: f32::INFINITY,
            iterations: 0,
            rng: CuRng::from_seed(seed),
            // Sized once so the per-iteration scans never touch the allocator.
            scratch_d2: vec![Area::default(); MAX_NODES],
            scratch_near: Vec::new(),
            scratch_stack: Vec::new(),
        };
        planner.restart(start, goal, seed);
        planner
    }

    /// Restarts the search on a new problem, keeping the capacity the previous
    /// job grew: after the first job the planner asks the allocator for much
    /// less.
    pub fn reset(&mut self, space: S, start: Point2f, goal: Point2f, seed: u64) {
        self.space = space;
        self.restart(start, goal, seed);
    }

    fn restart(&mut self, start: Point2f, goal: Point2f, seed: u64) {
        // The map can change between jobs, so a derived gamma must follow it.
        self.gamma = if self.params.gamma > 0.0 {
            self.params.gamma
        } else {
            self.space.rrt_star_gamma().raw()
        };
        self.start = start;
        self.goal = goal;
        self.tree.clear();
        self.positions.len = 0;
        self.positions.push(start);
        self.tree.push(TreeNode {
            parent: None,
            cost: 0.0,
            children: Vec::new(),
        });
        self.best_goal = None;
        self.best_cost = f32::INFINITY;
        self.iterations = 0;
        self.rng = CuRng::from_seed(seed);
    }

    /// Runs one bounded block of `iterations` RRT* iterations.
    pub fn grow(&mut self, iterations: u32) {
        for _ in 0..iterations {
            self.iterations += 1;
            if self.tree.len() < self.params.max_nodes as usize {
                self.step();
            }
            if self.params.prune_interval > 0
                && self.iterations.is_multiple_of(self.params.prune_interval)
                && self.best_goal.is_some()
            {
                self.prune();
            }
        }
    }

    /// Cost of the best path so far, infinite while no path is known.
    pub fn best_cost(&self) -> f32 {
        self.best_cost
    }

    /// True once a path to the goal exists.
    pub fn has_solution(&self) -> bool {
        self.best_goal.is_some()
    }

    pub fn tree_size(&self) -> u32 {
        self.tree.len() as u32
    }

    pub fn iterations(&self) -> u32 {
        self.iterations
    }

    /// True when the tree is full and pruning can never free room again, so no
    /// further iteration can change anything.
    pub fn is_exhausted(&self) -> bool {
        self.tree.len() >= self.params.max_nodes as usize
            && (self.params.prune_interval == 0 || self.best_goal.is_none())
    }

    /// Shortest conceivable path: the straight line, obstacles ignored.
    pub fn lower_bound(&self) -> f32 {
        dist(self.start, self.goal)
    }

    /// Normalized quality in `0.0..=1.0`: how close the best path is to the
    /// straight-line lower bound. 0.0 means no path yet, 1.0 means the path is
    /// as short as the world allows.
    pub fn quality(&self) -> f32 {
        if !self.has_solution() {
            return 0.0;
        }
        let lower_bound = self.lower_bound();
        if self.best_cost <= lower_bound {
            // Covers the degenerate job with the start on the goal, where the
            // ratio would divide zero by zero.
            return 1.0;
        }
        (lower_bound / self.best_cost).clamp(0.0, 1.0)
    }

    /// Nodes on the best path before shortcutting, the goal included. Zero
    /// while no path is known.
    pub fn tree_path_len(&self) -> usize {
        let Some(goal_node) = self.best_goal else {
            return 0;
        };
        let mut len = 1; // the goal itself, which is not a tree node
        let mut cursor = Some(goal_node);
        while let Some(index) = cursor {
            len += 1;
            cursor = self.tree[index as usize].parent;
        }
        len
    }

    /// Writes the best path into `out` and returns how many waypoints it used.
    ///
    /// The tree path routinely holds more nodes than [`MAX_WAYPOINTS`], so it
    /// is shortcut first: from each waypoint the path jumps to the furthest
    /// later one still reachable in a straight free line. Shortcutting is what
    /// a planner publishes anyway, and it keeps every published segment
    /// collision free - dropping the tail instead would publish a straight
    /// jump across the map.
    ///
    /// The shortcut path is never longer than the tree path, so the reported
    /// cost stays an upper bound on what the robot drives. `None` means even
    /// the shortcut path does not fit; the caller then publishes nothing
    /// rather than a path that cuts through an obstacle.
    pub fn write_path(&self, out: &mut [Point2f; MAX_WAYPOINTS]) -> Option<u32> {
        let goal_node = self.best_goal?;
        let mut chain = Vec::new();
        let mut cursor = Some(goal_node);
        while let Some(index) = cursor {
            chain.push(self.positions.get(index as usize));
            cursor = self.tree[index as usize].parent;
        }
        chain.reverse();
        chain.push(self.goal);

        let mut len = 0usize;
        let mut at = 0usize;
        loop {
            if len == MAX_WAYPOINTS {
                return None;
            }
            out[len] = chain[at];
            len += 1;
            if at == chain.len() - 1 {
                return Some(len as u32);
            }
            // The next tree node is always reachable - it is a tree edge - so
            // the scan only looks for something further.
            let mut next = at + 1;
            for candidate in (at + 2)..chain.len() {
                if self.segment_free(chain[at], chain[candidate]) {
                    next = candidate;
                }
            }
            at = next;
        }
    }

    /// True when the whole segment keeps positive clearance.
    fn segment_free(&self, a: Point2f, b: Point2f) -> bool {
        self.space.clearance_segment(a, b).raw() > 0.0
    }

    /// One RRT* iteration: sample, steer, choose the cheapest parent, rewire
    /// the neighborhood, then check whether the new node closes a better path.
    fn step(&mut self) {
        let sample = self.sample();
        let nearest = self.nearest(sample);
        let from = self.positions.get(nearest as usize);
        let new_pos = steer(from, sample, self.params.step_size);
        if !self.segment_free(from, new_pos) {
            return;
        }

        // Squared radius against squared distances: the whole neighborhood
        // scan then stays sqrt-free, which is what lets it vectorize.
        let radius = self.near_radius();
        let radius_sq = radius * radius;
        let n = self.positions.len();
        self.positions
            .distances_squared(new_pos, &mut self.scratch_d2);
        let mut near = core::mem::take(&mut self.scratch_near);
        near.clear();
        for index in 0..n {
            if self.scratch_d2[index].raw() <= radius_sq {
                near.push(index as u32);
            }
        }

        // Choose the parent that gives the cheapest path to the new node.
        let mut parent = nearest;
        let mut cost = self.tree[nearest as usize].cost + dist(from, new_pos);
        for &index in near.iter() {
            let candidate_pos = self.positions.get(index as usize);
            let candidate_cost = self.tree[index as usize].cost + dist(candidate_pos, new_pos);
            if candidate_cost < cost && self.segment_free(candidate_pos, new_pos) {
                parent = index;
                cost = candidate_cost;
            }
        }

        let new_index = self.tree.len() as u32;
        self.positions.push(new_pos);
        self.tree.push(TreeNode {
            parent: Some(parent),
            cost,
            children: Vec::new(),
        });
        self.tree[parent as usize].children.push(new_index);

        // Rewire: neighbors that are cheaper to reach through the new node.
        for &index in near.iter() {
            if index == parent {
                continue;
            }
            let neighbor_pos = self.positions.get(index as usize);
            let neighbor_cost = self.tree[index as usize].cost;
            let rewired_cost = cost + dist(neighbor_pos, new_pos);
            if rewired_cost < neighbor_cost
                && !self.is_ancestor(index, new_index)
                && self.segment_free(new_pos, neighbor_pos)
            {
                self.reparent(index, new_index, rewired_cost);
            }
        }
        self.scratch_near = near;

        // Does the new node close a better path?
        let to_goal = dist(new_pos, self.goal);
        if to_goal <= self.params.goal_threshold
            && self.segment_free(new_pos, self.goal)
            && cost + to_goal < self.best_cost
        {
            self.best_cost = cost + to_goal;
            self.best_goal = Some(new_index);
        }
        // Rewiring may have shortened the current best path too.
        if let Some(goal_node) = self.best_goal {
            let cost = self.tree[goal_node as usize].cost;
            let pos = self.positions.get(goal_node as usize);
            self.best_cost = self.best_cost.min(cost + dist(pos, self.goal));
        }
    }

    /// A random point of the space, biased toward the goal.
    fn sample(&mut self) -> Point2f {
        if self.rng.random::<f32>() < self.params.goal_bias {
            return self.goal;
        }
        self.space.sample(&mut self.rng)
    }

    /// Index of the tree node closest to `point`. Linear on purpose: over the
    /// packed SoA coordinates the scan is one vectorized sqrt-free pass, which
    /// is fast enough at `max_nodes` scale; a spatial index would only pay off
    /// on much larger trees. Squared distance has the same argmin as distance.
    fn nearest(&mut self, point: Point2f) -> u32 {
        let n = self.positions.len();
        self.positions.distances_squared(point, &mut self.scratch_d2);
        let mut best = 0u32;
        let mut best_distance = f32::INFINITY;
        for index in 0..n {
            let distance = self.scratch_d2[index].raw();
            if distance < best_distance {
                best_distance = distance;
                best = index as u32;
            }
        }
        best
    }

    /// RRT* rewiring radius `gamma * sqrt(ln n / n)`, capped at one step.
    fn near_radius(&self) -> f32 {
        let n = (self.tree.len() as f32).max(2.0);
        (self.gamma * (n.ln() / n).sqrt()).min(self.params.step_size)
    }

    /// True when `candidate` sits on the path from `node` up to the root.
    ///
    /// Rewiring an ancestor would turn the tree into a graph with a cycle, and
    /// every walk over it would then loop forever. Exact arithmetic already
    /// rules it out - reaching an ancestor through its own descendant is never
    /// cheaper - but rounding on two nearly coincident samples must not be able
    /// to break that.
    fn is_ancestor(&self, candidate: u32, node: u32) -> bool {
        let mut cursor = self.tree[node as usize].parent;
        while let Some(index) = cursor {
            if index == candidate {
                return true;
            }
            cursor = self.tree[index as usize].parent;
        }
        false
    }

    /// Moves `node` under `new_parent` and shifts the cost of its whole
    /// subtree by the same delta.
    fn reparent(&mut self, node: u32, new_parent: u32, new_cost: f32) {
        if let Some(old_parent) = self.tree[node as usize].parent {
            self.tree[old_parent as usize]
                .children
                .retain(|&child| child != node);
        }
        self.tree[node as usize].parent = Some(new_parent);
        self.tree[new_parent as usize].children.push(node);

        let delta = new_cost - self.tree[node as usize].cost;
        let mut stack = core::mem::take(&mut self.scratch_stack);
        stack.clear();
        stack.push(node);
        while let Some(index) = stack.pop() {
            self.tree[index as usize].cost += delta;
            for i in 0..self.tree[index as usize].children.len() {
                stack.push(self.tree[index as usize].children[i]);
            }
        }
        self.scratch_stack = stack;
    }

    /// Branch and bound: drop every node that cannot belong to a path better
    /// than the best one known.
    ///
    /// Walking down from the root keeps the tree consistent: a node is kept
    /// only if its parent is kept, so no orphan survives the compaction. The
    /// triangle inequality makes that almost free anyway - a kept node's parent
    /// always satisfies the bound as well.
    fn prune(&mut self) {
        // The best path itself is protected: rounding must never let branch and
        // bound drop the path it is bounding against.
        let mut protected = vec![false; self.tree.len()];
        let mut cursor = self.best_goal;
        while let Some(index) = cursor {
            protected[index as usize] = true;
            cursor = self.tree[index as usize].parent;
        }

        let mut keep = vec![false; self.tree.len()];
        let mut stack = core::mem::take(&mut self.scratch_stack);
        stack.clear();
        stack.push(0);
        keep[0] = true;
        while let Some(index) = stack.pop() {
            for i in 0..self.tree[index as usize].children.len() {
                let child = self.tree[index as usize].children[i];
                let cost = self.tree[child as usize].cost;
                let pos = self.positions.get(child as usize);
                if protected[child as usize] || cost + dist(pos, self.goal) <= self.best_cost {
                    keep[child as usize] = true;
                    stack.push(child);
                }
            }
        }
        self.scratch_stack = stack;

        let mut remap = vec![u32::MAX; self.tree.len()];
        let mut kept = Vec::with_capacity(self.tree.len());
        for index in 0..self.tree.len() {
            if keep[index] {
                // The positions compact in place: a kept node never moves to a
                // higher slot, so the copy never overwrites a slot still to read.
                let destination = kept.len();
                remap[index] = destination as u32;
                self.positions.x[destination] = self.positions.x[index];
                self.positions.y[destination] = self.positions.y[index];
                kept.push(TreeNode {
                    parent: self.tree[index].parent,
                    cost: self.tree[index].cost,
                    children: Vec::new(),
                });
            }
        }
        self.positions.len = kept.len();
        for node in kept.iter_mut() {
            node.parent = node.parent.map(|parent| remap[parent as usize]);
        }
        for index in 0..kept.len() {
            if let Some(parent) = kept[index].parent {
                kept[parent as usize].children.push(index as u32);
            }
        }
        self.best_goal = self.best_goal.map(|goal| remap[goal as usize]);
        self.tree = kept;
    }
}

/// Point at most `step_size` away from `from` in the direction of `to`.
fn steer(from: Point2f, to: Point2f, step_size: f32) -> Point2f {
    let distance = dist(from, to);
    if distance <= step_size {
        return to;
    }
    from.lerp(to, step_size / distance)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn start() -> Point2f {
        Point2f::from_meters(0.5, 0.5)
    }

    fn goal() -> Point2f {
        Point2f::from_meters(9.5, 9.5)
    }

    fn planner(seed: u64) -> RrtStar {
        RrtStar::new(World::depot(), RrtParams::default(), start(), goal(), seed)
    }

    #[test]
    fn world_rejects_too_many_obstacles() {
        let radius = Length::new::<meter>(0.1);
        let bounds = BBox2f::new(
            Point2f::from_meters(0.0, 0.0),
            Point2f::from_meters(10.0, 10.0),
        );
        let too_many = [Obstacle::new(Point2f::from_meters(1.0, 1.0), radius); MAX_OBSTACLES + 1];
        assert!(World::new(bounds, &too_many).is_err());
        assert!(World::new(bounds, &too_many[..MAX_OBSTACLES]).is_ok());
    }

    #[test]
    fn clearance_signs_match_the_geometry() {
        let world = World::depot();
        let point = Point2f::from_meters;
        // Straight through the pillar at (3, 3).
        assert!(
            world
                .clearance_segment(point(1.0, 1.0), point(5.0, 5.0))
                .raw()
                <= 0.0
        );
        // Along the free bottom edge.
        assert!(
            world
                .clearance_segment(point(0.2, 0.2), point(0.2, 9.8))
                .raw()
                > 0.0
        );
        // An endpoint out of bounds.
        assert!(world.clearance_segment(start(), point(11.0, 0.5)).raw() <= 0.0);
        // Inside a pillar the clearance goes negative.
        assert!(world.clearance(point(3.0, 3.0)).raw() < 0.0);
    }

    #[test]
    fn refinement_only_improves_the_path() {
        let mut planner = planner(42);
        planner.grow(400);
        assert!(planner.has_solution(), "no first path after the base block");

        let mut previous = planner.best_cost();
        for _ in 0..16 {
            planner.grow(256);
            assert!(
                planner.best_cost() <= previous + 1e-4,
                "cost went up: {} then {}",
                previous,
                planner.best_cost()
            );
            previous = planner.best_cost();
        }
        assert!(planner.quality() > 0.0 && planner.quality() <= 1.0);
        assert!(planner.best_cost() >= planner.lower_bound());
    }

    /// Every published path must be drivable, at every stop point the anytime
    /// policy could pick.
    ///
    /// A small `gamma` is included on purpose: it barely rewires, so its tree
    /// paths grow past [`MAX_WAYPOINTS`] and the shortcut is exercised rather
    /// than skipped. The same sweep doubles as the gamma comparison: a gamma
    /// below the value the free area implies shrinks the rewiring
    /// neighborhood, and refinement then converges to a worse path.
    #[test]
    fn published_path_is_valid_at_every_stop_point() {
        let world = World::depot();
        let derived = world.rrt_star_gamma().raw();
        assert!(
            (12.0..13.0).contains(&derived),
            "gamma for the depot map should be near 12.4 m, got {derived}"
        );

        let mut longest_tree_path = 0;
        // Total refined cost over all seeds, per gamma: the derived one first.
        let mut total_cost = [0.0f32; 2];
        for (seed, index) in (1..40u64).flat_map(|seed| [(seed, 0usize), (seed, 1)]) {
            let params = RrtParams {
                gamma: [0.0, 3.0][index],
                ..Default::default()
            };
            let mut planner = RrtStar::new(World::depot(), params, start(), goal(), seed);
            planner.grow(400);
            for _ in 0..24 {
                planner.grow(256);
                let mut waypoints = [Point2f::default(); MAX_WAYPOINTS];
                let Some(len) = planner.write_path(&mut waypoints) else {
                    panic!("seed {seed}: the shortcut path did not fit");
                };
                longest_tree_path = longest_tree_path.max(planner.tree_path_len());
                assert!(len >= 2, "a path has at least a start and a goal");
                assert_eq!(waypoints[0], start());
                assert_eq!(waypoints[(len - 1) as usize], goal());
                for pair in waypoints[..len as usize].windows(2) {
                    assert!(
                        world.clearance_segment(pair[0], pair[1]).raw() > 0.0,
                        "seed {seed}: published path crosses an obstacle"
                    );
                }
                // The shortcut only removes waypoints it can bypass in a
                // straight free line, so it never lengthens the path.
                let published: f32 = waypoints[..len as usize]
                    .windows(2)
                    .map(|pair| dist(pair[0], pair[1]))
                    .sum();
                assert!(
                    published <= planner.best_cost() + 1e-3,
                    "seed {seed}: shortcut path {published} longer than the cost {}",
                    planner.best_cost()
                );
            }
            total_cost[index] += planner.best_cost();
        }
        assert!(
            longest_tree_path > MAX_WAYPOINTS,
            "the tree path never outgrew MAX_WAYPOINTS, so the shortcut was never exercised"
        );
        assert!(
            total_cost[0] < total_cost[1],
            "the derived gamma should refine to a shorter path than a small one"
        );
    }

    /// A job with the start on the goal is solved by definition: quality 1.0,
    /// never NaN from the zero-by-zero ratio.
    #[test]
    fn degenerate_job_reports_full_quality() {
        let mut planner = RrtStar::new(World::depot(), RrtParams::default(), start(), start(), 3);
        planner.grow(400);
        assert!(planner.has_solution());
        assert_eq!(planner.quality(), 1.0);
    }

    #[test]
    fn same_seed_replays_the_same_tree() {
        let (mut a, mut b) = (planner(11), planner(11));
        a.grow(600);
        b.grow(300);
        b.grow(300);
        assert_eq!(a.tree_size(), b.tree_size());
        assert_eq!(a.best_cost(), b.best_cost());
    }

    #[test]
    fn pruning_keeps_the_best_path_reachable() {
        let mut planner = planner(3);
        planner.grow(1500);
        let cost_before = planner.best_cost();
        planner.prune();
        assert!(planner.has_solution(), "pruning dropped the goal node");
        // The tree stays consistent: every node still reaches the root.
        for index in 0..planner.tree.len() {
            let mut cursor = Some(index as u32);
            let mut hops = 0;
            while let Some(current) = cursor {
                cursor = planner.tree[current as usize].parent;
                hops += 1;
                assert!(hops <= planner.tree.len(), "cycle in the tree");
            }
        }
        assert_eq!(planner.best_cost(), cost_before);
    }
}
