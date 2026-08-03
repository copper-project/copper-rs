//! Seeded RRT* over a 2D world of round obstacles.
//!
//! The planner knows nothing about Copper: it only exposes [`RrtStar::grow`],
//! one bounded block of iterations. `tasks.rs` calls it once from `base()` and
//! once per anytime refinement quantum.
//!
//! The steps follow Karaman and Frazzoli: sample with goal bias, nearest,
//! steer, choose the cheapest parent, rewire the neighborhood, and prune by
//! branch and bound. Three points are stricter here than in a textbook write-up:
//! the final leg to the goal is collision checked and counted in the path cost,
//! rewiring refuses an ancestor so rounding cannot close a cycle, and the cost
//! shift after a rewire is iterative instead of recursive.

use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

/// Waypoints carried by a published path. Kept at 32 because serde derives
/// array impls up to that size.
pub const MAX_WAYPOINTS: usize = 32;

/// A point of the planar world, in meters.
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub struct Point2 {
    pub x: f32,
    pub y: f32,
}

impl Point2 {
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Euclidean distance to `other`.
    pub fn distance(self, other: Self) -> f32 {
        let (dx, dy) = (self.x - other.x, self.y - other.y);
        (dx * dx + dy * dy).sqrt()
    }
}

/// A round obstacle: the planner rejects any point or segment within `radius`
/// of `center`.
#[derive(Debug, Clone, Copy, Reflect)]
pub struct Obstacle {
    pub center: Point2,
    pub radius: f32,
}

/// The rectangular world `0..width` x `0..height` and its obstacles.
#[derive(Debug, Clone, Reflect)]
pub struct World {
    pub width: f32,
    pub height: f32,
    pub obstacles: Vec<Obstacle>,
}

impl World {
    /// The map every planner node of the example runs on: a 10x10 m depot with
    /// five pillars, placed so the straight line from start to goal is blocked.
    /// A first path is therefore always a detour, and refinement has real work
    /// to do.
    pub fn depot() -> Self {
        Self {
            width: 10.0,
            height: 10.0,
            obstacles: vec![
                Obstacle {
                    center: Point2::new(3.0, 3.0),
                    radius: 1.2,
                },
                Obstacle {
                    center: Point2::new(6.0, 6.0),
                    radius: 1.5,
                },
                Obstacle {
                    center: Point2::new(7.0, 2.5),
                    radius: 1.0,
                },
                Obstacle {
                    center: Point2::new(2.5, 7.0),
                    radius: 1.0,
                },
                Obstacle {
                    center: Point2::new(5.0, 1.5),
                    radius: 0.8,
                },
            ],
        }
    }

    /// True when `point` is inside the bounds and outside every obstacle.
    pub fn is_free(&self, point: Point2) -> bool {
        if point.x < 0.0 || point.y < 0.0 || point.x > self.width || point.y > self.height {
            return false;
        }
        self.obstacles
            .iter()
            .all(|o| point.distance(o.center) > o.radius)
    }

    /// True when the whole segment `a`-`b` is free.
    pub fn is_free_segment(&self, a: Point2, b: Point2) -> bool {
        if !self.is_free(a) || !self.is_free(b) {
            return false;
        }
        self.obstacles
            .iter()
            .all(|o| distance_to_segment(a, b, o.center) > o.radius)
    }

    /// Area left free by the obstacles. Assumes every obstacle lies inside the
    /// bounds and none overlap, which holds for [`World::depot`].
    pub fn free_area(&self) -> f32 {
        let blocked: f32 = self
            .obstacles
            .iter()
            .map(|o| core::f32::consts::PI * o.radius * o.radius)
            .sum();
        (self.width * self.height - blocked).max(f32::EPSILON)
    }

    /// The RRT* radius constant of Karaman and Frazzoli:
    /// `gamma* = 2 * (1 + 1/d)^(1/d) * (free_area / zeta_d)^(1/d)`, here with
    /// `d = 2` and `zeta_2 = pi`.
    ///
    /// A smaller constant shrinks the rewiring neighborhood below what
    /// asymptotic optimality needs, and the planner degrades toward plain RRT
    /// as the tree grows.
    pub fn rrt_star_gamma(&self) -> f32 {
        2.0 * 1.5f32.sqrt() * (self.free_area() / core::f32::consts::PI).sqrt()
    }
}

/// Distance from `point` to the segment `a`-`b`.
fn distance_to_segment(a: Point2, b: Point2, point: Point2) -> f32 {
    let (abx, aby) = (b.x - a.x, b.y - a.y);
    let len_sq = abx * abx + aby * aby;
    if len_sq <= f32::EPSILON {
        return a.distance(point);
    }
    let t = (((point.x - a.x) * abx + (point.y - a.y) * aby) / len_sq).clamp(0.0, 1.0);
    Point2::new(a.x + t * abx, a.y + t * aby).distance(point)
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
    /// derives it from the world through [`World::rrt_star_gamma`], which is
    /// the value RRT* needs to converge to the optimum.
    pub gamma: f32,
    /// Branch-and-bound prune every N iterations; 0 disables pruning.
    pub prune_interval: u32,
    /// Hard cap on the tree size, so one job cannot grow without bound.
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

/// xorshift64*, so a given seed always replays the same tree.
#[derive(Debug, Clone, Reflect)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        // splitmix64 finalizer: consecutive seeds must not start on neighboring
        // states, otherwise consecutive jobs explore almost the same tree.
        let mut state = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
        state = (state ^ (state >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        state = (state ^ (state >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        state ^= state >> 31;
        // xorshift64* must never start at zero.
        Self(if state == 0 { 1 } else { state })
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in `[0.0, 1.0)`.
    pub fn next_f32(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u32 << 24) as f32
    }
}

/// One vertex of the tree.
#[derive(Debug, Clone, Reflect)]
struct TreeNode {
    pos: Point2,
    /// `None` for the root only.
    parent: Option<u32>,
    /// Path cost from the start to this node.
    cost: f32,
    children: Vec<u32>,
}

/// An RRT* search for one start/goal pair.
///
/// The tree only ever improves: `best_cost` is monotone non-increasing over
/// iterations, which is what makes the algorithm a good anytime task.
#[derive(Debug, Reflect)]
pub struct RrtStar {
    world: World,
    params: RrtParams,
    start: Point2,
    goal: Point2,
    tree: Vec<TreeNode>,
    /// Node closing the best path found so far.
    best_goal: Option<u32>,
    /// Cost of the best path found so far, infinite until one is found.
    best_cost: f32,
    iterations: u32,
    rng: Rng,
    /// Reused between iterations to keep the search allocation-free.
    scratch_near: Vec<u32>,
    scratch_stack: Vec<u32>,
}

impl RrtStar {
    /// Starts a search rooted at `start`. An unreachable or blocked `start`
    /// simply never grows a tree; the caller sees "no path" and the anytime
    /// quality floor drops the result.
    pub fn new(
        world: World,
        mut params: RrtParams,
        start: Point2,
        goal: Point2,
        seed: u64,
    ) -> Self {
        if params.gamma <= 0.0 {
            params.gamma = world.rrt_star_gamma();
        }
        let mut planner = Self {
            world,
            params,
            start,
            goal,
            tree: Vec::new(),
            best_goal: None,
            best_cost: f32::INFINITY,
            iterations: 0,
            rng: Rng::new(seed),
            scratch_near: Vec::new(),
            scratch_stack: Vec::new(),
        };
        planner.reset(start, goal, seed);
        planner
    }

    /// Restarts the search on a new problem, keeping the capacity the previous
    /// job grew: after the first job the planner asks the allocator for much
    /// less.
    pub fn reset(&mut self, start: Point2, goal: Point2, seed: u64) {
        self.start = start;
        self.goal = goal;
        self.tree.clear();
        self.tree.push(TreeNode {
            pos: start,
            parent: None,
            cost: 0.0,
            children: Vec::new(),
        });
        self.best_goal = None;
        self.best_cost = f32::INFINITY;
        self.iterations = 0;
        self.rng = Rng::new(seed);
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
        self.start.distance(self.goal)
    }

    /// Normalized quality in `0.0..=1.0`: how close the best path is to the
    /// straight-line lower bound. 0.0 means no path yet, 1.0 means the path is
    /// as short as the world allows.
    pub fn quality(&self) -> f32 {
        if !self.has_solution() {
            return 0.0;
        }
        (self.lower_bound() / self.best_cost).clamp(0.0, 1.0)
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
    pub fn write_path(&self, out: &mut [Point2; MAX_WAYPOINTS]) -> Option<u32> {
        let goal_node = self.best_goal?;
        let mut chain = Vec::new();
        let mut cursor = Some(goal_node);
        while let Some(index) = cursor {
            let node = &self.tree[index as usize];
            chain.push(node.pos);
            cursor = node.parent;
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
                if self.world.is_free_segment(chain[at], chain[candidate]) {
                    next = candidate;
                }
            }
            at = next;
        }
    }

    /// One RRT* iteration: sample, steer, choose the cheapest parent, rewire
    /// the neighborhood, then check whether the new node closes a better path.
    fn step(&mut self) {
        let sample = self.sample();
        let nearest = self.nearest(sample);
        let from = self.tree[nearest as usize].pos;
        let new_pos = steer(from, sample, self.params.step_size);
        if !self.world.is_free_segment(from, new_pos) {
            return;
        }

        let radius = self.near_radius();
        let mut near = core::mem::take(&mut self.scratch_near);
        near.clear();
        for (index, node) in self.tree.iter().enumerate() {
            if node.pos.distance(new_pos) <= radius {
                near.push(index as u32);
            }
        }

        // Choose the parent that gives the cheapest path to the new node.
        let mut parent = nearest;
        let mut cost = self.tree[nearest as usize].cost + from.distance(new_pos);
        for &index in near.iter() {
            let candidate = &self.tree[index as usize];
            let candidate_cost = candidate.cost + candidate.pos.distance(new_pos);
            if candidate_cost < cost && self.world.is_free_segment(candidate.pos, new_pos) {
                parent = index;
                cost = candidate_cost;
            }
        }

        let new_index = self.tree.len() as u32;
        self.tree.push(TreeNode {
            pos: new_pos,
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
            let (neighbor_pos, neighbor_cost) = {
                let neighbor = &self.tree[index as usize];
                (neighbor.pos, neighbor.cost)
            };
            let rewired_cost = cost + neighbor_pos.distance(new_pos);
            if rewired_cost < neighbor_cost
                && !self.is_ancestor(index, new_index)
                && self.world.is_free_segment(new_pos, neighbor_pos)
            {
                self.reparent(index, new_index, rewired_cost);
            }
        }
        self.scratch_near = near;

        // Does the new node close a better path?
        let to_goal = new_pos.distance(self.goal);
        if to_goal <= self.params.goal_threshold
            && self.world.is_free_segment(new_pos, self.goal)
            && cost + to_goal < self.best_cost
        {
            self.best_cost = cost + to_goal;
            self.best_goal = Some(new_index);
        }
        // Rewiring may have shortened the current best path too.
        if let Some(goal_node) = self.best_goal {
            let node = &self.tree[goal_node as usize];
            self.best_cost = self.best_cost.min(node.cost + node.pos.distance(self.goal));
        }
    }

    /// A random point of the world, biased toward the goal.
    fn sample(&mut self) -> Point2 {
        if self.rng.next_f32() < self.params.goal_bias {
            return self.goal;
        }
        Point2::new(
            self.rng.next_f32() * self.world.width,
            self.rng.next_f32() * self.world.height,
        )
    }

    /// Index of the tree node closest to `point`. Linear on purpose: a real
    /// planner would index the tree, but a flat scan keeps the example short.
    fn nearest(&self, point: Point2) -> u32 {
        let mut best = 0u32;
        let mut best_distance = f32::INFINITY;
        for (index, node) in self.tree.iter().enumerate() {
            let distance = node.pos.distance(point);
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
        (self.params.gamma * (n.ln() / n).sqrt()).min(self.params.step_size)
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
                let node = &self.tree[child as usize];
                if protected[child as usize]
                    || node.cost + node.pos.distance(self.goal) <= self.best_cost
                {
                    keep[child as usize] = true;
                    stack.push(child);
                }
            }
        }
        self.scratch_stack = stack;

        let mut remap = vec![u32::MAX; self.tree.len()];
        let mut kept = Vec::with_capacity(self.tree.len());
        for (index, node) in self.tree.iter().enumerate() {
            if keep[index] {
                remap[index] = kept.len() as u32;
                kept.push(TreeNode {
                    pos: node.pos,
                    parent: node.parent,
                    cost: node.cost,
                    children: Vec::new(),
                });
            }
        }
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
fn steer(from: Point2, to: Point2, step_size: f32) -> Point2 {
    let distance = from.distance(to);
    if distance <= step_size {
        return to;
    }
    let ratio = step_size / distance;
    Point2::new(
        from.x + ratio * (to.x - from.x),
        from.y + ratio * (to.y - from.y),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    const START: Point2 = Point2::new(0.5, 0.5);
    const GOAL: Point2 = Point2::new(9.5, 9.5);

    fn planner(seed: u64) -> RrtStar {
        RrtStar::new(World::depot(), RrtParams::default(), START, GOAL, seed)
    }

    #[test]
    fn segment_collision_is_detected() {
        let world = World::depot();
        // Straight through the pillar at (3, 3).
        assert!(!world.is_free_segment(Point2::new(1.0, 1.0), Point2::new(5.0, 5.0)));
        // Along the free bottom edge.
        assert!(world.is_free_segment(Point2::new(0.2, 0.2), Point2::new(0.2, 9.8)));
        // Endpoints out of bounds.
        assert!(!world.is_free_segment(START, Point2::new(11.0, 0.5)));
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
    /// than skipped.
    #[test]
    fn published_path_is_valid_at_every_stop_point() {
        let world = World::depot();
        let mut longest_tree_path = 0;
        for (seed, gamma) in (1..40u64).flat_map(|seed| [(seed, 0.0f32), (seed, 3.0)]) {
            let params = RrtParams {
                gamma,
                ..Default::default()
            };
            let mut planner = RrtStar::new(World::depot(), params, START, GOAL, seed);
            planner.grow(400);
            for _ in 0..24 {
                planner.grow(256);
                let mut waypoints = [Point2::default(); MAX_WAYPOINTS];
                let Some(len) = planner.write_path(&mut waypoints) else {
                    panic!("seed {seed}: the shortcut path did not fit");
                };
                longest_tree_path = longest_tree_path.max(planner.tree_path_len());
                assert!(len >= 2, "a path has at least a start and a goal");
                assert_eq!(waypoints[0], START);
                assert_eq!(waypoints[(len - 1) as usize], GOAL);
                for pair in waypoints[..len as usize].windows(2) {
                    assert!(
                        world.is_free_segment(pair[0], pair[1]),
                        "seed {seed}: published path crosses an obstacle"
                    );
                }
                // The shortcut only removes waypoints it can bypass in a
                // straight free line, so it never lengthens the path.
                let published: f32 = waypoints[..len as usize]
                    .windows(2)
                    .map(|pair| pair[0].distance(pair[1]))
                    .sum();
                assert!(
                    published <= planner.best_cost() + 1e-3,
                    "seed {seed}: shortcut path {published} longer than the cost {}",
                    planner.best_cost()
                );
            }
        }
        assert!(
            longest_tree_path > MAX_WAYPOINTS,
            "the tree path never outgrew MAX_WAYPOINTS, so the shortcut was never exercised"
        );
    }

    /// A gamma below the value the free area implies shrinks the rewiring
    /// neighborhood, and refinement then converges to a worse path.
    #[test]
    fn derived_gamma_beats_an_arbitrary_one() {
        let world = World::depot();
        let derived = world.rrt_star_gamma();
        assert!(
            (12.0..13.0).contains(&derived),
            "gamma for the depot map should be near 12.4, got {derived}"
        );

        let cost_at = |gamma: f32| {
            let mut total = 0.0;
            for seed in 1..40u64 {
                let params = RrtParams {
                    gamma,
                    ..Default::default()
                };
                let mut planner = RrtStar::new(World::depot(), params, START, GOAL, seed);
                planner.grow(400);
                for _ in 0..24 {
                    planner.grow(256);
                }
                total += planner.best_cost();
            }
            total
        };
        assert!(
            cost_at(0.0) < cost_at(3.0),
            "the derived gamma should refine to a shorter path than a small one"
        );
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
