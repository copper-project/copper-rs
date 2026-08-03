# cu-rrt-star: an anytime RRT* path planner for Copper

An RRT* planner (Karaman and Frazzoli) packaged as a Copper anytime task.
`base()` grows the tree until it has a first path and publishes it; every
`refine()` runs one more block of iterations and republishes only when the
path got shorter. The RON `anytime:` policy — not the task — decides how many
refinement quanta run.

### Input and output

- Input: `cu_rrt_star::PlanRequest` — start, goal and the RNG seed of the job.
- Output: `cu_rrt_star::PlanPath` — up to `MAX_WAYPOINTS` (32) waypoints.
  Every consecutive pair of waypoints is collision free, so the path can be
  driven as published, whichever stop point the policy picked. `cost` is the
  RRT* tree path cost, an upper bound on the distance actually driven.

### Usage

```ron
(
    id: "planner",
    type: "cu_rrt_star::RrtStarPlanner",
    config: {
        "base_iterations": 400,
        "block_iterations": 256,
        "gamma": 0.0,
    },
    anytime: (
        max_refines: 16,
        time_budget_ms: 30.0,
        max_stall: 4,
        quality_floor: 0.05,
    ),
),
```

### Configuration

- `base_iterations` (default 400): iterations of the base block, aiming at a
  first path.
- `block_iterations` (default 256): iterations of one refinement quantum.
- `step_size` (default 0.8): longest edge added in one extension, in meters.
- `goal_bias` (default 0.05): probability of sampling the goal.
- `goal_threshold` (default 0.5): a node this close to the goal closes a path.
- `gamma` (default 0.0): rewiring radius constant; `0.0` derives it from the
  map, which is the value RRT* needs to converge to the optimum.
- `prune_interval` (default 512): branch-and-bound prune every N iterations;
  0 disables pruning.
- `max_nodes` (default 4000): hard cap on the tree size.

### Anytime policy

All the knobs of the RON `anytime:` block apply: `max_refines`,
`time_budget_ms`, `max_age_ms`, `quality_target`, `quality_floor`,
`max_stall`. The reported quality is `straight-line distance / best path
cost` in `0.0..=1.0`: 0.0 means no path yet, 1.0 means the path is as short
as the world allows, so a `quality_target` is portable between maps.

A job that found no path reports quality 0.0, which stays under any
configured `quality_floor`: the node then publishes nothing for that
copperlist.

### Determinism and debugging

The planner uses a seeded xorshift64* generator: the same seed replays the
same tree. A remote debug session sees a small `PlannerDebugState` projection
(iterations, tree size, best cost, published quality) instead of the whole
tree.

### See also

- `examples/cu_anytime_rrt_star`: a closed-loop navigation demo with a Rerun
  viewer.
- `tests/`: the same planner under a quick and a thorough policy, compared
  per copperlist.
