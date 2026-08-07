# cu-rrt-star: an anytime RRT* path planner for Copper

An RRT* planner (Karaman and Frazzoli) packaged as a Copper anytime task.
`base()` grows the tree until it has a first path and publishes it; every
`refine()` runs one more block of iterations and republishes only when the
path got shorter. The RON `anytime:` policy — not the task — decides how many
refinement quanta run.

### Input and output

- Input: `cu_rrt_star::PlanRequest` — the world (up to `MAX_OBSTACLES` (16)
  round obstacles in a rectangle), start and goal. The map travels with the
  job: the planner has no map of its own, so the source may change the map
  between jobs. Internally the algorithm only sees a sampling and clearance
  trait, so the round-obstacle world is one implementation, not a mandate.
- Output: `cu_rrt_star::PlanPath` — up to `MAX_WAYPOINTS` (32) waypoints.
  Every consecutive pair of waypoints is collision free, so the path can be
  driven as published, whichever stop point the policy picked. `cost` is the
  RRT* tree path cost, an upper bound on the distance actually driven.
- Points and bounds are `Point2f` and `BBox2f` from `cu-spatial-payloads`.
  Distances and costs are `cu29-units` lengths in meters; the quality is a
  `Ratio`. The tree keeps its positions in a `Point2fSoa`, so the two scans
  every iteration runs are one vectorized pass.

### 2D and 3D

`RrtStar` is generic over its space, and the dimension comes from the space's
point type: implement `Clearance` and `RrtSpace` with `Point = Point3f` and
the same planner solves 3D jobs, `Point3fSoa` storage and all. `PlanPoint` is
implemented for `Point2f` and `Point3f`; `tests` drives the planner through a
3D room to keep that honest.

The shipped task is 2D because a Copper node names one concrete type in RON:
`PlanRequest`/`PlanPath` carry `Point2f` and `World` is a rectangle of round
obstacles. A 3D node is a second space plus a second task over the same
`RrtStar`.

### Usage

The planner draws its randomness from a `cu_rng::CuRngBundle` resource, so
the node needs a resource and a binding:

```ron
resources: [
    ( id: "planner_rng", provider: "cu_rng::CuRngBundle", config: {"seed": 1} ),
],
tasks: [
    (
        id: "planner",
        type: "cu_rrt_star::RrtStarPlanner",
        resources: { "rng": "planner_rng.rng" },
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
],
```

### Configuration

RON values are plain numbers — meters for lengths, a 0..1 fraction for
`goal_bias`. `RrtParams` holds them as `cu29-units` `Length` and `Ratio`, so
the Rust API cannot mix a length with a count.

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
- `max_nodes` (default 4000): hard cap on the tree size, clamped to
  `MAX_NODES` (4096) — the capacity of the SoA position set. A larger value
  is capped with a warning.

### Anytime policy

All the knobs of the RON `anytime:` block apply: `max_refines`,
`time_budget_ms`, `max_age_ms`, `quality_target`, `quality_floor`,
`max_stall`. The reported quality is `straight-line distance / best path
cost` in `0.0..=1.0`: 0.0 means no path yet, 1.0 means the path is as short
as the world allows, so a `quality_target` is portable between maps.

A job that found no path reports quality 0.0, which stays under any
configured `quality_floor`: the node then publishes nothing for that
copperlist. A copperlist without a request also publishes nothing, and a
request whose start already lies on the goal converges at once with
quality 1.0.

### Determinism and debugging

Randomness comes from the `CuRng` resource (ChaCha8): job N's stream is a
pure function of the resource seed and N, and the job counter is part of the
frozen task state, so replay reruns every job on the stream it used live. A
remote debug session sees a small `PlannerDebugState` projection (iterations,
tree size, best cost, published quality) instead of the whole tree.

### See also

- `examples/cu_anytime_rrt_star`: a closed-loop navigation demo with a Rerun
  viewer.
- `tests/`: the same planner under a quick and a thorough policy, compared
  per copperlist.
