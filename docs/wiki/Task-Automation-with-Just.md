# Task Automation with just

Copper uses [`just`](https://github.com/casey/just) to keep common tasks repeatable. `just` looks for the nearest `justfile`, so run it from the project directory you want to work in.

## Install

Follow the installation instructions in the [`just` README](https://github.com/casey/just#installation) for your platform.

## Discover commands

```bash
just -l
```

## Visualize topology and schedule

Run `just dag` to see the declared task-graph topology. Run `just plan` to produce and open `plan.svg` with the exact generated per-CopperList process schedule, including separate serial and `parallel-rt` projections.

For mission or conditional-feature configs:

```bash
just plan mission=autonomous
just plan features=camera,mock
just plan-log
just plan log=logs/my-robot.copper bin=my-robot-logreader
```

The plan's equal-width columns show order, not elapsed time. Serial uses one main-executor lane; parallel staggers in-flight CopperLists across generated stage workers. Background gateway arrows feed the same named-pool worker lanes in both views, so pool capacity and possible overlap remain visible. Resource bindings are annotations; they are not scheduler reservations.

With `log=...`, the app-specific logreader adds typical and slowest CopperList timelines. Task segments are packed back-to-back but remain proportional to recorded `process_time` duration, and hover identifies every segment. Carried-forward message slots outside the current execution cluster are excluded so stale timestamps cannot flatten the chart. `just plan-log` discovers an example/project's default local log, typed logreader, required features, and recorded mission automatically. Shared declared resources whose intervals overlap are flagged as potential contention, not proven waiting. Serialization is not timestamped; grey residual gaps remain explicitly unclassified because they can also include rate limiting, scheduling, and I/O.

## Where to run `just`

- Repo root: CI and lint helpers.
- Examples:
  - `examples/cu_caterpillar`
  - `examples/ros_caterpillar`
  - `examples/ros_zenoh_caterpillar`
  - `examples/cu_rp_balancebot`
  - `examples/cu_elrs_bdshot_demo`
  - `examples/cu_standalone_structlog`
- Components:
  - `components/sources/cu_ads7883`
  - `components/sinks/cu_rp_sn754410`
  - `components/payloads/cu_ros_payloads`
- Support utilities:
  - `support` (general deploy/sdcard)
  - `support/docker` (build/run dev containers)
