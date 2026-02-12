# horus-caterpillar

## Overview

`horus-caterpillar` is a runnable Copper example.

## Prerequisites

- Rust stable toolchain
- Any hardware or services referenced by this example

## Run

```
cd examples/horus_caterpillar
cargo run --release
```

## Expected Output

Expect successful startup logs and normal task-loop execution without panics.

## Links

- Example path: `examples/horus_caterpillar`
- Build/deploy guide: <https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application>

## Additional Notes

### Horus caterpillar example

This is a Horus version of the caterpillar chain used for Copper vs ROS comparisons.
It matches the Copper constraints: 17 tasks (1 source + 8 propagate + 8 GPIO sinks) and
16 connections (8 chain + 8 GPIO). Per-message logging is disabled to keep the console
quiet; instead, the app records every message and prints a summary table once per second.

### Notes

- Deterministic execution is enabled via `Scheduler::enable_determinism()`.
- End-to-end latency uses `Instant` (monotonic); it reports src → gpio-7 in the `End2End` row.
- Per-task rows show node tick duration (recv + publish work) for each task.
