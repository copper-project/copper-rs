# cu-runtime-matrix

This example is a focused runtime matrix for:

- `async-cl-io` on and off
- `parallel-rt` on and off
- background tasks on and off
- graph shapes: `1->many`, `many->1`, `many->many`, and bridges

It is intentionally synthetic. The point is to exercise scheduler and logging semantics with deterministic mock-clock tests and a repeatable CPU workload, not to model a robot.

## What It Covers

- `OneToMany` and `OneToManyBackground`
- `ManyToOne` and `ManyToOneBackground`
- `ManyToMany` and `ManyToManyBackground`
- `BridgeFanout` and `BridgeFanoutBackground`

The tests check two things:

- the observed runtime trace matches the exact foreground trace and the sampled-delay contract for background missions
- the normalized CopperList stream is identical across two live runs of the same foreground mission

That second check is the part that makes `async-cl-io` relevant instead of just incidental.
It intentionally ignores volatile wall-clock processing-time metadata and compares the logged message content instead.
Background missions are not covered by that cross-run equality check because their worker scheduling is outside the live-run determinism contract; replay is the place where equivalence is expected.

## Commands

Run the full test matrix from [justfile](/home/gbin/projects/copper/copper-rs.parallel_cpu_io/examples/cu_runtime_matrix/justfile).

Typical commands:

- `just -f examples/cu_runtime_matrix/justfile`
- `just -f examples/cu_runtime_matrix/justfile test-serial`
- `just -f examples/cu_runtime_matrix/justfile test-async`
- `just -f examples/cu_runtime_matrix/justfile test-parallel`
- `just -f examples/cu_runtime_matrix/justfile test-both`
- `just -f examples/cu_runtime_matrix/justfile dag`
- `just -f examples/cu_runtime_matrix/justfile dag ManyToMany`

For quick performance numbers:

- `just -f examples/cu_runtime_matrix/justfile bench-serial`
- `just -f examples/cu_runtime_matrix/justfile bench-async`
- `just -f examples/cu_runtime_matrix/justfile bench-parallel`
- `just -f examples/cu_runtime_matrix/justfile bench-both`
- `just -f examples/cu_runtime_matrix/justfile bench-both all 128 256 0 2048 16 1.0`

`just` with no recipe now runs the four benchmark modes in release mode.
It prints one comparative summary instead of four raw benchmark dumps.
The summary uses `sync+serial` as the baseline and shows deltas for throughput and p99 latency.
Those default runs are duration-gated and execute a synthetic compute kernel inside the active source, task, and bridge nodes, so they are not just measuring an empty scheduler loop.

Benchmark knobs are positional in this order:

- `mission warmup min_iterations busy_spin_iters compute_words compute_rounds min_seconds`

The single-mode `bench-*` commands print actual measured `samples` and wall-clock `secs` for each mission.
That matters when `min_seconds > 0`, because the benchmark keeps running after `min_iterations` until the duration threshold is satisfied.

Background mission numbers need care:

- they measure scheduler loop cost while background work is in flight
- they can sample outputs instead of producing one output per input
- compare background modes against the same background mission, not against the foreground mission

`just dag` now renders all missions on one page by default. Pass a mission name to focus on one.
