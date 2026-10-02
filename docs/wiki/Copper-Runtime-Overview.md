# Copper Runtime Overview

Copper is a deterministic, data-oriented runtime for robotics. You describe your system declaratively, Copper compiles the schedule, and every run is bit-for-bit reproducible.

## Task Graph

The task graph defines your system topology, inter-task communication, and message types. It is expressed in RON and drives how Copper builds the runtime.

See [Copper RON Configuration Reference](Copper-RON-Configuration-Reference) for the full schema.

## Runtime Generator

Copper creates an execution plan from the task graph metadata. It preallocates a "Copper List" so execution stays cache-friendly and deterministic.

Two visualizers answer different questions:

- `just dag` shows topology: components, ports, connections, and resources.
- `just plan` shows the exact generated per-CopperList process order with one scheduler model in both projections. Serial is the main CopperList executor with one worker, including woven anytime refinements; parallel is the staggered generated-stage wavefront. Background nodes appear as poll/dispatch gateways in either projection, with arrows into aligned named-pool worker lanes whose open-ended bars expose work that may persist across later CopperLists. Large in-flight depths show a six-CopperList window (`CL n` through `CL n+5`) while reporting the real configured depth. Repeated configured resource targets are highlighted as potential contention, including background jobs and skew outside the nominal wavefront. The columns are ordinal positions, not measured time.

When a config contains missions, `just plan` stacks every mission by default; use `just plan mission=<id>` to select one. `just plan-log` auto-selects the local example log, typed logreader, required features, and mission recorded by normal `cargo run`; use `just plan log=<path>.copper bin=<typed-logreader>` to select inputs explicitly. Observed task segments are packed back-to-back while remaining proportional to recorded process duration, with carried-forward stale slots excluded. Observed overlap on the same declared resource is a risk signal, not proof of lock contention; serialization, queue wait, wrapper overhead, and CPU/worker identity cannot be isolated from the current log. Multi-Copper plans remain unsupported.

## Zero-Copy Data Logging

Messages between tasks are recorded without copying data. Logs can be replayed exactly, making debugging and certification practical.

## Fast Structured Logging

Structured log strings are interned and indexed at compile time. Runtime logging avoids string construction overhead and keeps hot paths fast.

## SDK Capabilities

- Task interface and lifecycle traits for implementing new algorithms, sensors, and actuators.
- Compile-time runtime generation with deterministic scheduling.
- Log readers that export Copper logs using Rust Serde formats.
- Structured log reader for efficient debug and telemetry data.
- Experimental Python support for offline log analysis and single-task prototyping. See [Python Support](Python-Support).
- A growing set of components and drivers (see the [Copper Component Catalog](https://cdn.copper-robotics.com/catalog/index.html)).
- Deterministic log replay and resimulation.
- A minimal simulation environment for hardware-free testing.
