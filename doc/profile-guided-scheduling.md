# Profile-Guided Scheduling

Profile-Guided Scheduling (PGS) turns one representative Copper run into ranked,
validated execution plans. It is an offline workflow: profiling and search add no
work to the runtime hot path.

## 1. Record a representative run

Enable CopperList logging and run the application under the workload it must
handle. The unified log records the effective configuration, mission, process
timestamps, and message delivery needed by the optimizer.

The source named by a latency chain must retain payload presence in the log. A
terminal sink is associated with the firing of its directly connected producer,
so that producer must retain payload presence too. For handle-backed payloads,
`handle_content: none` retains the message metadata and handle presence without
recording the referenced contents.

## 2. Describe the scheduling contract

Create `schedule.ron` beside the application:

```ron
(
    chains: [
        (
            id: "control",
            source: "camera",
            sink: "actuator",
            deadline_ms: 20,
        ),
    ],
    sources: [
        (
            task: "camera",
            period_ms: 10,
        ),
    ],
    cpus: [2, 3, 4, 5],
    max_in_flight: 4,
    headroom: 0.2,
    worker_policy: Fair,
)
```

`chains` defines the end-to-end deadlines. `sources` defines required delivery
periods. `cpus` is the exact set of logical CPUs available to generated workers.
`max_in_flight` bounds the pipeline depth explored by PGS. `headroom` reserves a
fraction of each deadline and worker window for timing variation.

For `Fifo` and `RoundRobin`, the configured priority is the base worker priority.
PGS may use the next priority for short-deadline work and the following priority
for the dispatcher.

## 3. Generate candidates

Use the application's typed logreader executable:

```sh
cargo run --release --bin robot-logreader -- \
    logs/robot.copper optimize-schedule \
    --contract schedule.ron \
    --output target/pgs \
    --candidates 3
```

For a log containing several recorded runs, add the global `--run <index>` option.
The selected run supplies the effective configuration and mission; there are no
parallel configuration or mission overrides for PGS.

The output directory contains:

- `contract.ron`: the normalized contract used for this search.
- `profile.ron`: costs, firing patterns, source delivery, and measured chains.
- `plan-N.plan.ron`: a standalone, validated `CuPlan`.
- `plan-N.config.ron`: the recorded configuration with that exact plan selected.
- `predictions.ron`: model output for every candidate.
- `report.txt`: the same readable ranking printed by the command.

Candidates are deterministic for identical inputs. Their score first preserves
source delivery, then minimizes deadline violations, use of configured headroom,
total normalized chain latency, and maximum worker load.

## 4. Build and measure a candidate

Point `#[copper_runtime(config = "...")]` at one emitted candidate configuration
and rebuild the application. This is an explicit compile-time choice: Copper
validates the saved plan against the static graph and generates that schedule.

Record a fresh run, then compare prediction and measurement with the same typed
logreader:

```sh
cargo run --release --bin robot-logreader -- \
    logs/baseline.copper optimize-schedule \
    --contract schedule.ron \
    --output target/pgs \
    --measure plan-1=logs/plan-1.copper
```

This adds `measurements.ron`, appends the measured ranking to `report.txt`, and
writes `selected.plan.ron` for the best measured baseline or candidate. Measured
ranking uses delivered source rate and p99 chain latency; deadline misses remain
visible in every chain row.

PGS rejects measurements from a different mission or workload graph. The
signature covers task types and kinds, component parameters, resources,
background and anytime policy, and graph connections.

## Reference workflows

`examples/cu_parallel_mandelbrot` is the controlled soundness fixture: its
deterministic compute graph exposes prediction and scheduling regressions without
device noise. It is not representative robotics performance evidence.

`examples/cu_flight_controller` demonstrates the realistic workflow on the
deployed companion-compute runtime. Its PGS contract covers the compute graph
from ZED input to ViTFly command output. The MCU runtime is outside PGS scope and
is not changed. The simulator's serialized one-CopperList stepping can check
wiring, but candidate evaluation and selection require fresh deployed-compute
recordings.
