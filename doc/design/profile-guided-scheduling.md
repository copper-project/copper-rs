# Profile-Guided Scheduling

PGS converts a representative recording into ranked, validated `CuPlan` candidates.
Search and scoring run offline; an application explicitly selects a saved plan
at compile time. This preserves static typed wiring and avoids an adaptive scheduler
on the runtime hot path.

## Data flow and choices

```text
recorded run + scheduling contract → measured profile → candidate plans
      → rebuild each candidate → new recordings → measured ranking → selected plan
```

| Stage | Implementation choice / reason |
| --- | --- |
| Input | Select one recorded run; its effective configuration and mission are authoritative, avoiding parallel config overrides |
| Profile | Aggregate fired/skipped operation costs, firing patterns, source delivery, and end-to-end chains from logged process metadata |
| Contract | Deadlines, source periods, exact logical CPU set, maximum in-flight lists, headroom, worker policy |
| Search | Fixed-seed deterministic local search; tune worker placement/pipeline schedule offline |
| Validation | Validate emitted plans against the static graph before code generation |
| Measurement | Re-run candidate binaries and rank delivered source rate and p99 chain latency; predictions do not replace measurements |

The source of a latency chain must retain logged payload presence. A terminal sink
is associated with its directly connected producer's firing, so that producer must
retain presence too. For pooled payloads, `handle_content: none` can retain presence
and metadata without the referenced bytes. No extra profiling pass is added to
ordinary task execution.

## Contract and artifacts

`CuContract` fields:

| Field | Meaning |
| --- | --- |
| `chains` | Named source/sink pairs with `deadline_ms` |
| `sources` | Required `period_ms` per source task |
| `cpus` | Exact available logical CPU IDs |
| `max_in_flight` | Maximum pipeline depth explored |
| `headroom` | Fraction reserved for timing variation in deadlines and worker windows |
| `worker_policy` | Worker scheduling policy; FIFO/RoundRobin priority is the base, with higher priorities available for short-deadline work/dispatcher |

Predicted score preserves source delivery first, then minimizes deadline violations,
headroom consumption, normalized chain latency, and maximum worker load. Identical
inputs produce deterministic candidates. The timing model uses the recorded workload;
it does not establish worst-case execution bounds for unseen inputs or environments.

The application's typed logreader exposes:

```sh
cargo run --release --bin robot-logreader -- \
    logs/baseline.copper optimize-schedule \
    --contract schedule.ron --output target/pgs --candidates 3
```

Use global `--run <index>` when selecting among multiple recorded constructions.
Outputs:

| Artifact | Contents |
| --- | --- |
| `contract.ron`, `profile.ron` | Normalized input contract and measured profile |
| `plan-N.plan.ron` | Standalone validated `CuPlan` |
| `plan-N.config.ron` | Recorded config with that exact schedule selected |
| `predictions.ron`, `report.txt` | Model output and readable ranking |

Point `#[copper_runtime(config = "...")]` at the emitted candidate configuration
and rebuild. After recording it, repeat optimization with
`--measure plan-1=logs/plan-1.copper`. Measurement writes `measurements.ron`, appends
the measured ranking to `report.txt`, and selects the best baseline/candidate as
`selected.plan.ron`. Deadline misses remain visible alongside p99 latency and
source delivery. Repeated rounds can retain separate measurement names.

Measured profiles must match mission and workload graph. The signature includes
task types/kinds, component parameters, resources, background/anytime policy, and
connections while allowing the schedule itself to differ. This prevents selecting
a plan based on a different application workload.

## Code and coverage

| Concern | Pointer |
| --- | --- |
| Contract/profile/signature types and tests | [`planner/pgs.rs`](../../core/cu29_runtime/src/planner/pgs.rs) |
| Profile extraction and message-origin chains | [`cu29_export/src/pgs.rs`](../../core/cu29_export/src/pgs.rs) |
| Deterministic candidate search and prediction tests | [`planner/propose.rs`](../../core/cu29_runtime/src/planner/propose.rs) |
| Measured ranking and graph/mission mismatch checks | [`planner/score.rs`](../../core/cu29_runtime/src/planner/score.rs) |
| Saved plan application/validation | [`explicit_schedule.rs`](../../core/cu29_runtime/src/planner/explicit_schedule.rs), [`schedule.rs`](../../core/cu29_runtime/src/planner/schedule.rs) |
| CLI artifacts, measurements, selected-plan output | [`cu29_export/src/lib.rs`](../../core/cu29_export/src/lib.rs): `run_schedule_optimizer` |
| Controlled scheduling fixture | [parallel Mandelbrot](../../examples/cu_parallel_mandelbrot/README.md), [PGS harness](../../examples/cu_parallel_mandelbrot/src/bin/pgs.rs) |
| Generated application workflow | [project templates](../../support/cargo_cunew/templates) |

Mandelbrot exposes scheduling/prediction regressions with deterministic compute;
it is not representative robotics performance evidence. Measure real sensors,
contention, storage, and deadlines before adopting a candidate.
