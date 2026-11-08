# Temporal system fuzzing

Status: proposal, not implemented. Commands, config, and RPC names below are
illustrative. V0 uses local reflection-based injection; users write no Rust to
configure ordinary campaigns.

## User workflow

```text
TimeTraveler: open log → select interval → Fuzz → choose fields/faults/checks
                                                   ├─ Run → live feedback
                                                   └─ Export → commit → CI
```

The GUI discovers ports, types, and fields from the target schema. It offers
numeric boundaries/noise, enum alternatives, payload absence, delay, hold, and
reordering. Controls save a declarative campaign; text editing is optional.
Bounds inferred from a log are suggestions, not operating assumptions. Coupled
constraints (for example quaternion normalization) use built-in constraint
presets where supported; reflection alone cannot infer physical meaning.

```ron
// Proposed fuzz.ron, separate from copperconfig.ron; exported by the GUI.
(
    version: 1,
    seed: "logs/mission.copper",
    random_seed: 42,
    policy: faults,
    search: coverage_guided,
    budget_ms: 60000,
    horizon_ms: 2000,
    mutations: [
        hold(source: "imu", max_ms: 250),
        noise(field: "imu.accel_z", amplitude: 0.5, correlation_ms: 30),
    ],
    checks: [
        response(
            when: age_gt(source: "imu", ms: 100),
            then: equals(field: "motors.enabled", value: false),
            within: cycles(2),
        ),
    ],
)
```

```bash
# Proposed CLI, using the same engine/config as the GUI.
cargo copper fuzz --target target/debug/robot-fuzz --config fuzz.ron
cargo copper fuzz replay fuzz/cases/stale-imu
```

Per PR: replay committed minimized cases to catch regressions. Nightly: explore
campaigns and publish coverage/findings as artifacts that TimeTraveler can open.
Commit campaigns and portable cases; store large seed logs as pinned artifacts.

## GUI screens and saved work

Add one `Replay | Campaigns` workspace switch. Keep existing replay controls and
`State / Messages / Logs` inspector tabs. Reuse the current
[target launcher](../../../cu-timetraveler/src/app/launcher.rs),
[timeline selection](../../../cu-timetraveler/src/app/replay.rs), and field picker.
`Fuzz selection` and field-context `Add to campaign` open the same campaign draft.
Launch a separate local worker using the selected target; never fuzz the attached
inspection process. Switching workspaces preserves the replay cursor and layout.

| Screen | Displays | Main actions |
| --- | --- | --- |
| Campaigns | Saved campaigns; latest run state, findings, last checkpoint | New, edit/duplicate, run/resume, import/export |
| Setup | Three steps below; plain-language scenario summary | Back/Next, save draft, Run, export for CI |
| Run | Progress, findings table, selected finding summary | Pause, resume, stop, inspect finding |
| Finding / case | Existing replay workspace plus a compact finding panel | Replay failure, minimize, save regression case, return to run |

Setup is a short wizard; saved campaigns reopen as the same editable sections:

| Step | Controls / defaults |
| --- | --- |
| 1. Inputs | Existing target + selected log interval; external ports selected; unsupported fields show why |
| 2. Disturbances | Hold/drop/delay/noise presets; field bounds, units, correlation and duration; small timeline preview |
| 3. Checks & run | Condition → required response → deadline builder; 60s wall budget; scenario horizon separate; Run/export |

Keep search algorithm, RNG seed, resource limits, and internal-state injection
under Advanced. Validate inline; missing target/log shows `Locate`, absent fuzz
support shows `Build compatible target`, absent instrumentation shows `Coverage
unavailable` while allowing property checks. No code editor required. A campaign
without user checks says `Crash/runtime checks only`. Editing an active campaign
creates a new revision; the current run keeps its immutable configuration.

```text
Campaign: IMU dropout       Running       [Pause] [Stop]
Budget       00:24 / 01:00 wall time       [========------------]
Exploration  1,240 scenarios · 52/s · 2,480 virtual seconds
Coverage     +37 edges vs seed · source branch report pending
Checks       3 active · 1 never triggered · 0 inconclusive
Findings     2 unique · 8 occurrences      Saved 4s ago
-------------------------------------------------------------
Check                First failure   Occurrences   Reproducer
stale IMU → off      cycle 14        7             Minimized
finite output        cycle 81        1             Minimizing
```

Numbers are illustrative. Refresh summaries at most twice/second; never stream
every candidate into plots or move the replay cursor automatically. Budget bar
means budget consumed, not percent of bugs found. Show task source branch
percentages only after corpus reporting; label unavailable/pending explicitly.
At completion say `No failures found under these checks/budget`, not `Validated`.
Show stopped/failed/interrupted states and their reason distinctly.

Finding inspection reuses plots, timeline, and inspectors. Add a persistent
`Finding: …` banner, first-failure jump, mutation markers, and check deadline.
Label seed baseline versus mutated candidate explicitly; do not confuse that
comparison with existing Logged/Replayed views. Preserve each recording's own
time axis; align only by saved scenario mapping. `Back to run` restores the run
selection. Opening a finding never pauses exploration.

Manage three objects: campaign = editable search recipe; run = immutable revision
plus exploration history; regression case = exact scenario plus expected check.
Findings belong to runs and are deduplicated. `Save regression case` copies the
confirmed reproducer, including when minimization is unfinished. The campaign
detail has `Runs | Cases`: cases show name, check, last pass/fail, and target
compatibility. Actions: replay selected/all, rename, export/import, delete with
confirmation. A changed target marks old results stale until rechecked. Deleting
a run never deletes independently saved cases. Export resolves seed dependencies
into bundled data or pinned artifact references; no machine-specific paths.

### Pause and resume

Yes: persist search state, not a suspended task stack. Checkpoint every 30 wall
seconds at a completed candidate boundary, and on pause/stop/clean exit. Write
atomically; a crash recovers the last complete checkpoint. Save corpus/findings,
PRNG and scheduler/feedback state, pending minimizations, counters, consumed
budget, and campaign/target/schema/engine identities. Recreate the app for the
next candidate on resume. An interrupted candidate or shrink attempt is retried;
uncheckpointed exploration can be lost. Checkpoints are local run data by default,
not files to commit with regression cases.

`Pause` finishes the current bounded candidate and checkpoints; display `Pausing`
until durable. The watchdog handles a stuck candidate. `Resume` uses the remaining
budget; `Continue +60s` explicitly extends it. `Stop` ends the run but preserves its
checkpoint; continuing a stopped/completed run is an explicit action. On GUI exit,
default to pause/checkpoint; `Keep running` explicitly detaches the worker. A
crash/disconnect leaves `Reconnect` or `Resume from checkpoint`, not a fake paused
state. Relaunch lists resumable runs and last save time.

Resume validates target/config/schema, engine/checkpoint version, and seed data
identities. A mismatch offers `New run from saved corpus`, never silent exact
continuation or merged coverage. V0 uses one deterministic worker; identical
checkpoint plus compatible build must reproduce the next candidates. GUI and
CLI use the same checkpoint format; fresh CI exploration and exact case replay
do not require a previous checkpoint.

## Local execution: no API in the search loop

```text
GUI / CLI ── submit campaign ──► long-lived local fuzz worker
                                   │
              LibAFL chooses scenario edits
                                   ↓
              reset app → inject locally → execute → check → retain
                                   │
GUI ◄──── periodic progress / findings / coverage summaries
```

LibAFL schedules structured inputs and observes feedback; Copper implements a
single generic reflection mutator/interpreter. It does not need a handwritten
mutator for each user message. A candidate is a sequence of edits over seeded
messages and their delivery schedule, not independent random byte strings.
The generated host target runs the engine and app harness in the same worker
process: no message-injection IPC between a separate engine and the application.

At campaign startup, validate the schema and resolve field paths into typed
access steps and mutation operators. During each candidate, borrow the concrete
payload at the simulation callback and apply those steps locally. Cache paths
and descriptors, never references/pointers into an app that will be reset.
Avoid repeated schema queries, string-path parsing, JSON roundtrips, payload
serialization, and RPC per field/message/cycle. Batch progress to the UI; write
full traces for findings and selected corpus entries rather than every candidate.

A fuzz-capable host replay target needs generic injection support once. Changing
campaigns does not require rebuilding it. Existing binaries without that support
need an updated build; edge/source coverage also needs an instrumented build.
Reflection is a reasonable V0 implementation, not a measured throughput promise.
Specialize frequently used access paths only if profiling justifies it.

### Existing pieces and actual gaps

| Existing code | Reuse / missing integration |
| --- | --- |
| [Mutable task reflection](../../core/cu29_runtime/src/reflect.rs) | `reflect_task_mut` exposes state locally; explicit task-state fault mode only |
| [Erased messages](../../core/cu29_traits/src/lib.rs) | `payload_reflect` is read-only; add a host injection boundary for mutable payload/envelope access |
| [Simulation callbacks](../../core/cu29_runtime/src/simulation.rs) | Inject before consumers execute; skip external I/O and run downstream logic |
| [Debug RPC](../../core/cu29_runtime/src/remote_debug.rs) | Navigation/schema/inspection exist; campaign submission/control is new |

`CuStampedData` is reflected as opaque: do not assume its private payload is
writable through ordinary field traversal. Copper must expose the payload and
ToV through the injection boundary. Generate any type dispatch from the static
graph; users do not implement it. Report supported operators per field. Opaque
handles/buffers need built-in handling or an explicit unsupported indication.
This addition belongs only in host fuzz/simulation builds, with no new production
hot-path copies, allocation, serialization, or latency.

### LibAFL boundary: validated spike

Validated with pinned LibAFL 0.16.1 on Rust 1.99.0. The standalone
[prototype](fuzzing-prototype/README.md) lives under `doc/design/fuzzing-prototype`,
outside the main workspace; its builds and generated cases stay under `target/`.

| Contract | Implemented by the fuzz backend, not application users |
| --- | --- |
| [`Input`](https://docs.rs/libafl/latest/libafl/inputs/trait.Input.html) | Private wrapper around shared scenario data: `Clone + Debug + Hash + Serialize + DeserializeOwned` |
| [`Mutator`](https://docs.rs/libafl/latest/libafl/mutators/trait.Mutator.html) + `Named` | Generic edit-plan operators, interpreted against reflected payloads |
| Executor | Reuse `InProcessExecutor` with a closure invoking the local harness |
| Feedback / objective | Reuse standard feedback; implement generic temporal-property feedback |

Do not implement `Input` on `CuMsg<T>` or add LibAFL bounds to `CuMsgPayload`,
tasks, or generated app traits. Float values in edit plans can use IEEE bits for
hashing/persistence; ordinary user float payloads need no `Hash` or `Arbitrary`
derive. The host target needs real `reflect` support, not the disabled-feature
placeholder. The probe used LibAFL features `std` and `serdeany_autoreg`; without
metadata registration its search machinery failed at runtime.

The probe ran LibAFL's scheduler/mutation stage, injected a field through Bevy
reflection into a real `CuMsg<Imu>`, advanced `RobotClock::mock`, and saved a
stale-data property failure despite a normal executor exit. Loading the persisted
scenario reproduced it; a corrected model controller passed. Only private backend
types implement LibAFL traits. The shared `Scenario` uses an `EngineInput` wrapper;
no main-workspace Cargo manifest or core code changed.

Scope: this proves the trait/dependency boundary and local injection, not a full
generated application, coverage guidance, checkpointed search, or throughput.
The probe uses a model controller, constant corpus feedback, and boundary sweeps;
it does not measure random time to discovery. Next gate: the same harness around
a generated Copper graph with real edge instrumentation and reset verification.

LibAFL stays outside Copper core as a dependency, but links into the generated
host fuzz target alongside the app. It is not a separate per-message service.
Shared runtime additions remain generic reflection/simulation hooks, with no
LibAFL types in their public signatures. Users build this host target once;
GUI campaign changes require neither traits nor recompilation.

## Scenarios include history and time

| Quantity | Meaning | Mutation |
| --- | --- | --- |
| `CuMsg.tov` | Measurement validity: absent, point, or range | Freeze, duplicate, shift, malformed timestamp faults |
| Delivery offset | Availability relative to scenario start | Delay, consecutive loss, reorder, burst |
| Task clock | Time observed while the application executes | Cadence jitter, long pause, recovery |

Use [existing clock types](../../core/cu29_clock/src/lib.rs) and respect the
[raw/execution timeline distinction](clock-sync.md). Delivery offsets advance
monotonically; ordinary candidates do not rewind the clock. Device timestamps
need a declared domain mapping. Mutating ToV is different from mutating delivery.

```text
clock      IMU ToV    input / expected reaction
  0ms        0ms     normal
 10ms       10ms     normal
 20–120ms   10ms     hold previous envelope; age grows
140ms       10ms     motors must be off (two cycles after stale detection)
150ms      150ms     fresh data resumes; verify recovery
```

Holding a value with fresh timestamps is a different fault from holding the
entire envelope. Dropped delivery must explicitly mean payload absent, hold old
envelope, or queue. Never silently refresh held ToV. Continue executing consumer
tasks during gaps. V0 injects at cycle/task boundaries under the existing static
plan, not arbitrary OS interleavings. Define equal-time ordering deterministically;
a single-slot source uses explicit latest-wins behavior. Queue/drain semantics
need built-in source-specific support.

| Use case | Mutations / check |
| --- | --- |
| Stale IMU / watchdog | Long gap or frozen ToV → bounded-time motor shutdown |
| Fusion with noncontiguous data | Reordered/duplicate samples → reject bad `dt`, remain finite |
| Unexpected sensor noise | Correlated noise, bias/drift, spikes → bounded output/recovery |
| Pause and reconnect | Variable cadence plus delivery burst → no runaway integrator |
| Sensor disagreement | Cross-stream skew/common-mode noise → declared degraded mode |

Search modes: `monte_carlo` independently samples entire trajectories;
`coverage_guided` retains useful scenarios for further mutations. Input policies:
`plausible` preserves declared constraints; `faults` selects external faults;
`internal_faults` explicitly allows internal message/state injection. Seeded RNG
plus saved realized edits makes scenarios reproducible. Search results are not
estimates of real-world failure probability.

## Checks, reset, and finding replay

GUI checks compile into a local expression plan: field comparisons, age tests,
finite/range checks, and bounded responses in cycles or virtual durations. No
user Rust is required. Unknown/future ToV behavior must be explicit. Count check
triggers; unfinished temporal obligations are inconclusive, not passed. Panics,
worker timeouts, and configured runtime errors are automatic findings; optional
existing monitors can add checks. Reflection does not infer correctness.

Use a long-lived isolated worker, recreate the app for each candidate, and reset
clock/RNG/resources/queues/check state. Require controlled I/O. If clean recreation
cannot reset process-global state, use per-candidate process isolation. An external
watchdog restarts hung/crashed workers. Snapshot reuse is an optimization requiring
reset-equivalence evidence. Preserve check history while replaying the unchanged
prefix; distinguish baseline failures from mutation-induced ones.

Follow generated `recorded_debug_replay_step` in
[the derive crate](../../core/cu29_derive/src/lib.rs): inject boundaries and execute
logic. Exact-output `recorded_replay_step` would hide consequences. After the first
mutation, never restore original log keyframes or original clock timestamps.
Starting from a keyframe requires complete state and correct intra-cycle position;
cold start plus prefix replay is the fallback.

Shrink edits, streams, gap lengths, noise amplitude, and duration while preserving
the check failure and assumptions. Keep required warm-up and actual elapsed time.
Save the realized scenario, check, initial state/prefix, handle contents, and
binary/config/schema/toolchain identities. Export a derived `.copper` log with
candidate outputs/state plus an executable regression case. A seed alone or a log
that only displays recorded outputs is not sufficient. Keep the confirmed finding
even if minimization is interrupted.

Physical-outcome checks need a simulator responding to changed actuator outputs.
Fixed input logs validate software reactions, not whether the robot stays upright.

## Coverage and speed evidence

LibAFL supports [structured inputs](https://aflplus.plus/libafl-book/core_concepts/input.html)
and [feedback](https://aflplus.plus/libafl-book/core_concepts/feedback.html).
Reuse its search/corpus machinery. Select and verify a Rust edge-instrumentation
backend in the prototype; the debug API cannot infer coverage.

| Metric | Collection |
| --- | --- |
| New application edges | Fast instrumented edge map in the local search loop |
| Task source coverage | Replay retained corpus separately with LLVM source coverage |
| Temporal exploration | Gap/age/noise buckets, declared mode transitions, check triggers |
| Throughput | Candidates and virtual seconds per wall second; mutation/reset/task/check costs |

The [root coverage script](../../support/ci/run_coverage.sh) reports lines,
functions, and regions today. [Branch coverage in cargo-llvm-cov requires nightly](https://github.com/taiki-e/cargo-llvm-cov),
using Rust's unstable [coverage options](https://doc.rust-lang.org/unstable-book/compiler-flags/coverage-options.html).
Pin a tested reporting toolchain; label stable line/region fallback accurately.
Map task implementations to scoped functions; count compiler-reported branch
outcomes including uncovered functions, and list exclusions. Shared instances
share implementation coverage; helpers are reported separately. Per-instance
attribution requires extra instrumentation. Edge counts are not source branch
percentages, and coverage is not correctness or MC/DC.

Benchmark identical fixed scenarios: baseline execution, reflection injection,
then specialized injection only if needed. Measure resets separately; they may
cost more than reflection. Compare discovery across multiple RNG seeds, record
first-failure and shrinking times, and verify output equivalence. Virtual-time
jitter tests are not measurements of production scheduling deadlines.

## Campaign control and first milestone

Proposed RPCs: `fuzz.start`, `fuzz.status`, `fuzz.pause`, `fuzz.resume`, `fuzz.stop`,
and `fuzz.open_finding`; start accepts a new recipe, resume a compatible checkpoint
and optional explicit budget extension. Poll status or receive batched summaries;
pause/resume/control are never calls inside the per-candidate loop.
Start returns a campaign ID immediately; status stays responsive while the worker
runs. Stop must work independently of today's placeholder `session.cancel`.
Opening a finding creates an ordinary replay session with its own time axis;
never mutate the user's inspection cursor as a search side effect. Advertise
capabilities and preserve existing clients, task traits, and log behavior. Version
campaign/case artifacts separately; confirm supported release/log versions before
implementing import changes. Keep host tooling out of `no_std` builds.

Proposed product split: open campaign/case schemas, worker API, headless engine,
mutation/check execution, CLI, and CI replay; paid TimeTraveler campaign authoring,
interactive investigation, and polished report UI. Headless runs and saved cases
must work without the GUI. Third-party licenses/notices still apply.

Public contract: versioned recipes, scenarios, checks, cases, and RPCs, shared by
CLI/GUI/CI. Internal details: resolved field access plans, scheduler state, and
LibAFL adapters. The public schema exposes no LibAFL types; a private wrapper
implements `Input`. Backend checkpoints remain version-sensitive implementation
data. A public API does not itself require an open engine or a paid-GUI model;
GUI-only monetization is the proposed product choice, not a technical constraint.
Proposed code placement is annotated inline in the
[prototype source](fuzzing-prototype/src/main.rs): `core/cu29_fuzz_schema` for the
public contract, `core/cu29_fuzz` for the open host engine/private backend, and
generic injection hooks in existing runtime/derive crates. These new crates are
proposed, not created. Runtime never depends on the fuzz engine.

V0 acceptance: configure a stale-IMU campaign entirely in the GUI, find a planted
bug using local reflection, minimize it, open it in TimeTraveler, and export a
case that fails identically in a fresh CI worker. No-mutation execution must match
baseline replay. Report actual throughput before promising discovery times.

## Implementation plan

Decision: CLI and GUI ultimately call the same worker RPC API. The CLI starts a
local worker automatically, or attaches to an existing one; no manual daemon
setup. Bootstrap with an in-process typed campaign service, then add transport
without duplicating campaign logic. RPC carries control/results only; the worker
owns the complete search/reset/injection/check loop.

```text
First:  CLI ─────────► typed campaign service → engine + app harness
Then:   CLI / GUI ─RPC─► same service         → engine + app harness
```

| Step | Deliverable / completion gate |
| --- | --- |
| 1. Local CLI baseline | Generate host injection access; load one seed and declarative scenario; mutate reflected fields/ToV locally. No-mutation replay matches baseline; stale-IMU case fails deterministically. |
| 2. Search and evidence | Integrate LibAFL, temporal operators/checks, edge feedback, shrinking, task source reports. Find/minimize the planted bug; measure reset/mutation/execution costs separately. |
| 3. Durable CLI / CI | Campaign files, case replay/export, pause/resume checkpoints, watchdog, machine-readable summaries. Fresh worker reproduces cases; resume matches uninterrupted candidate sequence. |
| 4. Worker API | Wrap the same service in capability-negotiated RPC, including case replay and lifecycle control; switch CLI to the RPC client. Status/pause work during search; CLI gates still pass. |
| 5. TimeTraveler | Campaigns workspace → setup wizard → run dashboard → existing replay for findings. GUI exports a campaign CI can run and reconnects/resumes saved runs. |

Before UI work, freeze the CLI/API artifact and error contracts. Distinguish
findings, invalid configuration, worker failures, and inconclusive checks in CI;
never emit success solely because the process exited cleanly. Cover stop/crash
recovery and incompatible checkpoints before declaring resume supported. Keep
headless tooling usable independently of the paid GUI; no production
hot-path or released-format changes in this plan.
