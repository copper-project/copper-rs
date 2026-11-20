# App supervision helpers

Status: API design for the stacked supervision changes. The generated runtime
and runnable example follow in the next two changes.

Coppernic currently owns a synchronous mission supervisor beside a Tokio runtime.
It repeats iteration loops for idle, camera startup, recording, and draining,
and uses shared flags to request mission changes. These helpers let Copper
own that machinery while applications retain their session and device policies.

## 1. Managed iteration loop

Call `run_until` on a running app to iterate until the caller's predicate returns
true or Copper requests shutdown. The caller can then resume iteration or stop the app.

```rust
pub fn run_until(
    &mut self,
    should_stop: impl FnMut() -> bool,
) -> CuResult<CuRunExit>;

pub enum CuRunExit {
    Requested,
    Shutdown,
}
```

User side, with `stop_requested: Arc<AtomicBool>` shared with external management:

```rust
let mut app = App::builder().build()?.start()?;

let run_result = app.run_until(|| {
    stop_requested.load(Ordering::Acquire)
});
let stop_result = app.stop(); // Attempt cleanup even after an iteration error.

let exit = run_result?;
stop_result?;
```

Expected behavior:

- Runs on the calling thread and borrows the running App. Start and stop remain
  explicit; return leaves the wrapper in `Running`, including on iteration errors.
- Checks the predicate and Copper shutdown request before each iteration. If both
  are set, returns `Shutdown`. An already-set request executes no iterations.
- Executes the same generated plan as `run_one_iteration()`, including parallel
  dispatch when configured. Reuses the existing loop's panic/error handling.
- Returns `Err(CuError)` on execution failure. The loop observes stop requests at
  iteration boundaries. A task already in progress runs to completion.
- The predicate must be quick and nonblocking. The helper adds no heap allocation
  per iteration. Repeated calls resume iteration; they do not restart tasks.

The synchronous loop supports `no_std`. Host signal handling uses the existing
`signal-handler` feature. A pipeline finishes the current scheduling cycle and
commits its in-flight CopperLists before returning.

## 2. Owned runner and control handle

An optional `std` helper owns one OS thread and the App lifecycle:

```rust
let runner = CuRunner::spawn(|| App::builder().build())?;
let control = runner.control();

// From Tokio or any other management thread:
control.request_stop();

// From the synchronous owner:
let exit = runner.join()?;
```

Expected behavior:

- The factory runs inside the Copper thread. Only the factory and its captures
  need `Send + 'static`; constructed tasks/resources need no blanket `Send` bound.
- The runner builds, starts, runs, stops, and drops the App on that thread. Reuses
  configured thread affinity and scheduling policy; no second policy schema.
- `spawn` reports OS thread creation failure. `join` reports build/start/run/stop
  failures, retaining both execution and cleanup errors if both occur.
- `request_stop` is nonblocking and idempotent. Submission does not mean the App
  has stopped; `join` acknowledges completed cleanup and resource release.
- Dropping the control handle does not stop the App. Dropping the runner requests
  stop and joins; async callers must perform joining/drop on a blocking thread.
- The helper uses bounded/preallocated signaling. It creates execution ownership,
  while allocator and memory pressure remain shared with the process.

Tokio users can move `runner.join()` into `tokio::task::spawn_blocking`. Copper
itself remains independent of Tokio; a future async adapter can expose completion.

## 3. Switching missions on the caller's thread

For a live app named `App`, the runtime macro generates `AppRunner` and
an `AppMission` enum with variants for the mission IDs in RON. An app named
`Robot` gets `RobotRunner` and `RobotMission`. This is the complete switch in the
camera example, where the declared IDs are `idle` and `recording`.

```rust
let mut missions = AppRunner::builder()
    .with_clock(clock.clone())
    .with_log_path("missions.copper", None)?
    .build()?;
missions.start(AppMission::Idle)?;
let switch_at = clock.raw_now() + CuDuration::from_secs(5);
if missions.run_until(|| clock.raw_now() >= switch_at)? == CuRunExit::Requested {
    missions.transition_to(AppMission::Recording)?;
}
missions.stop()?;
```

The caller chooses the next mission. Copper stops and drops the active graph,
resolves the target's declared resources, constructs its tasks, and calls their
start hooks. The switch returns after those hooks complete. The first version
restarts the entire graph. If stop, construction, or startup fails, the call
returns an error and the same handle can select a fallback. Explicit stop reports
cleanup errors. Dropping the handle attempts cleanup.

Set up the generated runner once. Its builder accepts the app builder's
clock, instance ID, logger, resources, and supported configuration overrides.
`build` produces an idle runner. `start` activates the initial mission.
`transition_to` can also restart the current mission by selecting the same ID.

`with_logger` retains the supplied writer, including on firmware.
`with_log_path` opens one host archive at the requested path. Its section contexts
identify each activation by run ID, mission index, and instance ID. Without an
explicit logger the runner uses the app builder's no-op logger.

`with_resources` accepts a repeatable resource factory. With the corresponding
features enabled, `with_clock_reference` and `with_logstream` take openers
for an owned reference or transport. Copper calls them at each activation after
releasing the previous graph. `with_telemetry_tx` takes a reusable transmitter
factory. Simulation callbacks belong to simulation apps.

## 4. Readiness and draining

Use bounded waiting helpers on a running app. Device-specific actions stay in
the application; the supplied observations must be nonblocking.

```rust
let mut app = App::builder().with_clock(clock.clone()).build()?.start()?;
app.wait_until_ready(CuDuration::from_secs(5), || cameras.ready())?;

let run_result = app.run_until(|| stop_requested.load(Ordering::Acquire));

cameras.request_eos();
let drain_result = app.drain_until(CuDuration::from_secs(2), || cameras.drained());
let stop_result = app.stop();

run_result?;
drain_result?;
stop_result?;
```

Both helpers continue iterating until the predicate succeeds, shutdown is
requested, an iteration fails, or the configured clock's raw monotonic deadline expires. Success means the
predicate passed; timeout/shutdown/failure returns a distinguishable error. A zero
timeout checks the predicate once and runs no iterations. They neither stop nor
drop the App; callers still attempt cleanup after failure, including readiness
failure (omitted from the short example). Waiting cannot preempt a blocked task.
