# App supervision helpers

Status: proposal. All APIs below are proposed; examples show their intended shape.
Existing lifecycle methods and manual iteration remain supported.

Coppernic currently owns a synchronous mission supervisor beside a Tokio runtime.
It repeats paced iteration loops for idle, camera startup, recording, and draining,
and uses shared flags to request mission changes. These helpers would let Copper
own that machinery while applications retain their session and device policies.

## 1. Managed iteration loop

Add `run_until` to the existing `CuAppLifecycle<S, L, A, Running>` wrapper:

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
- Uses the App's configured `rate_target_hz` and existing `LoopRateLimiter`;
  an absent rate means no pacing. Each call starts a fresh pacing schedule.
- Executes the same generated plan as `run_one_iteration()`, including parallel
  dispatch when configured. Reuses the existing loop's panic/error handling.
- Returns `Err(CuError)` on execution failure. A stop request never interrupts a
  task mid-call; response time includes the current iteration and pacing wait.
- The predicate must be quick and nonblocking. The helper adds no heap allocation
  per iteration. Repeated calls resume iteration; they do not restart tasks.

This is the first implementation target. It needs no Tokio dependency, new RON
fields, or feature flag. Its basic synchronous contract should support `no_std`;
host signal handling remains gated by existing features.

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

## 3. Typed mission supervisor

Build on the runner with statically registered mission factories. In this sketch,
`Mission` is an application enum and `Session` holds application-owned parameters;
the two `build_*` functions return their respective generated App lifecycle types.

```rust
let supervisor = CuMissionSupervisor::<Mission, Session>::builder()
    .mission(Mission::Idle, |ctx| build_idle_app(ctx.log_path()))
    .mission(Mission::Recording, |ctx| {
        build_recording_app(ctx.params(), ctx.log_path())
    })
    .command_capacity(4)
    .spawn()?;

// Proposed async adapter, called from management:
let ticket = supervisor.control()
    .request_transition(Mission::Recording, session)?;
ticket.wait().await?;
```

Expected behavior:

- Factories are registered before spawn and dispatched through generated/static
  typing, allowing different generated App types. Each build receives its own
  parameters and log path; application factories wire those into resources.
- The Copper owner serializes stop/drain → drop → build → start. Exclusive
  resources are released before the next factory runs. No hot graph mutation.
- Queue admission is nonblocking and fallible: full queue returns an error.
  Acceptance means queued; ticket completion means the target is ready, or failed.
- Accepted requests run in order. Repeated requests, including for the current
  mission, rebuild it; stop/restart cannot disappear between two ticks.
- Dropping a ticket does not cancel an accepted transition. Failure completes that
  ticket with an error and leaves the supervisor without a running mission;
  retries/fallback missions are explicit application requests.
- Session directories and log identities are chosen per build; a rebuild must not
  truncate a previous run. Unknown/unregistered mission variants are rejected.

Keep this in an optional helper layer. Fleet/session rules stay in the application.
The adapter for awaiting tickets is optional; synchronous completion is also needed.

## 4. Readiness and draining

Add bounded waiting helpers on a running App. Device-specific actions stay in
the application; the supplied observations must be nonblocking.

```rust
let mut app = App::builder().build()?.start()?;
app.wait_until_ready(Duration::from_secs(5), || cameras.ready())?;

let run_result = app.run_until(|| stop_requested.load(Ordering::Acquire));

cameras.request_eos();
let drain_result = app.drain_until(Duration::from_secs(2), || cameras.drained());
let stop_result = app.stop();

run_result?;
drain_result?;
stop_result?;
```

Both helpers continue paced iterations until the predicate succeeds, shutdown is
requested, an iteration fails, or the monotonic timeout expires. Success means the
predicate passed; timeout/shutdown/failure returns a distinguishable error. A zero
timeout checks the predicate once and runs no iterations. They neither stop nor
drop the App; callers still attempt cleanup after failure, including readiness
failure (omitted from the short example). Waiting cannot preempt a blocked task.

The mission supervisor would use these observations to acknowledge readiness and
finish draining before releasing resources. Camera barriers and EOS logic remain
component/application concerns.

## Implementation order

1. `core/cu29_runtime/src/app.rs` and
   `core/cu29_derive/src/runtime_macro.rs`: share the existing run-loop machinery
   with `run_until`. Verify early exit, configured pacing, errors, and unchanged
   sequential/parallel dispatch; preserve existing lifecycle APIs.
2. Add the optional `std` runner beside the lifecycle wrapper. Verify factory and
   destruction thread identity, repeated stop requests, and failure cleanup.
3. Add readiness/drain waits on the same loop machinery. Verify timeout and cleanup
   behavior before introducing the mission supervisor helper.
4. Implement typed transitions and bounded requests in the helper layer. Verify
   queued stop/restart, resource release ordering, and completion acknowledgements
   using an idle/recording example derived from Coppernic.
