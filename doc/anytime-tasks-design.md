# Anytime Tasks for copper-rs — Design

Status: proposal, v1 scope
Scope: this document covers the v1 implementation, which is **the anytime interface
and its integration with core copper-rs**. No example task ships in v1. Accelerator
(GPU/FPGA/TPU) support and randomized examples (RRT\*, YOLO, MCTS, particle filters)
are explicitly deferred and get their own design docs once the interface lands.

## 1. Motivation

Some computations are iterative and *interruptible*: they produce a usable result early
and improve it monotonically with more time. RRT\* path planning, iterative MPC,
particle filters, MCTS, and early-exit neural inference all fit this shape. Today these
tasks must either (a) be sized for the worst case and waste budget, or (b) overrun and
break the cycle.

An *anytime task* lets the runtime stop the computation when the budget is spent and
still get a meaningful result. Benefits:

- Predictable on-time output even under load — degraded in quality, not in timeliness.
- Reduced interference with other tasks in the same cycle.
- A path to dynamic resource allocation later (the optimizer surface in §10) without
  changing the task contract.

## 2. Guiding principle

**Reuse first.** Add the smallest amount of new machinery that lets a task implement
"produce a usable result by a deadline, improve until time runs out." Everything else —
replay, logging, clock, message flow, freeze/thaw, config — must use existing
copper-rs mechanisms as-is. Any new mechanism must justify itself against an existing
one we tried and rejected.

**Interface-first, examples later.** v1 delivers the trait, the `AnytimeTask<A>`
adapter, the payload wrapper, the overload policy, and the replay strategy —
nothing else.
No RRT\* example, no YOLO, no accelerator driver, no `CuRng` resource. The interface
must stand on its own merits, be exercised by trivial in-tree tests, and land in
core before any downstream example is designed. Every example that motivated this
design (RRT\*, iterative MPC, particle filters, MCTS, early-exit neural inference)
is a validation target for a *later* PR, not a v1 deliverable.

## 3. What qualifies as an anytime task

A task qualifies iff it has both:

1. A cheap **base** result — the basic computation for the anytime algorithm —
   that is always available before any plausible deadline (the *mandatory part* /
   safety floor).
2. A bounded **refine** step that monotonically (or near-monotonically) improves
   quality, and is small enough that overrun is at most one step.

Motivating shapes (informative; **no example ships in v1**):

| Shape | Base computation | Refinement step | Best-so-far |
|---|---|---|---|
| RRT\* | straight-line / last-cycle feasible path | add a batch of samples; rewire | current lowest-cost path |
| Iterative MPC | last cycle's solution warm-started | one SQP/ADMM iteration | current solution |
| Particle filter | prior particles resampled | one batch of particles | current posterior |
| MCTS | root value at horizon 0 | one batch of rollouts | current best action |
| Early-exit NN | first-block exit head | one more block + its exit head | current head's prediction |

The last one additionally needs the deferred accelerator driver; the others drop onto
the trait in §5 as pure-Rust tasks. Which of these ships first is out of scope for
this doc.

A task does **not** qualify if it has no valid result before the end (a fixed-iteration
solver whose partial state is garbage, a single monolithic call). Those remain normal
non-interruptible tasks.

## 4. Reuse map — what existing copper-rs mechanism covers each need

| Need | Existing mechanism | New? |
|---|---|---|
| Mode-aware clock (live, sim, replay) | `RobotClock` via `CuContext` (`core/cu29_runtime/src/context.rs`) | No |
| Per-cycle period notion | `LoopRateLimiter`, `rate_target_hz` (`core/cu29_runtime/src/curuntime.rs`, `config.rs`) | No |
| Determinism on exact-output replay | `recorded_replay_step` + `SimOverride::ExecutedBySim` (`core/cu29_runtime/src/simulation.rs`, `core/cu29_derive/src/lib.rs`) | No — see §6.2 |
| Determinism on debug replay | `recorded_debug_replay_step` + `SimOverride::ExecuteByRuntime` under simulated `RobotClock` | No — see §6.2 |
| Cross-cycle task state | `Freezable` + keyframes (`core/cu29_runtime/src/cutask.rs`) | No |
| Task config (per-node) | `ComponentConfig` from RON | Add fields, no new mechanism |
| Downstream quality contract | Wrap the payload (`Anytime<O>`) | No `CuMsgMetadata` change |
| Deadline polling inside `process()` | Cooperative loop in the `AnytimeTask<A>` adapter reading `RobotClock` — no preemption | New adapter, no runtime change |
| Seeded randomness (needed for debug-replay of randomized tasks) | Does not exist | Deferred; a separate `CuRng` resource design lands with the first randomized example |

## 5. The `Anytime` trait

`Anytime` is a new user-facing trait. It does **not** become a new runtime concept: the
runtime continues to see only `CuTask`. A small adapter `AnytimeTask<A>` implements
`CuTask` on behalf of any `A: Anytime` (§6). Per-cycle scratch and cross-cycle state
live on `self`, sized once at `new()` — there is no `State` associated type and no
returned local state, so `Freezable` on the user's type persists the algorithm's state
(warm-start MPC, particle-filter posteriors, MCTS trees) with no anytime-specific
machinery.

```rust
// core/cu29_runtime/src/cutask_anytime.rs (new module).

pub trait Anytime: Freezable + Reflect {
    type Input:  CuMsgPayload;
    type Output: CuMsgPayload;
    type Resources<'r>;

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized;

    /// Base computation: the basic result for the anytime algorithm.
    /// Cheap, MUST always fit within any plausible budget, produces a safe rough result.
    /// Reads `input` and resets per-cycle scratch on `self`. After this returns,
    /// `best()` and `progress()` are valid.
    fn base(&mut self, input: &Self::Input, ctx: &CuContext) -> CuResult<()>;

    /// One bounded refinement increment, sized so overshoot is at most one step.
    /// MUST NOT allocate on the RT path. Signature deliberately takes no `ctx`
    /// — `refine` must be pure w.r.t. wall time; the adapter checks the deadline
    /// between calls (§6, §9).
    fn refine(&mut self) -> CuResult<Step>;

    /// Best-so-far. Cheap. Valid after `base` and after every `refine`.
    fn best(&self) -> Self::Output;

    /// Quality marker logged alongside the output. Cheap.
    fn progress(&self) -> Progress;

    // Optional lifecycle hooks, forwarded 1:1 by the adapter. Defaults are no-op —
    // an anytime task keeps every knob a regular `CuTask` has.
    fn start(&mut self,       _ctx: &CuContext) -> CuResult<()> { Ok(()) }
    fn preprocess(&mut self,  _ctx: &CuContext) -> CuResult<()> { Ok(()) }
    fn postprocess(&mut self, _ctx: &CuContext) -> CuResult<()> { Ok(()) }
    fn stop(&mut self,        _ctx: &CuContext) -> CuResult<()> { Ok(()) }
}

pub enum Step { Continue, Converged }
```

### Why these methods specifically

- `base` separated from `refine` so the safety floor is named. The adapter does not
  need to know how cheap "cheap" is; the user guarantees it. The name emphasizes
  that this is the *basic computation* for the anytime algorithm — deliberately
  not called `seed` because that word is overloaded in this design (RNG seeds,
  keyframe seeds).
- **State lives on `self`.** `base` and `refine` mutate the task's own fields —
  arenas, particle buffers, solver workspaces — all preallocated in `new()` with
  fixed capacity sized in config. `Freezable`'s existing `freeze`/`thaw` then
  covers keyframes and cross-cycle warm-start with no anytime-specific hook.
- `refine` is **bounded** by the user, not by the adapter. The adapter checks the
  clock *between* refines; sizing the increment is the user's tuning knob.
  `refine`'s signature carries no `ctx`, so "MUST NOT read the clock" is
  structural, not a comment.
- `best` is the only thing downstream sees; the task's internal buffers stay
  private.
- `progress` is separate from `best` because downstream may decide based on quality
  without inspecting the value.
- Lifecycle hooks mirror `CuTask` and are forwarded verbatim by the adapter.

**Budget is not a trait method.** It is a per-node config field (§6, §8.3) read once
at adapter construction and cached. A default `budget()` on the trait would need
`ComponentConfig` access mid-cycle, which `CuContext` does not carry
(`core/cu29_runtime/src/context.rs:24`); adding it would be a new runtime concept
for a knob none of the motivating shapes in §3 need. If a task later requires
input-dependent budget, add a `fn budget(&self, input: &Input) -> CuDuration` hook
then, against a working baseline.

## 6. Runtime integration — the `AnytimeTask<A>` adapter

There is no separate "anytime driver." A thin adapter `AnytimeTask<A>` implements
`CuTask` on behalf of any `A: Anytime`, so `#[copper_runtime]` and `cu29_derive`
do not need to know that anytime tasks exist. RON names the adapter directly:

```ron
tasks: [
    ( id: "planner",
      type: "cu29::AnytimeTask<tasks::MyPlanner>",
      config: { "budget_us": 8000, "on_overload": "reuse_last" } ),
],
```

```rust
// core/cu29_runtime/src/cutask_anytime.rs

pub struct AnytimeTask<A: Anytime> {
    inner:       A,
    budget:      CuDuration,
    on_overload: OnOverload,
    last:        Option<CachedOutput<A::Output>>,
}

pub enum OnOverload { ReuseLast, Fail }

struct CachedOutput<O> { value: O, progress: Progress, tov: Tov }

impl<A: Anytime> Freezable for AnytimeTask<A> {
    // Budget/on_overload are restored from config on rebuild, not logged.
    // The last-output cache is per-cycle scratch; freezing it is unnecessary for
    // exact-output replay (§6.2) and would bloat keyframes.
    fn freeze<E: Encoder>(&self, e: &mut E) -> Result<(), EncodeError> { self.inner.freeze(e) }
    fn thaw<D: Decoder>(&mut self, d: &mut D)  -> Result<(), DecodeError> { self.inner.thaw(d) }
}

impl<A: Anytime> CuTask for AnytimeTask<A> {
    type Input<'m>     = input_msg!(A::Input);
    type Output<'m>    = output_msg!(Anytime<A::Output>);
    type Resources<'r> = A::Resources<'r>;

    fn new(cfg: Option<&ComponentConfig>, res: Self::Resources<'_>) -> CuResult<Self> {
        let budget      = read_budget_us(cfg)?;             // §11(2): rejects budget > cycle period.
        let on_overload = read_on_overload(cfg)?;
        Ok(Self { inner: A::new(cfg, res)?, budget, on_overload, last: None })
    }

    fn start(&mut self, ctx: &CuContext)       -> CuResult<()> { self.inner.start(ctx) }
    fn preprocess(&mut self, ctx: &CuContext)  -> CuResult<()> { self.inner.preprocess(ctx) }
    fn postprocess(&mut self, ctx: &CuContext) -> CuResult<()> { self.inner.postprocess(ctx) }
    fn stop(&mut self, ctx: &CuContext)        -> CuResult<()> { self.inner.stop(ctx) }

    fn process<'i, 'o>(
        &mut self,
        ctx:   &CuContext,
        input: &Self::Input<'i>,
        out:   &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let Some(in_payload) = input.payload() else {
            return self.emit_skip(out, SkipReason::NoInput);   // §8.3 skip-and-reuse.
        };

        let deadline = ctx.clock.now() + self.budget;

        self.inner.base(in_payload, ctx)?;
        let progress = loop {
            if ctx.clock.now() >= deadline {
                break self.inner.progress();
            }
            match self.inner.refine()? {
                Step::Continue  => continue,
                Step::Converged => break Progress::Converged,
            }
        };

        let value = self.inner.best();
        self.cache_last(&value, progress, ctx);                // private method on the adapter.
        out.set_payload(Anytime { value, progress });
        out.tov = Tov::Time(ctx.clock.now());
        Ok(())
    }
}
```

Three properties to notice:

1. **The clock is read by the adapter only**, never inside `refine`. `refine` takes
   no `ctx`, so "same clock reads → same deadline → same pass count" is structural,
   not a comment. This matters for §6.2.
2. **Overshoot is bounded** by exactly one `refine` call — the only knob the user
   has to size against the budget.
3. **No new codegen.** `cu29_derive` sees `AnytimeTask<A>` as an ordinary `CuTask`
   named by a string path in RON. The macro instantiates it, calls its `process`,
   and forwards its lifecycle hooks — exactly as it does for any hand-written
   `CuTask`.

### 6.1 Why an adapter and not a blanket `impl<A: Anytime> CuTask for A`

A blanket impl over every `Anytime` would force a hard user constraint (the same
type could never carry a hand-written `CuTask` impl — coherence forbids overlap)
and would leave the anytime layer nowhere to own per-cycle wrapper state (budget
cache, last-output slot, overload policy). The adapter puts that state on
`AnytimeTask<A>` where it belongs, keeps the user's type as a plain `impl
Anytime`, and mirrors the shape copper already uses for other reusable task-shape
wrappers. A new task kind, by contrast, would require changes in `cu29_derive`
(execution-plan codegen, `recorded_replay_step`, simulation hooks, monitoring
markers). The adapter needs none of those: it *is* a `CuTask`.

A user that impls `Anytime` for a type opts out of writing a manual `CuTask` for
that same type. This is by design — `AnytimeTask<A>` is the `CuTask`.

### 6.2 Replay — reuse copper's existing two-callback split

copper-rs already ships the two replay flavors anytime tasks need, driven by the
same simulation-callback mechanism every other task uses. **We do not add a new
mode, a new config field, or a new runtime concept.** The user chooses the flavor
at app level by picking which callback to install — exactly as they would for any
other task — and the `AnytimeTask<A>` adapter behaves correctly under both.

The two flavors live in `core/cu29_runtime/src/simulation.rs` and
`core/cu29_derive/src/lib.rs`:

- **Exact-output replay** — `recorded_replay_step`. The callback returns
  `SimOverride::ExecutedBySim`; `process()` is skipped and the logged `CuMsg` is
  copied into the CopperList verbatim.
- **Debug replay** — `recorded_debug_replay_step`. The callback returns
  `SimOverride::ExecuteByRuntime` for internal tasks; recorded external inputs are
  injected but the task's `process()` runs normally under a simulated `RobotClock`.

Both are gated by `sim_mode = true` on `#[copper_runtime]` and selected by the
harness the user runs (`recorded_replay_step` vs. `recorded_debug_replay_step`).
This is the same handle every copper task exposes to replay; the adapter does not
add a second one.

**What happens to an anytime task under each flavor.**

*Exact-output replay.* The adapter never runs. The logged `Anytime<O>` payload —
carrying the recorded `value` and `Progress::Passes { done }` — is injected into
the CopperList. Downstream tasks see byte-identical output and identical
`progress`. Bit-for-bit downstream replay holds with no determinism requirement
on `refine` beyond forward progress in live mode. This is the free path and is
the default for anyone just running a recorded log.

*Debug replay.* The adapter runs, driven by the simulated `RobotClock`. Its loop
reads `ctx.clock.now()`, computes the deadline, calls `base` once, then calls
`refine` until the deadline. Because the simulated clock advances deterministically
and `refine` takes no `ctx` (§5, §9), the same recorded scenario produces the
same pass count and the same convergence trajectory — the user can step through
the algorithm's progression from inside `refine`. This is the "why did the planner
converge to *this* path on cycle 4271?" path.

**Why the anytime interface needs nothing new for this to hold.** Two invariants
already secured by the design carry all the weight:

1. The adapter reads the clock; `refine`'s signature has no `ctx` at all (§9).
   Under a deterministic simulated `RobotClock`, "same clock reads → same
   deadline → same number of `refine` calls."
2. The `Anytime<O>` payload is an ordinary `CuMsg` payload, so exact-output replay
   handles it via the normal recorded-log path with zero anytime-specific code.

**Determinism obligations for debug replay** (an app-level opt-in — the user picks
debug replay by choosing that harness) are covered in §9. Randomized tasks debug-
replaying correctly need a reproducible RNG stream; that machinery (`CuRng`) is
deferred (§7). Non-randomized anytime tasks debug-replay for free.

**What is *not* in v1**: any per-task replay-mode config, any `ctx.is_replay()`
predicate on `CuContext`, any anytime-specific replay callback. If experience with
the interface later shows a real need for per-task granularity, that is a follow-up
design against a working baseline, not a v1 invention.

## 7. `CuRng` — deferred (prerequisite for randomized examples)

Randomized anytime tasks (RRT\*, particle filters, MCTS) need a task-accessible RNG.
No such resource exists in the repo today, and adding one is *not* a v1 deliverable
for the anytime interface. This section is a sketch of the shape it should take
when the first randomized task lands; it is deliberately not shipped now.

Sketch:

```rust
pub struct CuRng { /* ChaCha8Rng or SmallRng */ }

impl CuRng {
    pub fn from_seed(seed: u64) -> Self { /* ... */ }
}

impl rand_core::RngCore for CuRng { /* ... */ }
```

Wired via the existing resource mechanism:

```ron
resources: [
    ( id: "planner_rng", type: "cu_rng::CuRng", config: {"seed": 12345} ),
],
tasks: [
    ( id: "planner", type: "tasks::SomeRandomizedTask", resources: ["planner_rng"], ... ),
],
```

Why defer:

- The v1 anytime interface (trait, adapter, payload, overload, replay strategy)
  can be validated end-to-end with deterministic in-tree tests. It does not need an
  RNG.
- `CuRng` is useful well beyond anytime (Monte Carlo estimators, randomized
  control, dropout at inference) and deserves its own scoping — coupling it to the
  first anytime PR conflates two decisions.
- Exact-output replay (`recorded_replay_step`) works with any live-mode RNG the
  task brings, because downstream only sees the logged payload. Debug replay
  (`recorded_debug_replay_step`) of a randomized task **does** require a
  reproducible RNG stream; whichever randomized example first needs debug replay
  is what forces `CuRng` to land.

A separate design doc will specify `CuRng` — seed logging, per-task instances,
interaction with `Freezable`, and the trade-off between logging the seed vs. the
stream — when the first randomized task is proposed.

## 8. `Anytime<O>` payload, downstream contract, overload, convergence

### 8.1 Payload wrapper

The progress marker travels with the value as a wrapped payload, **not** as a
`CuMsgMetadata` extension. Wrapping leaves the wire format untouched and means
non-anytime tasks pay zero bytes.

```rust
#[derive(Encode, Decode, Reflect, Clone, Debug)]
pub struct Anytime<O: CuMsgPayload> {
    pub value: O,
    pub progress: Progress,
}

#[derive(Encode, Decode, Reflect, Clone, Copy, Debug)]
pub enum Progress {
    Passes { done: u32 },
    Converged,
    // ExitHead { index: u32, of: u32 } — added when §9 accelerator support lands.
    Skipped { reason: SkipReason },
}

#[derive(Encode, Decode, Reflect, Clone, Copy, Debug)]
pub enum SkipReason { NoInput, ReusedLast }
```

### 8.2 Downstream quality contract

Consumer tasks declare their own minimum acceptable quality and fall back to safe
behavior when it isn't met. Example: a controller requires
`Progress::Passes { done >= 3 } | Converged`, otherwise it holds the last command.

The runtime does **not** police this. The contract is local to the consumer, written in
ordinary task code, and therefore replayable.

### 8.3 Overload (`base` itself does not fit, or no input)

Per-task policy in RON config:

```ron
config: { "budget_us": 8000, "on_overload": "reuse_last" }
```

- `reuse_last` (default): the adapter's own `last` slot holds the previous cycle's
  `Anytime<O>`; on skip it re-emits with the **prior `tov`** so downstream can
  detect staleness. `progress` becomes `Skipped { reason: ReusedLast }`.
- `fail`: the adapter returns `CuError`; the existing copper-rs error path takes
  over.

Both are private methods on `AnytimeTask<A>` (`cache_last` / `emit_skip`) —
adapter state, not an extension trait the user has to reason about.

### 8.4 Converged early

`Step::Converged` exits the loop; the cycle finishes early; `LoopRateLimiter::limit`
reclaims the slack automatically. No new mechanism needed — the existing limiter
already does this for any task that finishes under budget.

## 9. RT-path constraints (binding)

The adapter runs on the RT path. Any `Anytime` impl must respect:

- **No allocation inside `refine()` or `best()`.** Growable structures (e.g. an
  RRT\* node arena) must be preallocated at `new()` with a fixed capacity sized in
  config; overflow returns `Step::Converged`.
- **No clock access inside `refine()`.** `refine`'s signature carries no `ctx`,
  so this is structural. Clock is the adapter's concern.
- **No I/O, no mutex contention, no syscalls** inside `refine()`.
- `base()` may allocate at `new()` time; **per-cycle `base()` must reset, not
  reallocate**. Working buffers are owned by the task struct on `self`.
- `just rtsan-smoke` must pass on the in-tree unit-test anytime task shipped with
  the interface. When randomized examples are added later, the same rule applies to
  each of them.

Library calls inside `refine()` are opaque to these rules. The task author is
responsible for ensuring called code obeys them.

**Additional constraints for debug-replay reproducibility** (§6.2 — debug replay is
an app-level opt-in via `recorded_debug_replay_step`; a task that does not care
about debug replay can ignore this section). Beyond the above:

- `base()` and `refine()` must be pure functions of `(state, input, RNG)`. No
  ambient reads (environment, config that changes mid-run, wall-clock). This is
  already required by the RT-path rules above; debug replay simply *depends* on
  it holding.
- If the task is randomized, its RNG state must be reproducible from something the
  task logs — the seed as part of a `Freezable` keyframe or the stream. The
  choice is the task's, but it must be documented. `CuRng` (deferred, §7) will
  standardize this when the first randomized example lands.
- The pass count logged in `Progress::Passes { done }` must exactly equal the
  number of `refine` calls that returned `Continue` in live mode. The adapter
  guarantees this by construction; the pass count is diagnostic under
  exact-output replay and observable under debug replay.

## 10. Phased plan

v1 is the top three phases. Everything below the line is out of scope for the
anytime-interface PR and gets its own design doc when it lands.

| Phase | Deliverable | Exit criterion |
|---|---|---|
| 1 | `Anytime` trait + `AnytimeTask<A>` adapter + `Anytime<O>` payload + `Progress` enum, in `core/cu29_runtime/src/cutask_anytime.rs` | A trivial in-tree counter-style anytime task compiles and runs; recorded `Anytime<O>` payload replays byte-for-byte under `recorded_replay_step` |
| 2 | `budget_us` config field, `on_overload` policy, skip-and-reuse, cache-last | Forced overload re-emits last `best()` with prior `tov`; cycle stays on time; exact-output replay holds through the overload |
| 3 | Verify `AnytimeTask<A>` composes correctly with the existing `recorded_debug_replay_step` callback (deterministic simulated `RobotClock` → identical pass counts → identical `refine` trajectory) | A deterministic in-tree anytime task under debug replay reproduces the recorded pass count and `best()` output on every cycle |
| — | — | — |
| Deferred (separate doc) | `CuRng` resource (§7) — needed by any randomized task that must reproduce under debug replay | — |
| Deferred (separate doc) | RRT\* example end-to-end (preallocated arena, downstream quality gate) | — |
| Deferred (separate doc) | Optional `#[anytime]` macro sugar (`base!` / `refine_loop!` / `best!` lowering to the trait impl) | — |
| Deferred (separate doc) | Accelerator driver — predict-then-dispatch, naive wall-clock between stages first, no cost-hint until measurement requires it | — |
| Deferred (separate doc) | YOLO early-exit example on the accelerator driver | — |
| Deferred (separate doc) | Optimizer hook surfacing per-task pass-count distributions to a future scheduler | — |

Deliberately not in v1: any example task; `CuRng`; macro sugar before the trait API
is exercised; accelerator path; cost-hint / cost-model machinery (correctness-adjacent,
smells like dynamic scheduling); cross-task budget allocation.

## 11. Risks and open questions

1. **`reuse_last` and `tov`.** Re-emitting with the prior `tov` lets downstream detect
   staleness; re-emitting with the current `tov` would hide it. Decision: prior `tov`.
   This is a per-task contract worth nailing down before Phase 2.
2. **Budget vs cycle period.** A `budget_us` larger than the cycle period derived from
   `rate_target_hz` is a config bug; reject it at startup with a clear error. Tiny
   validation pass at app build time.
3. **Determinism surface area of debug replay.** Running an anytime task under
   `recorded_debug_replay_step` pushes correctness obligations onto the task author
   (pure `refine`, reproducible RNG, deterministic simulated clock reads). The
   adapter cannot check these statically. Mitigation: document them clearly
   in the trait docstring; write an in-tree test that exercises a non-deterministic
   task under debug replay and asserts it diverges (i.e. we catch misuse, not that
   we prevent it). Exact-output replay is unaffected — it imposes no such
   obligations.
4. **Harness choice is the user's, per copper convention.** v1 does not add a
   per-task replay-mode config. The user selects exact-output vs. debug replay by
   choosing which callback the harness installs, the same way they would for any
   other task. If practice later shows a real need for per-task granularity, that
   is a follow-up design against a working baseline.
5. **Macro scope (deferred).** `#[anytime]` sugar, when it lands, must produce
   `impl Anytime` and not interact with `#[copper_runtime]`. Keep the two macros
   independent.
6. **Refine granularity.** Too fine: per-step overhead dominates. Too coarse:
   overshoot wastes budget. v1 leaves this to the task author; a future optimizer
   surface can tune it later.
7. **Library calls inside `refine()`** are opaque to the RT-path rules in §9. A
   structural lint to flag unbounded work inside an increment is desirable but out
   of scope for v1.

## 12. Non-goals

- Mid-kernel preemption.
- Async accelerator pipelining (overlap readback of stage *k* with compute of *k+1*).
- Cross-task automatic budget allocation.
- Any change to `CuMsgMetadata` or the on-wire `CuMsg` format.
- Any new runtime concept visible to `#[copper_runtime]` or `cu29_derive`.
- Any example task in the same PR as the interface. Examples land in follow-up PRs
  once the interface is on `master`.
