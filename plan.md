# Thread pools, CPU affinity & RT priorities — implementation plan

## Motivation

Thread pools currently live in the Copper resources layer and are minimal: a single
`ThreadPoolBundle` (rayon) exposed as a resource (`id: "threadpool"`), with one knob
(`config: {"threads": N}`), auto-injected whenever any task is `background: true`. The
`parallel-rt` executor is scaffolding only (`parallel_rt.rs` holds stage metadata + an
ordered commit cursor; it does not yet spawn real per-stage worker threads).

We want first-class, named thread pools that carry **CPU affinity** and **real-time
scheduler policy/priority**, consumed by both:

- the **parallel-rt execution pool** (the engine), and
- the **background-task pool(s)**.

### Design biases upheld

- Static / declarative: pools known at compile time, declared in RON, wired by the macro.
- `std`-only: the whole feature is `#[cfg(feature = "std")]`; `no_std`/embedded ignores it.
- RT hot path is sacred: affinity/priority syscalls run **once per worker at thread
  startup**, never on the per-CopperList path.
- Determinism/replay: affinity/priority are performance-only knobs; logged output and
  replay results must be identical regardless of pool config (guarded by the existing
  parallel-rt commit cursor).

## Resolved design decisions

- **Config home:** `runtime.thread_pools: [...]` (sits next to `rate_target_hz`).
- **Affinity semantics:** Spread, one worker pinned per core — `worker[i] → affinity[i % len]`.
  `threads == affinity.len()` gives the canonical one-RT-thread-per-dedicated-core setup.
- **Failure mode:** `Warn + continue` by default (log a `warning!`, fall back to default
  scheduling); `on_error: Strict` opt-in per pool hard-fails at `build()`.

### Canonical config

```ron
(
    runtime: (
        rate_target_hz: 1000,
        thread_pools: [
            ( id: "rt",         threads: 4, affinity: [2,3,4,5], scheduler: Fifo(priority: 80) ),
            ( id: "background", threads: 2, affinity: [0,1],     scheduler: Other ),
            ( id: "vision",     threads: 2, affinity: [6,7],     scheduler: Nice(10) ),
        ],
    ),
    tasks: [
        ( id: "t1", type: "...", background: true ),                 // -> "background"
        ( id: "t2", type: "...", background: (pool: "vision") ),     // -> named pool
    ],
)
```

- `scheduler`: `Other` | `Nice(i8)` | `Fifo(priority: u8)` | `RoundRobin(priority: u8)`
  (omit the field entirely for the default `Other`).
- Reserved ids: `"rt"` (parallel-rt execution pool, never task-bound) and `"background"`
  (default for `background: true`).
- `main.rs` unchanged; optional `.with_thread_pool_override("rt", ThreadPoolSpec{..})`
  builder hook for runtime-detected cores.

## Incremental phases

Each phase compiles green and is independently mergeable; later phases depend only on
earlier ones. Stop for review at each phase boundary.

### Phase 1 — Config schema (additive, inert)

- `config.rs`: add to `RuntimeConfig`:
  - `thread_pools: Vec<ThreadPoolConfig>` (default empty).
  - `ThreadPoolConfig { id, threads, affinity: Option<Vec<usize>>, scheduler, on_error }`.
  - enums `Scheduler { Other, Nice(i8), Fifo{priority}, RoundRobin{priority} }`,
    `OnError { Warn, Strict }`.
- Extend `Node.background` to accept `true` **or** `(pool: "name")`; keep `is_background()`,
  add `background_pool() -> &str` (default `"background"`).
- Validation: unique ids, priority `1..=99`, nice `-20..=19`, reserved ids `rt`/`background`.
- **Verify:** unit tests round-trip RON parse + validation errors. No consumer yet.

### Phase 2 — Pool builder + scheduling primitives (std-only, behind feature)

- New std-only feature `rt-scheduling`; add `core_affinity` + `libc` as optional deps,
  **not** pulled on wasm. (Deviation from original sketch: dropped `thread-priority` in
  favor of `libc` — niceness must be set per-thread via `setpriority(tid)` which
  `thread-priority` does not expose, so using `libc` directly for both niceness and
  `SCHED_FIFO`/`SCHED_RR` keeps the impl consistent and unsafe-surface small. Real-time
  policies are Linux-only; CPU affinity is cross-platform via `core_affinity`.)
- `thread_pool.rs`: `build_pool(spec) -> CuResult<rayon::ThreadPool>`. Affinity/scheduler
  are applied once per worker via `ThreadPool::broadcast` (runs on each worker thread,
  waits, returns ordered results) before any job runs. `Warn` logs and continues; `Strict`
  errors. Feature off ⇒ requested affinity/scheduler ignored with a warning.
- **DONE.** Unit tests: thread count, Warn tolerates unappliable requests, Strict fails on
  invalid affinity, valid-core pinning succeeds. Builds + clippy clean with and without
  `rt-scheduling`; `cu29` gains a passthrough `rt-scheduling` feature.

### Phase 3 — Background pool registry (behavior change, semantics preserved) — DONE

- All declared `runtime.thread_pools` are built at startup (in the generated
  `resources_instanciator`, which has the runtime `CuConfig`) via
  `thread_pool::build_pool` and registered in the `ResourceManager` under a synthetic
  `"threadpool"` registry bundle, indexed by their position in `thread_pools`.
- `ensure_threadpool_bundle` now (a) injects a default `"background"` pool when background
  tasks exist, (b) migrates a legacy `ThreadPoolBundle` resource's `{threads: N}` into that
  pool's thread count, and (c) keeps a marker `"threadpool"` resource bundle for indexing.
- Macro wiring: `CuTaskSpecSet` carries `background_pools`; each background task resolves its
  pool name → registry index (validated; unknown pool ⇒ compile error) and borrows that pool
  instead of the old fixed `BgThreads` slot. `background: true` → `"background"` (unchanged).
- `ThreadPoolBundle` provider kept as a documented legacy alias (no longer built directly).
- `examples/cu_background_task` migrated to `runtime.thread_pools` and shows both the
  `background: true` and `background: (pool: "slow")` forms.
- **Verify:** example runs on both forms; `cu_runtime_matrix` (legacy form) still builds;
  all `cu29-derive` (11) and `cu29-runtime` (152) tests pass; clippy clean. New config tests
  cover default-pool injection and legacy migration.

### Phase 4 — parallel-rt execution pool — DONE

- Spike result: the parallel-rt executor **is** implemented (in the macro codegen) — it
  spawns one `std::thread::scope` worker per process stage. No rayon pool involved, so the
  `"rt"` pool's affinity/scheduler are applied per worker thread, not via `build_pool`.
- Added `thread_pool::apply_current_thread_scheduling(spec, index)` (public, both feature
  states) that pins/sets policy on the *current* thread (Spread by stage index), mirroring
  `build_pool`'s Warn/Strict semantics.
- The generated run loop reads the `"rt"` pool from `runtime.runtime_config.thread_pools`
  and each stage worker applies it at startup. On a Strict pool a failure sets the shutdown
  flag and the worker returns, which cleanly aborts the pipeline (existing disconnect/send
  error handling surfaces the failure).
- **Verify:** `cu_runtime_matrix` builds and runs under `parallel-rt` and under
  `parallel-rt,rt-scheduling`; a parallel mission with an `rt` pool (`affinity:[0,1]`,
  `Nice(5)`) ran correctly with workers pinned. Clippy clean. Example gains an
  `rt-scheduling` passthrough feature.

### Phase 5 — Replay/sim safety, determinism, docs, examples

- Pools degrade to 1 thread / no RT priority in replay/sim; pool config never alters
  replayed output.
- Determinism regression test (parallel-rt output identical across pool configs).
- Optional builder hook `.with_thread_pool_override(...)`.
- Migrate `cu_runtime_matrix`; update skill/docs.
- **Verify:** `just pr-check`.

## Cross-cutting (every phase)

- `no_std` untouched (whole feature `#[cfg(feature = "std")]`).
- RT hot path never does pin/priority syscalls (startup-only).
- Platform support: Linux full; macOS + Windows best-effort with `Warn`.
