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

- New std-only feature `rt-scheduling`; add `core_affinity` + `thread-priority` as optional
  deps, **not** pulled on wasm.
- `thread_pool.rs`: `build_pool(spec) -> CuResult<rayon::ThreadPool>` using rayon
  `spawn_handler`/`start_handler` to pin affinity (Spread) and set policy/priority once per
  worker. `Warn` logs and continues; `Strict` errors.
- **Verify:** unit test thread count + names; affinity/priority asserted best-effort
  (skipped when unprivileged). Build with and without `rt-scheduling`.

### Phase 3 — Background pool registry (behavior change, semantics preserved)

- Build all declared pools at startup into a registry; auto-inject a default `"background"`
  pool when absent (mirrors today's `ensure_threadpool_bundle`).
- `cuasynctask` + macro wiring: resolve pool by `background_pool()` name instead of the
  single `ThreadPoolBundle` slot. `background: true` → `"background"` (unchanged behavior).
- Deprecate `ThreadPoolBundle`-as-resource: keep working as an alias to the `"background"`
  pool for one release, with a deprecation note.
- Migrate `examples/cu_background_task`.
- **Verify:** example runs; existing background tests pass.

### Phase 4 — parallel-rt execution pool

- Confirm the executor's current threading reality; hand the built `"rt"` pool to the
  worker threads it spawns, applying affinity/priority there.
- ⚠️ Risk/scope: if the stage-affine executor isn't actually spawning worker threads yet,
  finishing that is a larger sub-effort. Re-scope after a Phase-4 spike and check in before
  expanding.
- **Verify:** `cu_runtime_matrix` parallel-rt missions produce unchanged output.

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
