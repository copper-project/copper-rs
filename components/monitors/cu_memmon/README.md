# CuMemMon

Per-task heap-allocation monitor for Copper. Tells you, for every monitored
task and bridge: how many bytes were allocated/deallocated during each
lifecycle step (`start` / `preprocess` / `process` / `postprocess` / `stop`),
the per-call peak in `process`, the lifetime balance, and whether anything
leaked at shutdown.

## How it works

`cu29-runtime` ships a `CountingAlloc` global allocator (gated behind
`cu29/memory_monitoring`) that bumps two views of every `alloc` / `dealloc`:
a process-wide pair of atomic counters and a pair of per-thread `Cell<usize>`
counters. With the feature on, the `#[copper_runtime]` macro wraps every
task and bridge lifecycle step in a `ScopedAllocCounter` and fans the delta
out to all monitors via the `CuMonitor::observe_alloc` hook. `CuMemMon`
collects those deltas, keeps per-`(component, step)` running totals, emits
a throttled summary line every N copperlists, and reports leaks at `stop`.

`ScopedAllocCounter` snapshots the **per-thread** counters, so under
`parallel-rt` — where worker threads run monitored steps concurrently — a
step's reported delta only includes allocations performed on the same thread
that runs that step. Concurrent work on other worker threads does not
contaminate the number.

With the feature **off**, the macro emits no allocation scopes at all — zero
runtime cost — and the monitor warns at startup if the runtime probe shows
the allocator isn't installed, or on the first summary tick if the allocator
is installed but no scopes have fired (the derive-side feature is off).

Coverage in this release: task `start` / `stop` / `preprocess` / `process` /
`postprocess`; bridge `start` / `stop` / `preprocess` / `process` (rx and tx) /
`postprocess`; and the parallel-rt counterparts of both. So all monitored
lifecycle steps from cu29 1.0.0-rc2 are observed.

## Usage

```toml
[dependencies]
# `memory_monitoring` is a passthrough that turns on both halves of the
# feature — the counting allocator in cu29-runtime and the per-step scope
# codegen in cu29-derive. Without it the monitor compiles and runs but
# reports zeros.
cu-memmon = { version = "*", features = ["memory_monitoring"] }
```

In `copperconfig.ron`:

```ron
monitor: (
    type: "cu_memmon::CuMemMon",
    config: {
        // When true, any non-zero allocation observed during
        // preprocess/process/postprocess latches a violation that is
        // surfaced from the next `process_copperlist` as a fatal CuError —
        // the runtime then shuts down cleanly.
        "realtime_strict": false,
        // Emit a "[mem] task ..." summary every N copperlists. 0 disables.
        "summary_every": 100,
    },
)
```

## Output

Periodic summary lines (every `summary_every` copperlists):

```
[mem] task start=+0B -0B | proc max=+64B over 60 calls | stop=+0B -0B | leak=0B
```

At runtime shutdown:

```
cu_memmon final report: total +3840B / -3840B
cu_memmon: task task balanced (alloc 3840B, dealloc 3840B); process allocated 3840B over 60 calls
cu_memmon: task leaky leak=256B (alloc 4096B, dealloc 3840B); process allocated 4096B over 60 calls
```

Tasks that deallocate more than they allocate within their observed scopes
are surfaced too — usually that means they're freeing memory that was
allocated outside the lifecycle hooks (e.g. a payload passed in from a
sibling task), which is worth knowing but not necessarily a bug.

## Realtime-strict mode

With `realtime_strict: true`, any non-zero allocation in `preprocess`,
`process`, or `postprocess` is latched and surfaced as a `CuError` from the
following `process_copperlist` call — which in practice is the same iteration's
`process_copperlist`, since it runs after every step. The runtime treats that
as a fatal monitor error and shuts the application down, so a violation
reliably trips a CI smoke test regardless of whether any other component
happens to error. Unrelated task errors are not escalated by this mode. When
multiple steps violate in the same iteration the *first* one wins; later
violations are observed but don't overwrite the latched cause.

## Try it

```bash
cargo run -p cu-memmon --example copper_app --features memory_monitoring
```

The intermediate `Step` task in the example allocates and drops a small `Vec`
every iteration on purpose; lifetime totals should balance and the per-call
peak in `process` will read 64 bytes.
