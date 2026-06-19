# CuMemMon

Per-task heap-allocation monitor for Copper. Tells you, for every monitored
task: how many bytes were allocated/deallocated during each lifecycle step
(`start` / `preprocess` / `process` / `postprocess` / `stop`), the lifetime
balance, and whether anything leaked at shutdown.

## How it works

`cu29-runtime` ships a `CountingAlloc` global allocator (gated behind
`cu29/memory_monitoring`) that bumps two atomic counters on every `alloc` /
`dealloc`. The `#[copper_runtime]` macro now wraps every generated task step in
a `ScopedAllocCounter` and fans the delta out to all monitors via the
`CuMonitor::observe_alloc` hook. `CuMemMon` collects those deltas, keeps
per-`(component, step)` running totals, and reports them at `stop`.

Without `cu29/memory_monitoring`, the runtime still calls `observe_alloc` but
all deltas are zero. `CuMemMon` notices this once at startup and emits a single
`warning!` so the build configuration is obvious from the log.

## Usage

```toml
[dependencies]
cu-memmon = { version = "*" }
# Required for any non-zero numbers: installs the counting global allocator.
cu29 = { version = "*", features = ["memory_monitoring"] }
```

In `copperconfig.ron`:

```ron
monitor: (
    type: "cu_memmon::CuMemMon",
    // Optional: when true, any non-zero allocation observed during
    // preprocess/process/postprocess emits a one-shot warning. The next
    // process_error() then escalates to Decision::Shutdown.
    config: { "realtime_strict": false },
)
```

## Output

At runtime shutdown you get a per-task line like:

```
cu_memmon: task task balanced (alloc 1024B, dealloc 1024B); process allocated 512B over 60 calls
cu_memmon: task leaky leak=256B (alloc 4096B, dealloc 3840B); process allocated 4096B over 60 calls
```

## Try it

```bash
cargo run -p cu-memmon --example copper_app --features cu29/memory_monitoring
```

The intermediate `Step` task in the example allocates and drops a small `Vec`
every iteration on purpose; lifetime totals should balance.

## Status

Stage 1 (this crate): task-level coverage of `start`/`preprocess`/`process`/
`postprocess`/`stop`. Bridges and the `parallel-rt` execution engine are not
instrumented yet — they will receive the same hook in a follow-up.
