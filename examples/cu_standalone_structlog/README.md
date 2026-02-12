# cu-standalone-structlog

## Overview

`cu-standalone-structlog` is a runnable Copper example.

## Prerequisites

- Rust stable toolchain
- Any hardware or services referenced by this example

## Run

```bash
cargo run -p cu-standalone-structlog
```

## Expected Output

Expect successful startup logs and normal task-loop execution without panics.

## Links

- Example path: `examples/cu_standalone_structlog`
- Build/deploy guide: <https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application>

## Additional Notes

### Standalone structured log benchmarks

Profiling binaries for structured logging vs standard logging.

### Justfile commands

- `just profile-standardlog` — run `cargo flamegraph` on `standardlog_perf` with debug info in release.
- `just profile-structlog` — run `cargo flamegraph` on `structlog_perf` with debug info in release.
