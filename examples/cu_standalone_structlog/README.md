## Standalone structured log benchmarks

Profiling binaries for structured logging vs standard logging.

### Justfile commands

- `just profile-standardlog` — run `cargo flamegraph` on `standardlog_perf` with debug info in release.
- `just profile-structlog` — run `cargo flamegraph` on `structlog_perf` with debug info in release.
