# CuLogMon

## Overview

Compact Copper monitor that emits a one-line statistical snapshot every second via the normal `debug!` / `info!` macros.
It is designed to work both on `std` targets and in bare-metal builds that route logging through `defmt`.

## Interface

Observes runtime state or payload flow for diagnostics and telemetry.

## Configuration

Monitor behavior is controlled by task `config` keys and feature flags.

## Usage

```toml
[dependencies]
cu-logmon = { version = "*", default-features = true } # disable default-features for no_std
```

In `copperconfig.ron`:

```ron
monitor: (
    type: "cu_logmon::CuLogMon",
)
```

For bare-metal targets, disable default features and enable `defmt`:

```toml
cu-logmon = { version = "*", default-features = false, features = ["defmt"] }
```

### Example

Run the built-in demo (std targets):

```bash
cargo run -p cu-logmon --example demo
```

It simulates a 3-task pipeline at ~10 Hz and prints a one-line stat report every second.

### Full Copper runtime sample

There is also a minimal source->task->sink Copper app wired to `CuLogMon`:

```bash
cargo run -p cu-logmon --example copper_app
```

The runtime uses `copperconfig.ron` in this crate and logs to `logs/logmon_copper_app.copper`.

## Compatibility

Availability depends on runtime target and optional UI/logging features.

## Links

- Crate path: `components/monitors/cu_logmon`
- docs.rs: <https://docs.rs/cu-logmon>

## Additional Notes

### What it prints

- Copperlist index and windowed rate (Hz)
- End-to-end latency percentiles (p50/p90/p99 + max) in microseconds
- The slowest task in the last window (p99) and the time spent logging the previous report
