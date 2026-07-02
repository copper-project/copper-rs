# {{project-name}} Project

This template bootstraps a single-crate Copper project for quick experiments.

## Layout

- `src/main.rs`: runtime binary.
- `src/logreader.rs`: log reader binary.
- `src/resim.rs`: replay binary with one-shot replay and remote-debug server modes.
- `src/tasks.rs`: sample tasks.
- `copperconfig.ron`: runtime configuration.
- `justfile`: automation helpers like `just log`, `just cl`, `just resim`, `just resim-debug`, and `just dag` (`just rcfg` remains as a compatibility alias).

## Quick start

```bash
cargo run
```

The runtime config lives in `copperconfig.ron`.

To replay a recorded log once:

```bash
just resim
```

To start the replay-backed remote debug server manually:

```bash
just resim-debug
```

## Monitors

Copper runs without a monitor by default. To enable the per-task
heap-allocation monitor, uncomment the `cu-memmon` dependency and the
`memmon` feature in `Cargo.toml`, then add this entry to `copperconfig.ron`:

```ron
monitor: (
    type: "cu_memmon::CuMemMon",
    config: { "realtime_strict": false, "summary_every": 100 },
),
```

Build with `cargo run --features memmon`. See the `cu_memmon` crate README
for tuning knobs and output format.
