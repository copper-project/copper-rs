# {{project-name}} Project

This template bootstraps a single-crate Copper project for quick experiments.

## Layout

- `src/main.rs`: runtime binary.
- `src/logreader.rs`: log reader binary.
- `src/resim.rs`: replay binary with one-shot replay and remote-debug server modes.
- `src/tasks.rs`: sample tasks.
- `copperconfig.ron`: runtime configuration.
- `src/view.rs`: Rust launcher for the graph and schedule viewers.
- `justfile`: `log`, `cl`, `resim`, `resim-debug`, `graph-view[-log]`, and `schedule-view[-log]` recipes.
{% if pgs_enabled %}
- `schedule.ron`: starter scheduling contract for `src → sink`; tune its timing and CPU placement.
- `src/pgs.rs` and `src/pgs_candidate.rs`: offline PGS workflow and compile-time selected candidate.
{% endif %}

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

Both replay recipes use the `debug-optimized` Cargo profile so replay stays fast while
preserving Copper `debug!` structured logs and debug information.

{% if pgs_enabled %}
## Profile-guided scheduling

Record a representative run with `just pgs-baseline`, then run
`just pgs-optimize candidates=3`. Inspect `target/pgs/report.txt` and the
generated graph and schedule SVGs. Run `just pgs-candidate candidate=1` to
compile and record one candidate, then `just pgs-measure candidates="1"` to
compare measured runs. Candidate numbers are explicit; the selected config is
tracked under `target/pgs/selected.config.ron` for rebuilds.
{% endif %}

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
