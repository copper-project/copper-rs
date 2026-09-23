# {{project-name}} Workspace

This template bootstraps a Copper workspace with an app crate and a shared-components layout.

## Layout

- `apps/README.md`: overview for app crates.
- `apps/cu_example_app/`: primary app, local tasks/messages, config, and logs.
- `components/bridges/cu_example_shared_bridge/`: shared bridge example crate.
- `components/`: shared components by category (placeholders to extend).
- `doc/`: design notes and project docs.
- `justfile`: `graph-view[-log]`, `schedule-view[-log]`, `log`, `cl`, `resim`, and `resim-debug` recipes.
{% if pgs_enabled %}
- `apps/cu_example_app/schedule.ron`: starter `src → sink` scheduling contract with timing to tune.
- `apps/cu_example_app/src/pgs.rs`: offline PGS workflow with a compile-time selected candidate.
{% endif %}

## Quick start

```bash
cargo run -p cu_example_app
```

The runtime config lives in `apps/cu_example_app/copperconfig.ron`.

To decode structured logs, use the logreader helpers:

```bash
just log
just cl
```

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

Viewer and log recipes take the app name as their first positional argument. For example,
`just graph-view cu_example_app` renders the primary app.

{% if pgs_enabled %}
## Profile-guided scheduling

Run `just pgs-baseline`, `just pgs-optimize candidates=3`,
`just pgs-candidate candidate=1`, and `just pgs-measure candidates="1"`.
Inspect `apps/cu_example_app/target/pgs/report.txt` and the generated SVGs.
Tune the placeholder deadline and CPU list in `schedule.ron` before using its
ranking to make a deployment choice.
{% endif %}
