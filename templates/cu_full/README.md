# {{project-name}} Workspace

This template bootstraps a Copper workspace with an app crate and a shared-components layout.

## Layout

- `apps/README.md`: overview for app crates.
- `apps/cu_example_app/`: primary app, local tasks/messages, config, and logs.
- `components/bridges/cu_example_shared_bridge/`: shared bridge example crate.
- `components/`: shared components by category (placeholders to extend).
- `doc/`: design notes and project docs.
- `justfile`: automation helpers like `just rcfg`, `just log`, and `just cl`.

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
