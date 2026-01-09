# {{project-name}} Project

This template bootstraps a single-crate Copper project for quick experiments.

## Layout

- `src/main.rs`: runtime binary.
- `src/logreader.rs`: log reader binary.
- `src/tasks.rs`: sample tasks.
- `copperconfig.ron`: runtime configuration.
- `justfile`: automation helpers like `just log`, `just cl`, and `just rcfg`.

## Quick start

```bash
cargo run
```

The runtime config lives in `copperconfig.ron`.
