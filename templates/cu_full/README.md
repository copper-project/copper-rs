# {{project-name}} Workspace

This template bootstraps a multi-crate Copper workspace intended to scale beyond a single binary.

## Layout

- `apps/{{project-name}}/`: runtime binary, log reader, and config.
- `tasks/{{project-name}}-tasks/`: Copper task implementations.
- `libs/{{project-name}}-types/`: shared message types.
- `docs/`: design notes and project docs.
- `scripts/`: automation helpers like `log`, `cl`, and `rcfg`.

## Quick start

```bash
cargo run -p {{project-name|kebab_case}}
```

The runtime config lives in `apps/{{project-name}}/config/copperconfig.ron`.
