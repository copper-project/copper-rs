# {{project-name}} Workspace

This template bootstraps a multi-crate Copper workspace intended to scale beyond a single binary.

## Layout

- `apps/{{project-name|kebab_case}}/`: runtime binary, log reader, and config.
- `tasks/{{project-name|kebab_case}}-tasks/`: Copper task implementations.
- `libs/{{project-name|kebab_case}}-types/`: shared message types.
- `docs/`: design notes and project docs.
- `scripts/`: automation helpers like `log`, `cl`, and `rcfg`.

## Quick start

```bash
cargo run -p {{project-name|kebab_case}}
```

The runtime config lives in `apps/{{project-name|kebab_case}}/config/copperconfig.ron`.
