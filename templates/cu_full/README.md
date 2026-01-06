# {{project-name}} Workspace

This template bootstraps a multi-crate Copper workspace intended to scale beyond a single binary.

## Layout

- `apps/{{project-name}}/`: primary app, log reader, and config.
- `apps/{{project-name}}-demo/`: secondary app that reuses shared components.
- `components/payloads/{{project-name}}-types/`: shared message types.
- `components/sources/{{project-name}}-source/`: source task.
- `components/tasks/{{project-name}}-processor/`: processing task.
- `components/sinks/{{project-name}}-sink/`: sink task.
- `docs/`: design notes and project docs.
- `justfile`: automation helpers like `just log`, `just cl`, and `just rcfg` (set `APP_NAME={{project-name|kebab_case}}-demo APP_DIR={{project-name}}-demo` for the demo app).

## Quick start

```bash
cargo run -p {{project-name|kebab_case}}
```

Or run the demo app:

```bash
cargo run -p {{project-name|kebab_case}}-demo
```

The runtime configs live in:

- `apps/{{project-name}}/config/copperconfig.ron`
- `apps/{{project-name}}-demo/config/copperconfig.ron`
