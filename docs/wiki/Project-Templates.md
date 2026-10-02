# Project Templates

Copper ships templates to bootstrap new projects quickly. Use them to get a runnable task graph, logging setup, and basic app layout in minutes.

## Prerequisites

- Rust 1.95 or newer (rustup with the latest stable toolchain is recommended)
- `cargo-cunew`: `cargo install cargo-cunew`
- Optional for template development: `cargo-generate`: `cargo install cargo-generate`
- Optional: `just` for helper commands (see [Task Automation with just](Task-Automation-with-Just))

## Choose a template

| Template | Use it for |
| --- | --- |
| `project` / `cu_project` | Single-crate project for quick experiments. |
| `workspace` / `cu_full` | Multi-crate workspace with `apps/` and `components/`. |

## Preferred path: cargo-cunew

Generate a single-crate project:

```bash
cargo cunew my_project
cd my_project
cargo run
```

Generate a workspace:

```bash
cargo cunew --template workspace my_workspace
cd my_workspace
cargo run -p cu_example_app
```

Choose a dependency source:

- default `--source crates.io`: resolves the latest stable Copper crate versions from crates.io
- `--source git`: uses the Copper git repo, plus optional `--git-branch`, `--git-tag`, or `--git-rev`
- `--source local --copper-root /path/to/copper-rs`: points generated manifests at a local checkout

## Direct template path for development

The template source of truth now lives under:

```text
support/cargo_cunew/templates/
```

You can still use it directly with `cargo-generate` from a repo checkout:

```bash
cargo +stable generate \
    --path support/cargo_cunew/templates/cu_project \
    --name my_project \
    --destination . \
    --define copper_source=local \
    --define copper_root_path=/absolute/path/to/copper-rs
```

Or for the workspace template:

```bash
cargo +stable generate \
    --path support/cargo_cunew/templates/cu_full \
    --name my_workspace \
    --destination . \
    --define copper_source=local \
    --define copper_root_path=/absolute/path/to/copper-rs
```

## Shortcuts

- `cd support/cargo_cunew/templates && just gen-project`
- `cd support/cargo_cunew/templates && just gen-workspace`

Generated projects include `just dag` for graph topology and `just plan` for the exact generated process order. `just plan-log` uses the default Copper log, while `just plan log=logs/<app>.copper` selects one explicitly; both add packed, proportional observed task timing through the generated typed logreader. Local and published/git dependency modes resolve the matching first-party renderer automatically.
