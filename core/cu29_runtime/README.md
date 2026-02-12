# cu29-runtime

## Overview

`cu29-runtime` contains Copper’s execution engine, configuration loading, task graph orchestration, and runtime lifecycle support.
It is the core implementation used by `cu29`.

## Usage

Most applications should depend on `cu29` directly. Depend on `cu29-runtime` when you need lower-level runtime APIs or runtime tooling binaries such as `cu29-rendercfg`.

Example:

```bash
cargo run -p cu29-runtime --bin cu29-rendercfg -- --help
```

## Compatibility

- Default feature set targets `std`.
- Supports `no_std` builds with reduced runtime/tooling capabilities.
- Optional features include `cuda`, `reflect`, and `remote-debug`.

## Links

- docs.rs (cu29): <https://docs.rs/cu29>
- Config reference: <https://copper-project.github.io/copper-rs/Copper-RON-Configuration-Reference>
- DAG rendering: <https://copper-project.github.io/copper-rs/Config-and-Missions-Visualization>
- Crate path: `core/cu29_runtime`
- docs.rs: <https://docs.rs/cu29-runtime>
