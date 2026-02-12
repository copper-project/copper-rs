# Examples

This directory contains runnable Copper applications for host, simulation, bridge integration, and embedded workflows.

## Overview

Use these crates as references for task graphs, resource configuration, and deployment patterns.

## Recommended Starting Points

| Goal | Example |
|---|---|
| Small runtime + simulation behavior | `examples/cu_run_in_sim` |
| End-to-end host app graph | `examples/cu_caterpillar` |
| Configuration modularization | `examples/modular_config_example` |
| Logging and visualization | `examples/cu_logviz_demo` |
| Embedded skeleton (RP2350) | `examples/cu_rp2350_skeleton` |
| Embedded flight controller stack | `examples/cu_flight_controller` |
| Zenoh bridge integration | `examples/cu_zenoh_bridge_demo` |

## Quick Run

From repository root:

```bash
cargo run -p cu-run-in-sim
```

Or run any example package directly:

```bash
cargo run -p <example-package>
```

## Links

- Style guide: [`doc/readme-style-guide.md`](../doc/readme-style-guide.md)
- Build/deploy guide: <https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application>
- Supported platforms: <https://copper-project.github.io/copper-rs/Supported-Platforms>
