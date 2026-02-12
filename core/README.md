# Core Crates

The `core/` workspace provides Copper runtime primitives, macros, logging, and value/serialization building blocks.

## Overview

These crates form the deterministic execution model used by application and component crates.

## Crate Map

| Crate | Purpose |
|---|---|
| `core/cu29` | Main Copper crate and public entrypoint. |
| `core/cu29_runtime` | Runtime graph execution, config loading, and lifecycle wiring. |
| `core/cu29_traits` | Core task/bridge/runtime traits. |
| `core/cu29_derive` | Runtime derive/proc-macro support. |
| `core/cu29_clock` | Monotonic timing and simulation/mock clock support. |
| `core/cu29_log` | Logging macros and compile-time log-level support. |
| `core/cu29_log_derive` | Structured logging derive macros. |
| `core/cu29_log_runtime` | Runtime log transport structures. |
| `core/cu29_unifiedlog` | Unified binary log support and utilities. |
| `core/cu29_helpers` | Runtime/bootstrap helpers. |
| `core/cu29_value` | Value transport and serialization helpers. |
| `core/cu29_export` | Log export helpers. |
| `core/cu29_soa_derive` | Structure-of-arrays derive support. |

## Usage

Most users depend on `cu29` and optionally add lower-level crates for specific runtime or logging integration needs.

## Links

- Style guide: [`doc/readme-style-guide.md`](../doc/readme-style-guide.md)
- Runtime overview: <https://copper-project.github.io/copper-rs/Copper-Runtime-Overview>
- Workspace root: [`README.md`](../README.md)
