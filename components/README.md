# Components

Copper components are reusable building blocks for application graphs.

## Overview

The `components/` tree is organized by role. Most crates are plug-and-play from `copperconfig.ron` once imported in your app type registry.

## Categories

| Directory | Purpose |
|---|---|
| `components/bridges` | Bidirectional adapters to external systems (transport/protocol bridges). |
| `components/libs` | Shared support libraries used by runtime components. |
| `components/monitors` | Runtime monitors and observability helpers. |
| `components/payloads` | Common message payload definitions for task I/O. |
| `components/res` | Platform resource bundles (HAL-like providers). |
| `components/sinks` | Outgoing endpoints, usually actuation side. |
| `components/sources` | Incoming endpoints, usually sensor side. |
| `components/tasks` | In-graph computation tasks and algorithms. |
| `components/testing` | Test support components and integration helpers. |

## Interface Notes

- Sources produce messages into Copper.
- Tasks transform messages inside Copper.
- Sinks consume messages from Copper.
- Bridges expose shared transports with typed Rx/Tx channels.

## Links

- Style guide: [`doc/readme-style-guide.md`](../doc/readme-style-guide.md)
- Component inventory (wiki): <https://copper-project.github.io/copper-rs/Available-Components>
- Workspace root: [`README.md`](../README.md)
