# cu-zenoh-bridge-demo

## Overview

This demo starts two Copper apps that exchange ping/pong messages over Zenoh on three channels,
using different wire formats per channel (bincode, json, cbor).

## Prerequisites

- Rust stable toolchain
- Any hardware or services referenced by this example

## Run

Terminal 1:
```bash
cargo run -p cu-zenoh-bridge-demo --bin zenoh-pong
```

Terminal 2:
```bash
cargo run -p cu-zenoh-bridge-demo --bin zenoh-ping
```

`zenohd` is optional for local peer-to-peer runs, but you can start it for routed setups:
```bash
zenohd
```

You should see pong logs tagged with `pong-bincode`, `pong-json`, and `pong-cbor` in the ping app.

## Expected Output

Expect successful startup logs and normal task-loop execution without panics.

## Links

- Example path: `examples/cu_zenoh_bridge_demo`
- Build/deploy guide: <https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application>
