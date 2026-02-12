# cu-iceoryx2-bridge-demo

## Overview

Ping/pong demo over the Iceoryx2 bridge.

Run in two terminals:

```sh
# Terminal 1
cargo run -p cu-iceoryx2-bridge-demo --bin iceoryx2-pong

# Terminal 2
cargo run -p cu-iceoryx2-bridge-demo --bin iceoryx2-ping
```

## Prerequisites

- Rust stable toolchain
- Any hardware or services referenced by this example

## Run

```bash
cargo run -p cu-iceoryx2-bridge-demo
```

## Expected Output

Expect successful startup logs and normal task-loop execution without panics.

## Links

- Example path: `examples/cu_iceoryx2_bridge_demo`
- Build/deploy guide: <https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application>
