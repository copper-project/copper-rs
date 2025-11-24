# cu-msp-bridge-loopback

This example exercises the `cu_msp_bridge` component end-to-end in a controlled loopback setup. It spins up a pseudo-terminal–based MSP “flight controller” emulator, rewires the bridge to talk to it, and validates that a single `MSP_RC` request receives the expected response.

## Running

```bash
cargo run -p cu-msp-bridge-loopback
```

## What it does

1. Preallocates Copper unified log slabs under `logs/`.
2. Creates a PTY pair and exposes the emulator slave via a symlink (`logs/msp_bridge_loopback_slave`).
3. Overrides the bridge device to point at that symlink.
4. Sends exactly one `MSP_RC` request from the `LoopbackSource` task.
5. The emulator echoes back an `MSP_RC` response.
6. `LoopbackSink` inspects the response and marks the run as validated.
