# cu-zenoh-bridge-demo

This demo starts two Copper apps that exchange ping/pong messages over Zenoh on three channels,
using different wire formats per channel (bincode, json, cbor).

## Run

Terminal 1:
```bash
cargo run -p cu-zenoh-bridge-demo --bin zenoh-pong -- --iterations 40 --instance-id 2
```

Terminal 2:
```bash
cargo run -p cu-zenoh-bridge-demo --bin zenoh-ping -- --iterations 20 --instance-id 1
```

`zenohd` is optional for local peer-to-peer runs, but you can start it for routed setups:
```bash
zenohd
```

You should see pong logs tagged with `pong-bincode`, `pong-json`, and `pong-cbor` in the ping app.

Logs are written under `examples/cu_zenoh_bridge_demo/logs/` by default. You can override the
path with `--log <path>`.

## Validate The Strict Multi-Copper Config Layer

This example also includes `multi_copper.ron`, a strict umbrella config that models the ping and
pong apps as two explicit Copper subsystems connected through bridge channels.

```bash
just dag
cargo run -p cu-zenoh-bridge-demo --bin validate-multi-config
```

## Inspect Recorded Bridge Provenance

After running both apps, inspect the recorded bridge RX provenance in the Copper logs:

```bash
cargo run -p cu-zenoh-bridge-demo --bin inspect-ping-provenance
cargo run -p cu-zenoh-bridge-demo --bin inspect-pong-provenance
```

The inspectors print bridge RX slots with the remote `{subsystem_code, instance_id, cl_id}` that
arrived over Zenoh and was persisted in the local CopperList log.
