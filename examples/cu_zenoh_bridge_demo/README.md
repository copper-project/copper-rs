# cu-zenoh-bridge-demo

This demo starts two Copper apps that exchange ping/pong messages over Zenoh on three channels,
using different wire formats per channel (bincode, json, cbor).

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
