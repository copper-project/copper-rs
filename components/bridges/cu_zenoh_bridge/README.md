# cu-zenoh-bridge

Bidirectional Copper bridge over Zenoh, with multiple typed channels per bridge.

## Config

Bridge-level config:
- `zenoh_config_file`: path to a Zenoh config file (optional). This should be a JSON5 file following
  Zenoh's session configuration schema. You can start from the default config:
  https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5
  See the configuration docs: https://zenoh.io/docs/manual/configuration/
- `zenoh_config_json`: Zenoh config as an inline JSON5 string (optional). This is useful when you
  want to embed a small config directly in your Copper config instead of providing a file.
  The schema is the same as the JSON5 file above.
- `wire_format`: default wire format (`bincode`, `json`, or `cbor`).

Per-channel config (inside `channels`):
- `route`: Zenoh key expression for the channel.
- `config.wire_format`: override the default wire format per channel.

Tx empty-message behavior is static and comes from the Rust channel declaration, not from
`copperconfig.ron`:
- Tx channels skip `send` by default when their `CuMsg` payload is empty.
- Prefix a `tx_channels!` declaration with `[publish_empty]` to keep metadata-only publishes enabled.

Example declaration:
```rust
tx_channels! {
    ping_bin => Ping,
    [publish_empty] heartbeat => Tick = "demo/heartbeat",
}
```

With `cu-zenoh-bridge`, `[publish_empty]` means the bridge will still publish a `CuMsg` carrying
`payload = None`, `tov`, Copper metadata, and the usual Zenoh attachment provenance.

Example:
```ron
bridges: [
  (
    id: "zenoh",
    type: "bridges::DemoZenohBridge",
    config: {
      "wire_format": "bincode",
      "zenoh_config_file": "/path/to/zenoh.json5",
    },
    channels: [
      Tx(id: "ping_bin", route: "demo/ping/bin"), // default to bincode as per bridge config
      Tx(id: "ping_json", route: "demo/ping/json", config: { "wire_format": "json" }),
      Rx(id: "pong_bin", route: "demo/pong/bin"),
      Rx(id: "pong_json", route: "demo/pong/json", config: { "wire_format": "json" }),
    ],
  ),
],
```

Inline JSON5 example:
```ron
config: {
  "zenoh_config_json": r#"
    {
      scouting: { timeout_ms: 1000 },
      transport: { unicast: { max_links: 2 } }
    }
  "#,
}
```

The bridge uses a single Zenoh session and declares one publisher/subscriber per configured channel.

## no_std / RP2350 serial

Enable `cu-zenoh-bridge` with `default-features = false, features = ["nostd"]` to keep the
same `ZenohBridge<Tx, Rx>` application type on an MCU.  The embedded backend uses the in-tree
`support/zenoh-nostd` fork and supports bincode messages plus Copper provenance
attachments only.

The application must provide an `Arc<spin::Mutex<dyn ZenohNostdRuntime>>` resource.  That
resource owns the Embassy Zenoh session and UART task, and its `try_publish` / `try_receive`
methods must be nonblocking.  A full outgoing queue drops the newest frame; inspect
`ZenohBridge::dropped_newest()` for the count.  This keeps serial latency and allocation out of
Copper's execution cycle.

Embedded routes are intentionally static: declare them in `tx_channels!` / `rx_channels!` and do
not override them in RON.  JSON/CBOR and Zenoh JSON5 configuration are host-only.  The first
transport is a client-only `serial/...` link; configure the host router with its matching serial
listener.  `zenoh-nostd` needs a board-provided `getrandom` custom backend for its session ID;
the RP2350 firmware must register its hardware entropy source before it starts the session task.
