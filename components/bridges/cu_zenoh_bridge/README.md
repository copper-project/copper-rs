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
