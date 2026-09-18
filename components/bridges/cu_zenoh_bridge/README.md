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
- `config.queue_mode` (**Rx only**): `fifo` (default) or `ring`.
- `config.ring_size`: depth when `queue_mode` is `ring` (default `1`, i.e. latest-wins).

### Choosing a queue mode

The bridge consumes **at most one sample per `receive` call**, i.e. one per graph iteration. So a
channel whose publisher is faster than the consuming graph's rate falls behind, and under the
default `fifo` handler it falls behind *losslessly and without bound*: the consumer keeps reading
ever-older samples, in order, with nothing dropped and no error on either side.

Measured on a loopback link, 724 B at 200 Hz against a consumer draining 100/s: the consumer
received sequence numbers 0..2490 contiguously while the publisher had reached 5908 — 12 s behind
and growing linearly. The publisher was not slowed (197 Hz sustained), so this is staleness, not
back-pressure, and it is invisible to a consumer that only checks *whether* samples arrive.

- `fifo` — use when every sample matters and the consumer is guaranteed to keep up: commands,
  events, anything where dropping one is a lost instruction.
- `ring` with `ring_size: 1` — use for sensor streams, where only the newest value is wanted.
  Drops the oldest sample when full, so `receive` always yields the most recent one available.

`queue_mode` on a **Tx** channel is rejected at construction rather than ignored: it reads like it
bounds the publisher and would do nothing at all.

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
      // A sensor stream: keep only the newest sample rather than falling behind in order.
      Rx(id: "odom", route: "demo/odom", config: { "queue_mode": "ring", "ring_size": 1 }),
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
