# cu-ros2-bridge

Bidirectional Copper bridge for ROS 2 messaging over Zenoh transport.

## Config

Bridge-level config:
- `domain_id`: ROS 2 domain id (default `0`).
- `namespace`: ROS 2 namespace (default `copper`).
- `node`: ROS 2 node name (default `node`).
- `zenoh_config_file`: path to a Zenoh JSON5 config file (optional).
- `zenoh_config_json`: inline Zenoh JSON5 config (optional).

Per-channel config (`bridges[].channels`):
- `route`: ROS 2 topic path used by that channel.

Example:

```ron
bridges: [
  (
    id: "ros2",
    type: "bridges::DemoRos2Bridge",
    config: {
      "domain_id": 0,
      "namespace": "copper",
      "node": "bridge_node",
    },
    channels: [
      Tx(id: "outgoing", route: "/output"),
      Rx(id: "incoming", route: "/input"),
    ],
  ),
],
```

## Payload codec registry

The bridge uses a payload codec registry keyed by Copper payload type. It ships with:
- `bool`, `i8`, `i16`, `i32`, `i64`, `u8`, `u16`, `u32`, `u64`, `f32`, `f64`, `String`

To add your own payloads, implement `cu_ros2_payloads::RosBridgeAdapter` for the payload type and
register it once at startup:

```rust
cu_ros2_bridge::register_ros2_payload::<MyPayload>();
```
