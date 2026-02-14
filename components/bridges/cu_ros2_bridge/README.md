# cu-ros2-bridge

Bidirectional Copper bridge for ROS 2 messaging over Zenoh transport.

## Config

Bridge-level config:
- `domain_id`: ROS 2 domain id (default `0`).
- `namespace`: ROS namespace (default `copper`).
- `node`: ROS node name (default `node`).
- `zenoh_config_file`: path to a Zenoh JSON5 config file (optional).
- `zenoh_config_json`: inline Zenoh JSON5 config (optional).

Per-channel config (`bridges[].channels`):
- `route`: ROS topic path used by that channel.

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

## Supported payload mappings

Current built-in conversion mapping:
- `i8` <-> `std_msgs/Int8`

Additional payload mappings can be added in the bridge implementation.
