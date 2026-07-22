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

Rx-only options:
- `queue_mode`: `"fifo"` (default) First In First Out mode. Delivers samples in order, one per cycle. `"ring"` creates a buffer and drops oldest sample when full.
- `ring_size`: depth of the ring buffer when `queue_mode` is `"ring"` (default `1`).

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
      // fifo: ordered delivery, one sample per cycle (default)
      Rx(id: "incoming", route: "/input"),
      // ring: always receive the freshest sample
      Rx(id: "sensor",   route: "/sensor", config: {"queue_mode": "ring"}),
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

## Wire encoding

Copper -> ROS 2 samples are serialized as OMG CDR little-endian (`CdrLe`). Inbound samples use
`cdr::deserialize`, which reads the CDR encapsulation header and accepts either endianness.

### ROS 2 Humble image compatibility

Enable the `humble` feature when bridging `cu_sensor_payloads::CuImage<Vec<u8>>` to ROS 2 Humble.
The image adapter uses Humble's legacy `yuv422_yuy2` and `yuv422` encoding names for YUYV and
UYVY buffers. Humble's `cv_bridge` cannot consume NV12, NV21, I420, or YV12 images, so the bridge
rejects those formats instead of publishing an image that tools such as `rqt_image_view` cannot
decode. Convert those images to a supported packed format before sending them through the bridge.

## ROS 2 interop test

The `ros2_interop` test auto-detects a sourced ROS 2 environment. If `ros2`, `std_msgs`, or
`rmw_zenoh_cpp` are unavailable, the test prints a warning with the skip reason and returns
success. The Linux CI image includes ROS 2 Lyrical and `rmw_zenoh_cpp`, so the same normal test
flow exercises real ROS 2 bridge interoperability in CI.

When available, the test starts `rmw_zenohd`, runs a real ROS 2 subscriber with
`ros2 topic echo --once`, publishes an `std_msgs/msg/Int32` through `Ros2Bridge`, and requires
the ROS 2 process to print `data: 42`.

```bash
cargo test -p cu-ros2-bridge --test ros2_interop -- --nocapture
```
