# Copper ROS2 bridge loopback example

This example uses `cu_ros2_bridge` as a proper Copper bridge with channel slots.
It wires one Tx slot to one Rx slot on the same topic and verifies that:
- Rx path works.
- Message conversion works (`i8` <-> `std_msgs/Int8`).

## Run

```bash
cargo run -p cu-ros2-bridge-demo
```

Expected output includes:

```text
ros2 bridge loopback OK: received ... messages
```
