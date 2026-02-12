# Copper ROS2 payloads

## Overview

Those are the payloads for Copper (see crate cu29).

Types that are supported are:

- `std_msgs/Int8`
- `sensor_msgs/PointCloud2`

## Interface

### Adding new payloads (feel free to contribute!)

To add a new payload, we pregenerated all the RIHS01 in the [RIHS payloads](all_rihs.md) file from Jazzy.

Added the script [extract_rihs01.sh](extract_rihs01.sh) to extract the RIHS01 from Jazzy.
As an alternation you can do:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
ros2 topic echo /FOO std_msgs/String
ros2 topic info /FOO -v
```

## Configuration

No direct runtime config keys; select payload types in task/channel type definitions.

## Usage

### Justfile command

- `just ros-rihs01-hashes` — list ROS2 RIHS01 type hashes from `/opt/ros/jazzy/share`.

## Compatibility

Designed for cross-crate reuse; feature compatibility is defined in `Cargo.toml`.

## Links

- Crate path: `components/payloads/cu_ros_payloads`
- docs.rs: <https://docs.rs/cu-ros-payloads>
