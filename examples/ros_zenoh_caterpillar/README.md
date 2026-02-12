# ros-zenoh-caterpillar

## Overview

`ros-zenoh-caterpillar` is a runnable Copper example.

## Prerequisites

- Rust stable toolchain
- Any hardware or services referenced by this example

## Run

```bash
cargo run -p <package-name>
```

## Expected Output

### Output

- A stats table prints once per second with min/max/mean/stddev/jitter per task.
- `End2End` reports src → gpio-7 latency using a monotonic timestamp.

## Links

- Example path: `examples/ros_zenoh_caterpillar`
- Build/deploy guide: <https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application>

## Additional Notes

### ROS2 + Zenoh Caterpillar example

ROS2 caterpillar demo using `rmw_zenoh_cpp`.

### Justfile command

- `just ros-zenoh-caterpillar-run` — clean build artifacts, build with `colcon`, source `install/setup.bash`, set `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, start `rmw_zenohd`, and launch the caterpillar node chain with ROS_DOMAIN_ID=42.
