## ROS2 + Zenoh Caterpillar example

ROS2 caterpillar demo using `rmw_zenoh_cpp`.

### Justfile command

- `just ros-zenoh-caterpillar-run` — clean build artifacts, build with `colcon`, source `install/setup.bash`, set `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, start `rmw_zenohd`, and launch the caterpillar node chain with ROS_DOMAIN_ID=42.

### Output

- A stats table prints once per second with min/max/mean/stddev/jitter per task.
- `End2End` reports src → gpio-7 latency using a monotonic timestamp.
