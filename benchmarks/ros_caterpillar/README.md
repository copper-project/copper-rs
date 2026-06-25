## ROS2 Caterpillar example

ROS2 version of the caterpillar demo built with `colcon`.

### Justfile command

- `just ros-caterpillar-run` — clean build artifacts, build with `colcon`, source `install/setup.bash`, and launch the caterpillar node chain with ROS_DOMAIN_ID=42.

### Output

- A stats table prints once per second with min/max/mean/stddev/jitter per task.
- `End2End` reports src → gpio-7 latency using a monotonic timestamp.
