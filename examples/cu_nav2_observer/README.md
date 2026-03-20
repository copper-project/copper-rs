# Copper Nav2 observer example

This example uses `cu_ros2_bridge` as a read-only bridge into a ROS2/Nav2 graph and visualizes:

- `nav_msgs/Path` on `/plan`
- `nav_msgs/Path` on `/received_global_plan`
- `nav_msgs/Path` on `/plan_smoothed`
- `nav_msgs/Path` on `/local_plan`
- `nav_msgs/Odometry` on `/odom`

The default configuration is intentionally simple:

- Copper subscribes to ROS2 topics through the existing Zenoh-based ROS2 bridge.
- The observer uses an explicit Zenoh session config that connects to the local `rmw_zenohd` router on `tcp/localhost:7447`.
- A Copper sink task renders the current plan, robot pose, and odometry trail in Rerun.
- No Nav2 action client is involved. This is an observer, not a `NavigateToPose` driver.

## What this proves

It shows that a Copper app can sit next to a Nav2 stack, receive ROS2 messages through the bridge,
log them, and visualize them without writing a bespoke ROS node.

## Requirements

- ROS 2 Jazzy or newer on the host
- `rmw_zenoh_cpp`
- Nav2 binaries:

```bash
sudo apt install \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-nav2-minimal-tb* \
  ros-$ROS_DISTRO-rmw-zenoh-cpp
```

This follows the official Nav2 Jazzy getting-started path:
- https://docs.nav2.org/getting_started/
- https://docs.nav2.org/development_guides/build_docs/index.html

If your Nav2 stack uses different topic names, edit [`copperconfig.ron`](/home/gbin/projects/copper/copper-rs.nav2/examples/cu_nav2_observer/copperconfig.ron).

## Pull The Upstream Demo Repo

If you want the official upstream example sources locally for reference or further customization:

```bash
cd examples/cu_nav2_observer
just nav2-demo-pull
```

That clones or updates:
- https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation

## Run

### Recommended On Arch: Docker

Open two terminals.

Terminal 1, run the ROS/Nav2 side in Docker:

```bash
cd examples/cu_nav2_observer
just nav2-demo-docker-run
```

This builds a Docker image from:
- `osrf/ros:jazzy-desktop-full`

and runs a headless Nav2 demo that:
- starts `rmw_zenohd`
- launches `tb3_simulation_launch.py` with `headless:=True` and `use_rviz:=False`
- sets the initial pose automatically
- sends a demo goal automatically

Terminal 2, run the Copper observer on the host:

```bash
cd examples/cu_nav2_observer
just observer-run
```

After that, Copper should start receiving `/odom` and whichever Nav2 path topics are active in that bringup. Rerun should show the robot trail plus any of:

- `/plan`
- `/received_global_plan`
- `/plan_smoothed`
- `/local_plan`

The Copper app writes logs to `logs/nav2_observer.copper` and spawns a Rerun recording stream.

### Native Host Run

If you already have Nav2 plus `rmw_zenoh_cpp` installed on the host, you can still use:

```bash
cd examples/cu_nav2_observer
just nav2-demo-run
```

That path launches the official:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

and expects you to set the initial pose and goal manually in RViz.

## Notes

- The bridge currently supports pub/sub only, not ROS2 actions or services.
- Because of that, this example subscribes to Nav2 outputs instead of commanding goals.
- The example keeps its `nav_msgs/Path` and `nav_msgs/Odometry` adapters local so it stays self-contained.
- The upstream minimal simulation repo is not vendored into this workspace; `just nav2-demo-pull` fetches it into `.deps/`.
- The Docker path is intended for hosts like Arch that have ROS 2 but not packaged Jazzy Nav2 and `rmw_zenoh_cpp`.
