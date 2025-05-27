# Copper ROS2 payloads

Those are the payloads for Copper (see crate cu29).

Types that are supported are:

- `std_msgs/Int8`
- `sensor_msgs/PointCloud2`

## Adding new payloads (feel free to contribute!)

To add a new payload, we pregenerated all the RIHS01 in the [RIHS payloads](all_rihs.md) file from Jazzy.

Added the script [extract_rihs01.sh](extract_rihs01.sh) to extract the RIHS01 from Jazzy.
As an alternation you can do:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
ros2 topic echo /FOO std_msgs/String
ros2 topic info /FOO -v
```

