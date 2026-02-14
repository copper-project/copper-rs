# Copper ROS2 payloads

Those are the payloads for Copper (see crate cu29).

Types that are supported are:

- `std_msgs/Bool` <-> `bool`
- `std_msgs/Int8` <-> `i8`
- `std_msgs/Int16` <-> `i16`
- `std_msgs/Int32` <-> `i32`
- `std_msgs/Int64` <-> `i64`
- `std_msgs/UInt8` <-> `u8`
- `std_msgs/UInt16` <-> `u16`
- `std_msgs/UInt32` <-> `u32`
- `std_msgs/UInt64` <-> `u64`
- `std_msgs/Float32` <-> `f32`
- `std_msgs/Float64` <-> `f64`
- `std_msgs/String` <-> `String`
- `sensor_msgs/Image` <-> `CuImage<Vec<u8>>`
- `sensor_msgs/PointCloud2` <-> `PointCloudSoa<N>`
- `sensor_msgs/Imu` <-> `ImuPayload` (accel/gyro mapped; temperature defaults to `0°C` when decoding from ROS)
- `sensor_msgs/MagneticField` <-> `MagnetometerPayload`

## Adding new payloads (feel free to contribute!)

To add a new payload, we pregenerated all the RIHS01 in the [RIHS payloads](all_rihs.md) file from Jazzy.

Added the script [extract_rihs01.sh](extract_rihs01.sh) to extract the RIHS01 from Jazzy.
As an alternation you can do:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
ros2 topic echo /FOO std_msgs/String
ros2 topic info /FOO -v
```

## Justfile command

- `just ros-rihs01-hashes` — list ROS2 RIHS01 type hashes from `/opt/ros/jazzy/share`.
