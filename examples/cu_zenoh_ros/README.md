# ROS bridge using Zenoh

## To run the bridge

Launch a zenoh router.

```bash
$ zenohd
```

Then run this example.

```bash
$ cd cuexamples/cu_zenoh_ros
$ cargo run --release 
```

Display the result.

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic echo /output
```

