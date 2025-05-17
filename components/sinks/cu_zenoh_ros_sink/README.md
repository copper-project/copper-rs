# cu-zenoh-ros-sink

This is a Copper sink task that publishes messages to ROS 2 via Zenoh middleware.
It allows seamless integration between Copper applications and ROS 2 nodes by using the [RMW zenoh implementation](https://github.com/ros2/rmw_zenoh).

## Configuration

The sink requires the following configuration parameters:

- zenoh_config_json: Zenoh config [as json string](https://github.com/ros2/rmw_zenoh/blob/rolling/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5) (optional).
- domain_id: ROS domain ID (optional, defaults to "0"),
- namespace: The ROS namespace (optional, defaults to "copper").
- node: The ROS node name (optional, defaults to "node").
- topic: The ROS topic name to publish to (optional, defaults to "copper").

Example configuration in a Copper application:

```ron
(
    tasks: [
        (
            id: "publisher",
            config: {
                "domain_id": 0,
                "namespace": "copper",
                "node": "sink_node",
                "topic": "output",
            }
        ),
    ],
    // ... rest of the configuration
)
```

## Integration with ROS 2

To receive messages in ROS 2:

1. Install "rmw_zenoh":

```bash
sudo apt install ros-$ROS_DISTRO-rmw-zenoh
```

2. Set the RMW implementation to Zenoh:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh
```

3. Run your ROS 2 nodes as usual - they will automatically receive messages published by this sink.

## Limitations

- Payload has to implement `RosMsgAdapter` trait (see `cu_ros_payloads`).
- QoS is not implemented (default QoS).
- Only works with Humble distribution (hash type not implemented).
- No service implemented.

## Caveats

- Add an extra serialization step to CDR format that could impact performance.


See the crate [cu29](https://crates.io/crates/cu29) for more information about the Copper project.
