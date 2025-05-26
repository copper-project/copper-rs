# Copper ROS bridge using Zenoh

ONLY SUPPORTS HUMBLE at the moment.

## Install the zenoh daemon

```bash
cargo install zenohd --all-features
```

## To run the bridge

Launch a zenoh router.

```bash
$ RUST_LOG=debug zenohd
```

The log as debug will help you understand the INSANE mapping between ROS and zenoh.

Then run this example.

```bash
$ cd cuexamples/cu_zenoh_ros
$ cargo run --release 
```

## Run ROS from Docker (optional)

If you don't have access to a perfect Ubuntu version to run ROS, you can do it from the Docker container given

docker build -t ros-jazzy-zenoh .
docker run --rm -it --net=host -v "$PWD":/ros2_ws -w /ros2_ws ros-jazzy-zenoh bash

## Display the result.

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic echo /output
```

## Troubleshooting Zenoh

To inspect zenoh, this is suddedly some python stuff, so let's go and make a virtualenv.

```bash 
python3 -m venv zcli
source zcli/bin/activate
pip install zenoh-cli
```

Dumping ALL the traffic:

```bash
zenoh subscribe -k '0/output/**'
```
0 is the ROS2 domain
output is the name of the topic

It is followed with some mangled things like the RIHS01 an and QOS.

