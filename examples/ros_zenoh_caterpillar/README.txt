This a Zenoh based ROS2 caterpillar benchmark.

Note: ROS2 basically only compiles on specific OS so you need to use a VM or Docker container.

docker build -t ros-jazzy-zenoh .

# be sure you are in this directory

cd ~/[path_to_copper]/examples/ros_caterpillar

docker run --rm -it -v "$PWD":/ros2_ws -w /ros2_ws ros-jazzy-zenoh ./run.sh

# the run script will clean, rebuild and start all the nodes.
