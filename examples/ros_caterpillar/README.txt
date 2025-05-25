This a ROS2 comparison for the caterpillar example.

Note: ROS2 basically only compiles on specific OS so you need to use a VM or Docker container.

# install docker on your system
docker pull osrf/ros:jazzy-desktop

# be sure you are in this directory

cd ~/[path_to_copper]/examples/ros_caterpillar

docker run --rm -it -v "$PWD":/ros2_ws -w /ros2_ws osrf/ros:jazzy-desktop ./run.sh

# the run script will clean, rebuild and start all the nodes.






