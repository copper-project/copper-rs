#!/usr/bin/env bash
set -e 

## !!! this needs to be run in the target environment of ROS2, e.g. in a docker container with ROS2 installed or target machine.

# Clean up previous builds and installations
rm -rf build/ install/ log/

# build
colcon build
. install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

ros2 run rmw_zenoh_cpp rmw_zenohd &
read -n1 -r -p "Press any key to continue..."

echo "Starting all nodes..."
ros2 run gpio_caterpillar propagate_node gpio_node_1 flip_topic_1 flip_topic_2 &
ros2 run gpio_caterpillar actuation_node gpio_node_1 4 &
ros2 run gpio_caterpillar propagate_node gpio_node_2 flip_topic_2 flip_topic_3 &
ros2 run gpio_caterpillar actuation_node gpio_node_2 17 &
ros2 run gpio_caterpillar propagate_node gpio_node_3 flip_topic_3 flip_topic_4 &
ros2 run gpio_caterpillar actuation_node gpio_node_3 27 & 
ros2 run gpio_caterpillar propagate_node gpio_node_4 flip_topic_4 flip_topic_5 &
ros2 run gpio_caterpillar actuation_node gpio_node_4 22 &
ros2 run gpio_caterpillar propagate_node gpio_node_5 flip_topic_5 flip_topic_6 &
ros2 run gpio_caterpillar actuation_node gpio_node_5 5 &
ros2 run gpio_caterpillar propagate_node gpio_node_6 flip_topic_6 flip_topic_7 &
ros2 run gpio_caterpillar actuation_node gpio_node_6 6 &
ros2 run gpio_caterpillar propagate_node gpio_node_7 flip_topic_7 flip_topic_8 &
ros2 run gpio_caterpillar actuation_node gpio_node_7 19 &
ros2 run gpio_caterpillar propagate_node gpio_node_8 flip_topic_8 flip_topic_9 &
ros2 run gpio_caterpillar actuation_node gpio_node_8 26 &
sleep 2  # Wait for all nodes to start
ros2 run gpio_caterpillar flip_node &
echo "All nodes started"
wait
