#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"

router_pid=""
launch_pid=""

cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" >/dev/null 2>&1; then
    kill "$launch_pid" >/dev/null 2>&1 || true
  fi
  if [[ -n "$router_pid" ]] && kill -0 "$router_pid" >/dev/null 2>&1; then
    kill "$router_pid" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT INT TERM

echo "[container] starting rmw_zenohd on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
ros2 run rmw_zenoh_cpp rmw_zenohd &
router_pid="$!"

sleep 2

echo "[container] launching headless Nav2 TB3 simulation"
ros2 launch nav2_bringup tb3_simulation_launch.py \
  headless:=True \
  use_rviz:=False \
  use_composition:=False \
  autostart:=True &
launch_pid="$!"

sleep 5

echo "[container] waiting for Nav2 activation, then sending an automatic goal"
python3 /opt/copper_nav2/auto_nav_demo.py

wait "$launch_pid"
