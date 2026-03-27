#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
example_dir="$(cd "$script_dir/.." && pwd)"

ros_distro="${ROS_DISTRO:-jazzy}"
setup_bash="/opt/ros/$ros_distro/setup.bash"
headless="${NAV2_HEADLESS:-False}"

if [[ ! -f "$setup_bash" ]]; then
  echo "Missing ROS setup: $setup_bash"
  echo "Install ROS 2 $ros_distro first."
  exit 1
fi

# ROS/colcon setup scripts are not consistently nounset-safe.
set +u
source "$setup_bash"
set -u

missing=0
is_arch=0
if command -v pacman >/dev/null 2>&1; then
  is_arch=1
fi

install_hint() {
  local ubuntu_hint="$1"
  if [[ "$is_arch" -eq 1 ]]; then
    echo "Install hint: this host uses pacman/Arch; Nav2 and rmw_zenoh_cpp are not visible in the sourced ROS environment."
    echo "Install hint: use your Arch ROS packaging source, build these packages in a source overlay, or run the ROS side in a container."
  else
    echo "Install hint: $ubuntu_hint"
  fi
}

check_pkg() {
  local pkg="$1"
  local hint="$2"
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "Missing ROS package: $pkg"
    install_hint "$hint"
    missing=1
  fi
}

check_pkg "nav2_bringup" "sudo apt install ros-$ros_distro-nav2-bringup ros-$ros_distro-navigation2"
check_pkg "rmw_zenoh_cpp" "sudo apt install ros-$ros_distro-rmw-zenoh-cpp"

if ! ros2 pkg prefix nav2_minimal_tb3_sim >/dev/null 2>&1 \
  && ! ros2 pkg prefix nav2_minimal_tb4_sim >/dev/null 2>&1; then
  echo "Missing Nav2 minimal turtlebot simulation packages"
  install_hint "sudo apt install ros-$ros_distro-nav2-minimal-tb*"
  missing=1
fi

if [[ "$missing" -ne 0 ]]; then
  exit 1
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

router_pid=""
cleanup() {
  if [[ -n "$router_pid" ]] && kill -0 "$router_pid" >/dev/null 2>&1; then
    kill "$router_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting rmw_zenohd on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
ros2 run rmw_zenoh_cpp rmw_zenohd &
router_pid="$!"

sleep 2

cat <<EOF
Launching the official Nav2 TB3 simulation demo.

Next steps once RViz is up:
  1. Click "Startup" if Nav2 is not already active.
  2. Set the initial pose with "2D Pose Estimate".
  3. Send a goal with "Nav2 Goal".
  4. In another terminal, run:
       cd "$example_dir"
       just observer-run

If the path topic is not /plan in your setup, update copperconfig.ron.
EOF

ros2 launch nav2_bringup tb3_simulation_launch.py "headless:=$headless"
