#!/usr/bin/env bash
set -euo pipefail

ROS_DIR="/opt/ros/jazzy/share"

echo "| Type Name | RIHS01 Hash |"
echo "|-----------|-------------|"

find "$ROS_DIR" -name '*.json' -print0 |
  xargs -0 jq -r '
    .type_hashes[]? |
    "\(.type_name) \(.hash_string)"
  ' |
  sort -u |
  awk '{ printf "| %s | %s |\n", $1, $2 }'
