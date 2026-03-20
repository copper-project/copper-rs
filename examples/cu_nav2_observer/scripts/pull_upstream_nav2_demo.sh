#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
example_dir="$(cd "$script_dir/.." && pwd)"

repo_url="${NAV2_UPSTREAM_REPO:-https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation.git}"
repo_branch="${NAV2_UPSTREAM_BRANCH:-main}"
target_dir="${NAV2_UPSTREAM_DIR:-$example_dir/.deps/nav2_minimal_turtlebot_simulation}"

mkdir -p "$(dirname "$target_dir")"

if [[ -d "$target_dir/.git" ]]; then
  echo "Updating upstream Nav2 demo in $target_dir"
  git -C "$target_dir" fetch --tags origin "$repo_branch"
  git -C "$target_dir" checkout "$repo_branch"
  git -C "$target_dir" pull --ff-only origin "$repo_branch"
else
  echo "Cloning upstream Nav2 demo into $target_dir"
  git clone --branch "$repo_branch" "$repo_url" "$target_dir"
fi

echo
echo "Upstream Nav2 demo available at:"
echo "  $target_dir"
