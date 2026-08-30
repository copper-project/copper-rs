#!/usr/bin/env bash
set -euo pipefail

seconds=${1:-60}
seconds=${seconds#seconds=}
out=analysis/data/lame
mkdir -p "$out"

docker build -f docker/Dockerfile.lame -t cu-autoware-lame:humble .
docker run --rm \
  --cap-add SYS_NICE \
  --security-opt seccomp=unconfined \
  -v "$(pwd)/analysis/data/lame:/results" \
  cu-autoware-lame:humble "$seconds" /results "$(id -u)" "$(id -g)"
