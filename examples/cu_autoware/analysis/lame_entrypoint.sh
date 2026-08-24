#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: lame-entrypoint SECONDS OUTPUT_DIR HOST_UID HOST_GID" >&2
  exit 2
fi
seconds=$1
out=$2
host_uid=$3
host_gid=$4
mkdir -p "$out"
set +u
source /opt/ros/humble/setup.bash
source /opt/lame_ws/install/setup.bash
set -u

raw="$out/raw.log"
setsid /opt/lame_ws/install/lib/latency_mgmt/latency_mgmt 1 >"$raw" 2>&1 &
pid=$!

# LaME profiles for 15 seconds, pauses for 5, then computes its allocation. Wait for the
# second start marker so both the requested duration and process sampling cover only the
# managed phase. Fail after three minutes instead of hanging on an upstream failure.
for _ in $(seq 1 1800); do
  [ "$(grep -c 'Executor and all timers start.' "$raw" || true)" -ge 2 ] && break
  kill -0 "$pid" 2>/dev/null || break
  sleep 0.1
done
if [ "$(grep -c 'Executor and all timers start.' "$raw" || true)" -lt 2 ]; then
  kill -INT "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  echo "LaME did not enter its managed phase; see $raw" >&2
  exit 1
fi

/usr/local/bin/sample-proc "$out/proc.csv" "$pid" &
sampler=$!
sleep "$seconds"
kill -INT "-$pid" 2>/dev/null || true
for _ in $(seq 1 100); do
  kill -0 "$pid" 2>/dev/null || break
  sleep 0.1
done
kill -KILL "-$pid" 2>/dev/null || true
wait "$pid" 2>/dev/null || true
wait "$sampler" 2>/dev/null || true

python3 /usr/local/bin/parse-lame "$raw" "$out"
python3 /usr/local/bin/compare-results summarize "$out" lame
chown -R "$host_uid:$host_gid" "$out"
