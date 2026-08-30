#!/usr/bin/env bash
# CPU and RSS of one process at ~10Hz, as t_s,cpu_pct,rss_mb. psrecord without the dependency.
#
#   sample_proc.sh data/proc.csv 12345
#   sample_proc.sh data/proc.csv -- ../../../target/release/cu-autoware 400
set -euo pipefail
export LC_ALL=C

if [ $# -lt 2 ]; then
    echo "usage: sample_proc.sh OUT.csv PID | sample_proc.sh OUT.csv -- COMMAND [ARGS...]" >&2
    exit 2
fi
out=$1
shift
mkdir -p "$(dirname "$out")"
if [ "$1" = "--" ]; then
    shift
    "$@" &
    pid=$!
    owned=1
    # We own this child: don't orphan it when the sampler is killed or errors out.
    trap 'kill "$pid" 2>/dev/null || true' EXIT INT TERM
else
    pid=$1
    owned=0
fi

# `comm` can hold spaces, so drop everything through it: the rest starts at stat field 3.
sample() {
    while [ -r "/proc/$pid/stat" ]; do
        line=$(< "/proc/$pid/stat") || break
        printf '%s %s\n' "$EPOCHREALTIME" "${line#*') '}"
        sleep 0.1
    done
}

# $13,$14 are utime,stime in clock ticks; $23 is the resident set in pages.
sample | awk -v hz="$(getconf CLK_TCK)" -v page="$(getconf PAGESIZE)" '
BEGIN { print "t_s,cpu_pct,rss_mb" }
{ cpu = ($13 + $14) / hz }
NR == 1 { t0 = $1 }
NR > 1 && $1 > t {
    printf "%.2f,%.1f,%.1f\n", $1 - t0, 100 * (cpu - c) / ($1 - t), $23 * page / 1048576
}
{ t = $1; c = cpu }
' > "$out"

# Propagate the child's exit status; an attached PID is not ours to wait on.
status=0
if [ "$owned" = 1 ]; then
    wait "$pid" || status=$?
    trap - EXIT
fi
echo "wrote $out" >&2
exit "$status"
