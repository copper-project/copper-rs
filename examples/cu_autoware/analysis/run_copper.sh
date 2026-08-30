#!/usr/bin/env bash
set -euo pipefail

seconds=${1:-60}
seconds=${seconds#seconds=}
variant=${2:-vanilla}
case "$variant" in
  vanilla)
    out=analysis/data/copper
    log=logs/benchmark.copper
    system=copper
    features=()
    ;;
  hybrid)
    out=analysis/data/copper-hybrid
    log=logs/benchmark-hybrid.copper
    system=copper-hybrid
    features=(--features hybrid-background)
    ;;
  *)
    echo "usage: $0 [seconds] [vanilla|hybrid]" >&2
    exit 2
    ;;
esac
mkdir -p "$out" logs

cargo build --release -p cu-autoware "${features[@]}" --bins
../../target/release/calibrate --output "$out/copperconfig.ron"
analysis/sample_proc.sh "$out/proc.csv" -- ../../target/release/cu-autoware \
  --seconds "$seconds" --log-base "$log" --config "$out/copperconfig.ron"
../../target/release/kpi --log-base "$log" --out-dir "$out"
python3 analysis/compare_results.py summarize "$out" "$system"
