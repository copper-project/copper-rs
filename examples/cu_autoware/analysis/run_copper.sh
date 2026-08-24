#!/usr/bin/env bash
set -euo pipefail

seconds=${1:-60}
seconds=${seconds#seconds=}
out=analysis/data/copper
mkdir -p "$out" logs

cargo build --release -p cu-autoware --bins
../../target/release/calibrate --output "$out/copperconfig.ron"
analysis/sample_proc.sh "$out/proc.csv" -- ../../target/release/cu-autoware \
  --seconds "$seconds" --log-base logs/benchmark.copper --config "$out/copperconfig.ron"
../../target/release/kpi --log-base logs/benchmark.copper --out-dir "$out"
python3 analysis/compare_results.py summarize "$out" copper
