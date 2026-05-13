#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

TOOLCHAIN="${COVERAGE_TOOLCHAIN:-stable}"
BASE_FEATURES="${BASE_FEATURES:-mock,cu-sensor-payloads/image,kornia,gst,faer,nalgebra,glam,debug_pane,bincode,log-level-debug}"
FEATURES_FLAG="--features ${BASE_FEATURES},python"
EMBEDDED_EXCLUDES="$(python3 support/ci/embedded_crates.py excludes --toolchain "$TOOLCHAIN")"

source <(cargo +"$TOOLCHAIN" llvm-cov show-env --sh)
cargo +"$TOOLCHAIN" llvm-cov clean --workspace

cargo +"$TOOLCHAIN" nextest run --all-targets --workspace $EMBEDDED_EXCLUDES
cargo +"$TOOLCHAIN" nextest run --all-targets --workspace $FEATURES_FLAG $EMBEDDED_EXCLUDES

RAYON_NUM_THREADS=1 COPPER_DETERMINISM_ITERS=256 COPPER_DETERMINISM_DT_TICKS=1000 \
    cargo +"$TOOLCHAIN" test -p cu-caterpillar --features determinism_ci \
    -- determinism_record_and_resim --test-threads=1

mkdir -p target/llvm-cov
cargo +"$TOOLCHAIN" llvm-cov report --html --output-dir target/llvm-cov/html
cargo +"$TOOLCHAIN" llvm-cov report --lcov --output-path target/llvm-cov/lcov.info
cargo +"$TOOLCHAIN" llvm-cov report --json --summary-only --output-path target/llvm-cov/summary.json

python3 - <<'PY'
import json
from pathlib import Path

root = Path("target/llvm-cov")
data = json.loads((root / "summary.json").read_text())
totals = data["data"][0]["totals"]

rows = []
for name in ("lines", "functions", "regions"):
    metric = totals[name]
    rows.append(
        f"| {name.title()} | {metric['covered']} | {metric['count']} | {metric['percent']:.2f}% |"
    )

(root / "summary.md").write_text(
    "\n".join(
        [
            "## Coverage Summary",
            "",
            "| Metric | Covered | Total | Percent |",
            "| --- | ---: | ---: | ---: |",
            *rows,
            "",
            "Coverage matches the Linux std CI test surfaces for nextest and the",
            "`cu-caterpillar` determinism regression. Doctests stay outside this",
            "report because `cargo-llvm-cov` requires nightly for doctest coverage.",
            "",
            "HTML report artifact: `coverage-report`",
        ]
    )
    + "\n"
)
PY
