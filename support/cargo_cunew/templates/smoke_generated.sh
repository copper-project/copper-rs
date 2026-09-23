#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "usage: $0 <toolchain> <project-dir> <workspace-dir> [cargo-generate-project-dir]" >&2
  exit 1
fi

toolchain="$1"
project_dir="$2"
workspace_dir="$3"
cargo_generate_project_dir="${4:-}"

smoke_project() {
  local dir="$1"
  (
    cd "$dir"
    cargo +"$toolchain" build
    cargo +"$toolchain" build --profile debug-optimized --features sim-debug --bins
    mkdir -p target/pgs
    cp copperconfig.ron target/pgs/selected.config.ron
    cargo +"$toolchain" check --features parallel-rt,pgs-candidate --bins
  )
}

smoke_workspace() {
  local dir="$1"
  (
    cd "$dir"
    cargo +"$toolchain" build
    cargo +"$toolchain" build --profile debug-optimized -p cu_example_app --features sim-debug --bins
    mkdir -p apps/cu_example_app/target/pgs
    cp apps/cu_example_app/copperconfig.ron apps/cu_example_app/target/pgs/selected.config.ron
    cargo +"$toolchain" check -p cu_example_app --features parallel-rt,pgs-candidate --bin cu_example_app-pgs-candidate
  )
}

smoke_project "$project_dir"
smoke_workspace "$workspace_dir"

if [[ -n "$cargo_generate_project_dir" ]]; then
  smoke_project "$cargo_generate_project_dir"
fi
