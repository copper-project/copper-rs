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
    cargo +"$toolchain" build --features sim-debug --bins
  )
}

smoke_workspace() {
  local dir="$1"
  (
    cd "$dir"
    cargo +"$toolchain" build
    cargo +"$toolchain" build -p cu_example_app --features sim-debug --bins
  )
}

smoke_project "$project_dir"
smoke_workspace "$workspace_dir"

if [[ -n "$cargo_generate_project_dir" ]]; then
  smoke_project "$cargo_generate_project_dir"
fi
