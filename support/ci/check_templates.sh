#!/usr/bin/env bash
set -euo pipefail

toolchain="${1:-stable}"
export CARGO_NAME="${CARGO_NAME:-Copper Tests}"
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$PWD/target}"
output="$PWD/target/ci-templates"
rm -rf "$output"
mkdir -p "$output"
cargo +"$toolchain" run -p cargo-cunew -- "$output/project" --template project --source local --copper-root "$PWD" --no-vcs
cargo +"$toolchain" run -p cargo-cunew -- "$output/workspace" --template workspace --source local --copper-root "$PWD" --no-vcs
cargo +"$toolchain" generate --path support/cargo_cunew/templates/cu_project --name cargo_generate_project --destination "$output" -d copper_source=local -d copper_root_path="$PWD" --silent
bash support/cargo_cunew/templates/smoke_generated.sh "$toolchain" "$output/project" "$output/workspace" "$output/cargo_generate_project"
