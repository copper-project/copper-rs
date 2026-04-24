#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
    echo "usage: $0 [output-dir]" >&2
    exit 1
fi

ROOT="$(git rev-parse --show-toplevel)"
OUTPUT_DIR="${1:-$ROOT/catalog/generated}"
DEPLOY_DIR=""

if [[ "$OUTPUT_DIR" != /* ]]; then
    OUTPUT_DIR="$ROOT/$OUTPUT_DIR"
fi

DEPLOY_DIR="$OUTPUT_DIR/catalog"

echo "building catalog deploy tree in $OUTPUT_DIR"

rm -rf "$DEPLOY_DIR" "$OUTPUT_DIR/assets"
rm -f "$OUTPUT_DIR/catalog.json" "$OUTPUT_DIR/catalog.md" "$OUTPUT_DIR/index.html"
mkdir -p "$DEPLOY_DIR"
cp -R "$ROOT/catalog/assets" "$DEPLOY_DIR/assets"
cp "$ROOT/catalog/index.ron" "$DEPLOY_DIR/index.ron"

cargo +stable run --manifest-path "$ROOT/support/cu_catalog/Cargo.toml" -- \
    generate \
    --index "$DEPLOY_DIR/index.ron" \
    --output-dir "$DEPLOY_DIR" \
    --local-root "$ROOT"

rm -f "$DEPLOY_DIR/index.ron"

echo "catalog deploy tree ready: $DEPLOY_DIR"
