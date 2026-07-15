#!/usr/bin/env bash
set -euo pipefail

changed=()

while IFS= read -r -d '' f; do
    [[ -f "$f" ]] || continue

    tmp=$(mktemp)
    cp "$f" "$tmp"
    fmtron --input "$tmp" >/dev/null

    if ! cmp -s "$f" "$tmp"; then
        changed+=("$f")
    fi

    rm -f "$tmp" "$tmp.bak"
done < <(
    git ls-files -z '*.ron' \
        ':!examples/modular_config_example/motors.ron' \
        ':!examples/cu_flight_controller/mcu_graph.ron'
)

if ((${#changed[@]})); then
    printf 'RON formatting check failed:\n'
    printf '%s\n' "${changed[@]}"
    exit 1
fi
