#!/usr/bin/env bash
set -euo pipefail

changed=()

while IFS= read -r -d '' f; do
    [[ -f "$f" ]] || continue

    if ! fmtron --check "$f" >/dev/null; then
        changed+=("$f")
    fi
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
