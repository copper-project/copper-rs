#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-$(git rev-parse --show-toplevel)}"
BASELINE_DIR="${ROOT}/api/v1"
UPDATE="${UPDATE_PUBLIC_API:-0}"
PUBLIC_API_TOOLCHAIN="${PUBLIC_API_TOOLCHAIN:-nightly-2026-06-25}"
CARGO_PUBLIC_API=(cargo "+${PUBLIC_API_TOOLCHAIN}" public-api)

CRATES=(
  cu29
  cu29-runtime
  cu29-traits
  cu29-derive
  cu29-export
  cu29-unifiedlog
)

mkdir -p "${BASELINE_DIR}"

status=0
for crate in "${CRATES[@]}"; do
  snapshot="${BASELINE_DIR}/${crate}.txt"
  current="$(mktemp)"

  "${CARGO_PUBLIC_API[@]}" --manifest-path "${ROOT}/Cargo.toml" -p "${crate}" -sss --color never > "${current}"

  if [[ "${UPDATE}" == "1" ]]; then
    cp "${current}" "${snapshot}"
    echo "updated ${snapshot#${ROOT}/}"
  elif [[ ! -f "${snapshot}" ]]; then
    echo "missing API snapshot: ${snapshot#${ROOT}/}" >&2
    status=1
  elif ! diff -u "${snapshot}" "${current}"; then
    status=1
  fi

  rm -f "${current}"
done

exit "${status}"
