#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(git rev-parse --show-toplevel)"
projects_root="$(dirname "$workspace_root")"

checkout_dependency() {
    local repository="$1"
    local branch="$2"
    local directory="$3"
    local destination="${projects_root}/${directory}"

    if [[ -d "${destination}/.git" ]]; then
        git -C "$destination" checkout "$branch"
        git -C "$destination" pull --ff-only origin "$branch"
    else
        git clone --depth 1 --branch "$branch" \
            "https://github.com/copper-project/${repository}.git" \
            "$destination"
    fi
}

checkout_dependency zed master zed
checkout_dependency cu-vitfly main cu-vitfly

if rg -n --glob Cargo.toml \
    '(^|[,{[:space:]])(git|branch|rev|tag)[[:space:]]*=|\[patch\."https?://' \
    "$workspace_root" "${projects_root}/zed" "${projects_root}/cu-vitfly"; then
    echo "Cargo Git dependencies are forbidden; use local path dependencies." >&2
    exit 1
fi
