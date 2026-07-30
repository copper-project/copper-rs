#!/usr/bin/env bash
set -euo pipefail

workspace_root="${GITHUB_WORKSPACE:-$(git rev-parse --show-toplevel)}"
projects_root="$(dirname "$workspace_root")"

checkout_dependency() {
    local repository="$1"
    local branch="$2"
    local directory="$3"
    local destination="${projects_root}/${directory}"

    if [[ -d "${destination}/.git" ]]; then
        git -c safe.directory="$destination" -C "$destination" checkout "$branch"
        git -c safe.directory="$destination" -C "$destination" pull --ff-only origin "$branch"
    else
        git clone --depth 1 --branch "$branch" \
            "https://github.com/copper-project/${repository}.git" \
            "$destination"
    fi
}

checkout_dependency zed master zed
checkout_dependency cu-vitfly main cu-vitfly

# Copper-owned development dependencies must use the sibling checkouts above.
# Third-party release pins remain remote.
found_copper_git_dependency=0
for repository_root in \
    "$workspace_root" \
    "${projects_root}/zed" \
    "${projects_root}/cu-vitfly"; do
    if git -c safe.directory="$repository_root" -C "$repository_root" grep -nE \
        'git[[:space:]]*=[[:space:]]*"https://github\.com/copper-project/|\[patch\."https://github\.com/copper-project/' \
        -- Cargo.toml ':(glob)**/Cargo.toml'; then
        found_copper_git_dependency=1
    fi
done

if [[ "$found_copper_git_dependency" -ne 0 ]]; then
    echo "Copper-owned Cargo Git dependencies are forbidden; use local path dependencies." >&2
    exit 1
fi
