#!/usr/bin/env bash
set -euo pipefail

stable_image="ghcr.io/copper-project/copper-rs-ci:${CI_IMAGE_TAG}"
force_rebuild="${REFRESH_IMAGE}"
relevant_image_paths='^(support/docker/|support/ci/prepare_ci_image\.sh$|\.github/actions/ci-image/|\.github/workflows/build-ci-images\.yml$)'

if [[ "$GITHUB_EVENT_NAME" == "pull_request" ]]; then
  if [[ "${PR_HEAD_REPO:-}" != "$GITHUB_REPOSITORY" ]]; then
    {
      echo "image="
      echo "should_build=${REFRESH_IMAGE}"
      echo "tags=$stable_image"
    } >> "$GITHUB_OUTPUT"
    exit 0
  fi

  git fetch --no-tags --depth=1 origin "${BASE_SHA:-}"
  changed_files="$(git diff --name-only "${BASE_SHA:-}" "$GITHUB_SHA")"
  if grep -Eq "$relevant_image_paths" <<<"$changed_files"; then
    force_rebuild=true
  fi
elif [[ "$GITHUB_EVENT_NAME" == "push" ]]; then
  if [[ "${BEFORE_SHA:-}" != "0000000000000000000000000000000000000000" ]]; then
    git fetch --no-tags --depth=1 origin "${BEFORE_SHA:-}"
    changed_files="$(git diff --name-only "${BEFORE_SHA:-}" "$GITHUB_SHA")"
  else
    changed_files="$(git diff-tree --root --no-commit-id --name-only -r "$GITHUB_SHA")"
  fi
  if grep -Eq "$relevant_image_paths" <<<"$changed_files"; then
    force_rebuild=true
  fi
fi

if [[ "$force_rebuild" != "true" ]] && docker manifest inspect "$stable_image" >/dev/null 2>&1; then
  {
    echo "image=$stable_image"
    echo "should_build=false"
  } >> "$GITHUB_OUTPUT"
  exit 0
fi

run_image="ghcr.io/copper-project/copper-rs-ci:${CI_IMAGE_TAG}-run-${GITHUB_RUN_ID}-${GITHUB_RUN_ATTEMPT}"
{
  echo "image=$run_image"
  echo "should_build=true"
  echo "tags<<EOF"
  echo "$run_image"
  if [[ "$GITHUB_REF" == "refs/heads/master" ]]; then
    echo "$stable_image"
    echo "${stable_image}-${GITHUB_SHA}"
  fi
  echo "EOF"
} >> "$GITHUB_OUTPUT"
