#!/bin/bash

set -e

TAG=$(grep '^version =' Cargo.toml | head -n 1 | sed -E 's/version = "([^"]+)"/v\1/g')

read -p "Creating new release for $TAG. Do you want to continue? [Y/n] " prompt

if [[ $prompt == "y" || $prompt == "Y" || $prompt == "yes" || $prompt == "Yes" ]]; then
    TAG=$TAG python scripts/prepare_changelog.py
    git add -A
    git commit -m "Prepare for release $TAG" || true && git push
    echo "Creating new git tag $TAG"
    git tag "$TAG" -m "$TAG"
    git push --tags
else
    echo "Cancelled"
    exit 1
fi
