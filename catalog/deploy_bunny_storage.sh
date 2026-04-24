#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 2 ]]; then
    echo "usage: $0 [generated-root] [remote-prefix]" >&2
    exit 1
fi

ROOT="$(git rev-parse --show-toplevel)"
LOCAL_DIR="${1:-$ROOT/catalog/generated}"
REMOTE_PREFIX="${2:-catalog}"
DRY_RUN="${BUNNY_DRY_RUN:-false}"
SOURCE_DIR=""
BUNNY_STORAGE_ZONE="copper"
BUNNY_STORAGE_ENDPOINT="ny.storage.bunnycdn.com"
CATALOG_PUBLIC_URL="https://cdn.copper-robotics.com/catalog/"

if [[ "$LOCAL_DIR" != /* ]]; then
    LOCAL_DIR="$ROOT/$LOCAL_DIR"
fi

REMOTE_PREFIX="${REMOTE_PREFIX#/}"
REMOTE_PREFIX="${REMOTE_PREFIX%/}"

if [[ -z "$REMOTE_PREFIX" ]]; then
    echo "remote prefix must not be empty" >&2
    exit 1
fi

SOURCE_DIR="$LOCAL_DIR/$REMOTE_PREFIX"

if [[ ! -d "$SOURCE_DIR" ]]; then
    echo "generated publish tree does not exist: $SOURCE_DIR" >&2
    exit 1
fi

if [[ ! -f "$SOURCE_DIR/index.html" ]]; then
    echo "catalog output is missing index.html: $SOURCE_DIR/index.html" >&2
    exit 1
fi

CATALOG_PUBLIC_URL="${CATALOG_PUBLIC_URL%/}/"

if [[ "$DRY_RUN" != "true" && "$DRY_RUN" != "1" ]]; then
    : "${BUNNY_STORAGE_PASSWORD:?set BUNNY_STORAGE_PASSWORD to your Bunny storage zone password}"
    if [[ "${BUNNY_SKIP_PURGE:-false}" != "true" && "${BUNNY_SKIP_PURGE:-false}" != "1" ]]; then
        : "${BUNNY_API_KEY:?set BUNNY_API_KEY to your bunny.net account API key or export BUNNY_SKIP_PURGE=1}"
    fi
fi

BASE_URL="https://${BUNNY_STORAGE_ENDPOINT}"
DELETE_URL="${BASE_URL}/${BUNNY_STORAGE_ZONE}/${REMOTE_PREFIX}"

tmp_body="$(mktemp)"
trap 'rm -f "$tmp_body"' EXIT

file_count="$(find "$SOURCE_DIR" -type f | wc -l | tr -d ' ')"

echo "deploying $file_count files from $SOURCE_DIR to ${BUNNY_STORAGE_ZONE}/${REMOTE_PREFIX} via ${BUNNY_STORAGE_ENDPOINT}"

if [[ "$DRY_RUN" == "true" || "$DRY_RUN" == "1" ]]; then
    echo "dry run: would delete ${DELETE_URL}"
else
    delete_status="$(
        curl \
            --silent \
            --show-error \
            --output "$tmp_body" \
            --write-out "%{http_code}" \
            --request DELETE \
            --url "$DELETE_URL" \
            --header "AccessKey: $BUNNY_STORAGE_PASSWORD"
    )"

    case "$delete_status" in
        200|201|202|204|404)
            ;;
        *)
            echo "failed to delete Bunny prefix ${REMOTE_PREFIX} (HTTP ${delete_status})" >&2
            cat "$tmp_body" >&2
            exit 1
            ;;
    esac
fi

while IFS= read -r -d '' file; do
    relative="${file#$SOURCE_DIR/}"
    upload_url="${BASE_URL}/${BUNNY_STORAGE_ZONE}/${REMOTE_PREFIX}/${relative}"
    echo "upload ${relative}"

    if [[ "$DRY_RUN" == "true" || "$DRY_RUN" == "1" ]]; then
        continue
    fi

    curl \
        --silent \
        --show-error \
        --fail \
        --request PUT \
        --url "$upload_url" \
        --header "AccessKey: $BUNNY_STORAGE_PASSWORD" \
        --header "Content-Type: application/octet-stream" \
        --upload-file "$file" \
        >/dev/null
done < <(find "$SOURCE_DIR" -type f -print0 | sort -z)

if [[ "${BUNNY_SKIP_PURGE:-false}" == "true" || "${BUNNY_SKIP_PURGE:-false}" == "1" ]]; then
    echo "skipping CDN purge for ${CATALOG_PUBLIC_URL}"
elif [[ "$DRY_RUN" == "true" || "$DRY_RUN" == "1" ]]; then
    echo "dry run: would purge ${CATALOG_PUBLIC_URL}"
else
    curl \
        --silent \
        --show-error \
        --fail \
        --request POST \
        --url "https://api.bunny.net/purge?url=${CATALOG_PUBLIC_URL}&async=false" \
        --header "AccessKey: $BUNNY_API_KEY" \
        >/dev/null
    echo "purged ${CATALOG_PUBLIC_URL}"
fi
