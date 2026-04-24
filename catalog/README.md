# Copper Catalog

This folder defines a contributor-friendly catalog for Copper components.

The source of truth is `index.ron`. Each entry points at a crate source and the
generator merges that source with Cargo package metadata. Keep `entries` sorted
alphabetically by `name`.

## Metadata Model

Use normal Cargo package metadata for the generic package fields:

- `description`
- `keywords`
- `categories`
- `repository`
- `homepage`
- `license`

Use `[package.metadata.copper]` for Copper-specific classification:

```toml
[package.metadata.copper]
kind = "source"
domains = ["sensor/camera"]
environments = ["host"]
host_os = ["linux"]
```

The flattened fields are:

- `kind`
  - One of `source`, `sink`, `bridge`, `task`, `payload`, `resource`, `monitor`,
    `codec`, `lib`, `testing`
- `domains`
  - A controlled list like `sensor/lidar`, `algorithm/control`,
    `resource/board-bundle`
- `environments`
  - Any of `host`, `embedded`
- `host_os`
  - Optional host platform hints like `linux`, `macos`, `windows`
- `embedded_targets`
  - Optional Rust embedded targets like `thumbv7em-none-eabihf`

Derived values:

- `std` is shown when `host` is present
- `no_std` is shown when `embedded` is present
- an embedded-only crate is represented as `environments = ["embedded"]`

## Visual Assets

- `catalog/assets/components/<catalog_name>.<ext>`
  - Crate-specific visuals used by the generated catalog
- `catalog/assets/domains/<domain>.<ext>`
  - Shared fallback, with `/` replaced by `-`
  - Example: `sensor/camera` becomes `sensor-camera.svg`
- `catalog/assets/kinds/<kind>.<ext>`
  - Fallback for a Copper kind like `source`, `task`, or `bridge`

The generated catalog resolves visuals in this order:

1. component asset
2. domain asset
3. kind asset
4. `unclassified` kind asset

## Index Format

`index.ron` is a list of source entries. For the current branch experiment all
entries point at GitHub on `gbin/catalog`, and the generator can resolve them
locally before that branch is pushed.

```ron
(
  version: 1,
  defaults: (
    github_repo: "copper-project/copper-rs",
    github_rev: "gbin/catalog",
  ),
  entries: [
    (
      name: "cu-v4l",
      source: GitHub(path: "components/sources/cu_v4l"),
    ),
  ],
)
```

Supported sources:

- `GitHub`
  - Fetches `Cargo.toml` from the GitHub repo/path/rev
  - Also fetches the repo root `Cargo.toml` so `*.workspace = true` fields work
- `CratesIo`
  - Fetches package metadata from the crates.io API
  - For now, Copper-specific classification should come from `overrides`

Overrides are optional and exist for third-party crates that do not publish
Copper metadata yet.

For external crates you want to preview before they add Copper metadata, set:

```ron
overrides: (
  allow_missing_copper_metadata: true,
)
```

Those entries still render in the catalog, but they are marked as needing
metadata instead of being treated as classified Copper components.

## Generator

The generator lives in `support/cu_catalog`.

Local generation while iterating on this branch:

```bash
just
```

That resolves GitHub entries from the local checkout with `--local-root`, then
builds the publish tree under `generated/`, with `generated/` representing the
CDN root:

- `generated/catalog/catalog.json`
- `generated/catalog/catalog.md`
- `generated/catalog/index.html`
- `generated/catalog/assets/...`

## Deploying To Bunny

`catalog/generated` is the local mirror of what gets published. Build it with:

```bash
cd catalog && just build-generated
```

Preview the built catalog locally at:

```bash
xdg-open catalog/generated/catalog/index.html
```

Publish that exact generated tree with the same path CI uses:

```bash
cd catalog && just deploy
```

Preview the upload plan without touching Bunny:

```bash
cd catalog && just deploy-dry-run
```

Local deploy expects:

- `BUNNY_STORAGE_ZONE`
- `BUNNY_STORAGE_PASSWORD`
- `BUNNY_STORAGE_ENDPOINT`
  - defaults to `storage.bunnycdn.com`, but set this to your zone's actual region endpoint
- `BUNNY_API_KEY`
  - required unless you export `BUNNY_SKIP_PURGE=1`
- `CATALOG_PUBLIC_URL`
  - defaults to `https://cdn.copper-robotics.com/catalog/`

The GitHub Actions deploy job uses the same script and expects a protected
environment named `catalog-production` with:

- secret `BUNNY_STORAGE_PASSWORD`
- secret `BUNNY_API_KEY`
- variable `BUNNY_STORAGE_ZONE`
- variable `BUNNY_STORAGE_ENDPOINT`
- variable `CATALOG_PUBLIC_URL`
