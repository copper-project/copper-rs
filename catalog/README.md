# Copper Catalog

This folder defines a contributor-friendly catalog for Copper components.

The source of truth is `index.ron`. Each entry points at a crate source and the
generator merges that source with Cargo package metadata.

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

## Generator

The generator lives in `support/cu_catalog`.

Local generation while iterating on this branch:

```bash
just catalog
```

That resolves GitHub entries from the local checkout with `--local-root`, then
renders:

- `catalog/generated/catalog.json`
- `catalog/generated/catalog.md`
- `catalog/generated/index.html`
