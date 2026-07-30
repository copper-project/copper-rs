# Contributing to Copper

Thank you for helping build a deterministic, production-ready robotics runtime.
Contributions of code, components, documentation, examples, and careful bug reports
are all welcome.

## Choose Where to Start

| You want to… | Best next step |
| --- | --- |
| Ask a usage question | Search the [documentation](https://copper-project.github.io/copper-rs/) or ask on [Discord](https://discord.gg/VkCG7Sb9Kw). |
| Report a bug or request a feature | Open the matching [issue template](https://github.com/copper-project/copper-rs/issues/new/choose). |
| Propose a runtime or architectural change | Start a [Discussion](https://github.com/copper-project/copper-rs/discussions) before investing in a large implementation. |
| Add a reusable driver, task, bridge, or payload | Read [Contributing Components](#contributing-components); an independent crate is often the best home. |
| Improve docs or examples | Open a focused pull request and update related wiki/book material when behavior and documentation move together. |
| Report a vulnerability | Follow the private process in [SECURITY.md](SECURITY.md). Do not open a public issue. |

All community participation is governed by our
[Code of Conduct](CODE_OF_CONDUCT.md).

## Set Up the Workspace

Copper's minimum supported Rust version is 1.95. Install the latest stable Rust
toolchain with [rustup](https://rustup.rs/), then install
[`just`](https://just.systems/) and clone your fork:

```bash
cargo install just
git clone https://github.com/YOUR_USERNAME/copper-rs.git
cd copper-rs
```

The Ubuntu development dependencies are documented in
[`support/docker/Dockerfile.ubuntu`](support/docker/Dockerfile.ubuntu). Other
platforms may require equivalent system packages.

<details>
<summary><strong>One-time tools used by the full PR check</strong></summary>

The root `justfile` is the source of truth. Its checks use `cargo-nextest`,
`typos-cli`, `taplo-cli`, `fmtron`, and `prek`:

```bash
cargo install --locked cargo-nextest typos-cli taplo-cli fmtron prek
```

The public API check also pins a nightly toolchain and `cargo-public-api` version.
Run `just api-check`; if either is missing, it prints the exact versioned install
command required by the current repository.

Coverage additionally requires `cargo-llvm-cov` and `llvm-tools-preview`:

```bash
cargo install --locked cargo-llvm-cov
rustup component add llvm-tools-preview --toolchain stable
```

</details>

## Contribution Workflow

1. Create a branch from `master` using `user/kind/description`, for example
   `alex/fix/replay-seek`.
2. Make one focused change. Add tests and update documentation or examples where
   the behavior needs them.
3. Run `just` from the repository root. It formats the workspace, runs lint and
   API checks, and tests the std and `no_std` surfaces.
4. Review the complete diff, then commit with a clear message. Conventional
   Commits are welcome but not required.
5. Push your branch and open a pull request to `master`. Explain the problem, the
   chosen approach, and how you verified it.

The pull request template contains the final submission checklist.

## Checks

Prefer the root `justfile` over copying long Cargo command lines:

| Change | Run |
| --- | --- |
| Most pull requests | `just` or `just pr-check` |
| Formatting, typos, and clippy only | `just lint` |
| Host/runtime behavior across the std feature matrix | `just std-ci` |
| Shared, embedded-facing, or `no_std` code | `just nostd-ci` |
| Coverage-sensitive behavior | `just coverage` |
| Runtime proc-macro expansion | `just expand-runtime pkg=<crate> bin=<bin> [features=<features>]` |
| SoA derive expansion | `just expand-soa` |

Linux is the primary full-workspace gate. macOS is also release-blocking, Windows
validates a reduced core surface, and embedded coverage is compile-time rather than
hardware-in-the-loop. See
[Supported Platforms](https://copper-project.github.io/copper-rs/Supported-Platforms)
for the public platform matrix.

## Design Expectations

Copper has a deliberately opinionated architecture. Contributions should preserve
these properties:

- **Static over dynamic:** prefer types, compile-time wiring, and generated code to
  runtime string lookup or mutable graph topology.
- **Realtime paths stay lean:** do not add allocations, copies, serialization
  passes, or latency to the hot path without explicit design agreement.
- **`no_std` is a real target:** shared crates, traits, and macros must not assume
  host-only APIs.
- **Determinism and replay are product features:** runtime changes must preserve
  unified logging and reproducible replay.
- **Use Copper's logs:** inspect recorded CopperLists and structured logs before
  adding ad hoc text instrumentation.
- **Keep abstractions understandable:** solve the underlying design problem instead
  of hiding it behind runtime magic or invisible environment variables.

When proc-macro behavior is unclear, use the expansion recipes above. When a runtime
failure already has a `.copper` log, prefer extraction and resimulation before adding
new instrumentation.

## Contributing Components

Reusable components do not always need to live in this monorepo. For a driver, task,
bridge, payload, or monitor, the preferred path is usually:

1. Publish it as an independent crate or repository so it can evolve on its own
   release cycle.
2. Document its supported targets, configuration, and usage as a standalone Copper
   component.
3. Add it to the
   [Copper Component Catalog](https://cdn.copper-robotics.com/catalog/index.html).

Open an issue or Discussion first if the component must be maintained in the main
workspace or changes a shared Copper interface.

<details>
<summary><strong>Dependency changes</strong></summary>

Keep dependencies minimal and explain why each new dependency is required. Run
`cargo shear` when changing manifests:

```bash
cargo install --locked cargo-shear
cargo shear
```

If `cargo-shear` flags a feature-gated or build-time dependency incorrectly, document
the exception next to the affected package:

```toml
[package.metadata.cargo-shear]
# Required by build.rs for generated bindings.
ignored = ["some-crate"]
```

</details>

<details>
<summary><strong>Optional local commit hooks</strong></summary>

The repository's `prek` hooks run file hygiene, formatting, and typo checks:

```bash
prek install -f
prek run --all-files
```

Update `.pre-commit-config.yaml` if the shared hook configuration needs to change.

</details>

Maintainer release branches, versioning, tagging, and backporting are documented in
[RELEASING.md](RELEASING.md).
