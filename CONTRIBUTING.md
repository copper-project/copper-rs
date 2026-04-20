# Contributing to Copper-rs

First off, thank you for considering contributing to Copper-rs! We welcome contributions from everyone. This document provides guidelines for contributing to the project.

## Getting Started

### Prerequisites

* **Rust:** Ensure you have a recent stable Rust toolchain installed. You can install it using [rustup](https://rustup.rs/).
    ```bash
    curl --proto '=https' --tlsv1.2 -sSf [https://sh.rustup.rs](https://sh.rustup.rs) | sh
    ```
* **just:** We use [just](https://github.com/casey/just) as a command runner for common development tasks. Install it via cargo or your package manager:
    ```bash
    cargo install just
    ```
* **cargo-nextest:** For running tests efficiently.
    ```bash
    cargo install cargo-nextest
    ```
* **typos:** For spell checking.
    ```bash
    cargo install typos-cli
    ```
* **Platform-Specific Dependencies:** Depending on your operating system and the features you intend to work with, you might need additional dependencies. For Ubuntu 22.04, please checkout our [Dockerfile](support/docker/Dockerfile.ubuntu).


### Using justfile (Recommended)

The project includes a `justfile` that mirrors the CI workflow. This is the recommended way to run checks locally before submitting a PR:

```bash
# Run the default PR pipeline (format + lint + tests for std/no_std)
just pr-check

# Same as above (default target)
just

# Run lint checks (fmt-check + typos + clippy for std and no_std)
just lint

# Run unit tests (std + no_std)
just test

# Run CI-aligned std checks (build/tests/templates)
just std-ci

# Run CI-aligned embedded/no_std checks
just nostd-ci

# Run in release mode
just std-ci release

# Run with CUDA features
just std-ci cuda-release
```

The `just pr-check` command runs:
- Formatting (`just fmt`)
- Lint checks (`just lint`: format check + typos + std/no_std clippy)
- Unit tests (`just test`: std + no_std)

The `just std-ci` command remains CI-aligned for std paths, including:
- Format check (`just fmt-check`)
- Typos check (`typos -c .config/_typos.toml`)
- Clippy with `--deny warnings`
- Build with all features
- Unit tests with cargo-nextest
- Project generation tests (debug mode only)

### Platform Support Tiers

The public platform list lives in the [Supported Platforms](https://copper-project.github.io/copper-rs/Supported-Platforms) wiki page. For contribution and release purposes, platform support is tied to the validation we actually run. Do not promote a platform to a stronger support tier unless the matching CI or release validation exists.

| Tier | Platform / target | Current validation scope |
| --- | --- | --- |
| Tier 1: CI-tested and release-blocking | Linux x86_64 | Full std workspace build/test/clippy, feature coverage, generated template builds, and the `cu-caterpillar` determinism regression. |
| Tier 1: CI-tested and release-blocking | macOS GitHub runner | Std workspace build/test and generated template builds. CUDA is excluded, and clippy is not currently part of the macOS PR gate. |
| Tier 1: CI-tested and release-blocking | Windows x86_64 | Core crates build/test only. The full workspace is not currently a Windows support guarantee. |
| Tier 1: CI-tested and release-blocking | Core `no_std` | Core build/tests with default features disabled. |
| Tier 1: CI-tested and release-blocking | Embedded crates and RP2350 skeleton | Embedded-only crate build/clippy plus RP2350 firmware/host skeleton cross-build and clippy coverage. This is compile-time validation, not hardware-in-the-loop validation. |
| Tier 2: expected to work | Linux aarch64, SBCs, and Jetson-class targets | Documented deployment path and project intent, but not equivalent to the full PR CI matrix. |
| Tier 2: expected to work | Other maintained embedded targets | Expected when using maintained embedded crates and supported HAL/resource patterns. Board-specific behavior depends on separate hardware validation. |
| Tier 3: experimental | Android arm64 | Listed as a deployment target, but not currently covered by release-blocking CI. |
| Tier 3: experimental | Linux armv7 and riscv64 | Listed as deployment targets, but not currently covered by release-blocking CI. |
| Tier 3: experimental | Bare-metal boards beyond maintained examples | Experimental unless they have explicit CI, release-checklist, or hardware validation coverage. |

When changing shared runtime, macro, trait, logging, replay, resource, or embedded-facing code, consider the Tier 1 surfaces affected by the change. In practice, this usually means running `just std-ci` for host/runtime changes and `just nostd-ci` for `no_std` or embedded-facing changes before requesting review.

### Prek hooks (Recommended)

To mirror the CI lint steps locally, you can install prek hooks that run formatting and typos checks on each commit.

1. Install `prek`
    ```bash
    cargo binstall install prek
    # or
    cargo install install prek
    ```
2. Reinstall the git hooks if you previously executed `pre-commit install`
    ```bash
    prek install -f
    ```

3. Run once to verify:
    ```bash
    prek run --all-files
    ```

If you want to customize prek, please modify `.pre-commit-config.yaml`.

### Proc-macro expansion (cargo expand)

Prefer `cargo expand` when inspecting generated code from Copper proc-macros.
Install it once with:

```bash
cargo install cargo-expand
```

Then use the root `justfile` helpers:

```bash
# Expand a crate that uses cu29_derive (provide your target)
just expand-runtime pkg=<crate_name> bin=<bin_name> [features=<comma_separated_features>]

# Expand the SoA derive tests (default target: proctest)
just expand-soa
```

You can also run `cargo expand -p <crate> --bin <bin>` directly when preferred.

### Building the Project

To build the project, navigate to the root directory and run:

```bash
cargo build --workspace
```

For a release build, use:

```bash
cargo build --release --workspace
```

### Running Tests

We use cargo-nextest for running tests. To run all unit tests:

```bash
cargo nextest run --workspace --all-targets
```

To run tests including specific features (matching the CI 'debug' mode on Linux):

```bash
cargo nextest run --workspace --all-targets --features mock,image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode,log-level-debug
```

## Contribution Workflow

1. **Fork the Repository**: Create your own fork of the copper-rs repository on GitHub.

2. **Clone Your Fork**: Clone your forked repository to your local machine.
    ```bash
    git clone https://github.com/YOUR_USERNAME/copper-rs.git
    cd copper-rs
    ```

3. **Create a Branch**: Create a new branch for your changes. Choose a descriptive branch name. ex: ```yang/chore/beta_clippy```, ```gbin/feat/soa```, ```gbin/fix/balancebot_sim```.
    ```bash
    git checkout -b user/feat/description
    ```

4. **Make Changes**: Implement your feature or fix the bug. Write clear and concise code.

5. **Run Checks**: Before committing, ensure your code adheres to the project's standards. The default recommendation is `just pr-check` (or simply `just`). For CI-aligned paths, run `just std-ci` and/or `just nostd-ci` as applicable. You can also run individual checks:
    - **All checks (recommended)**: ```just pr-check```
    - **Std CI flow**: ```just std-ci```
    - **Embedded/no_std CI flow**: ```just nostd-ci```
    - **Lint only**: ```just lint``` (format check + typos + std/no_std clippy)
    - **Tests only**: ```just test``` (std + no_std unit tests)
    - **Formatting check**: ```just fmt-check```
    - **Typos**: ```typos -c .config/_typos.toml```
6. **Commit and push Changes**: Commit your changes with clear and descriptive commit messages. (Consider using [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/))
    ```bash
    git add .
    git commit -m "feat: new feature"
    git push user/feat/description
    ```
7. **Create a Pull Request**: Open a pull request (PR) from your branch to the master branch of the main copper-rs repository. Provide a clear description of your changes in the PR.

## Contributing Components

Reusable Copper components are welcome, but they do not always need to live inside the main `copper-rs` repository.

If you are building a driver, task, bridge, or other generally useful component, the preferred path is usually:

1. Publish it as its own crate or repository so it can evolve on its own release cycle.
2. Make sure it is documented clearly and works well as a standalone Copper component.
3. Add it to the [Copper Component Catalog](https://cdn.copper-robotics.com/catalog/index.html) so users can discover it easily.

This keeps the core repository focused while still growing the Copper ecosystem. If you are unsure whether a component belongs in the main workspace or should live as an external package, open an issue or start a discussion first.

## Dependency Management

We keep our dependencies minimal and up-to-date.

We use ```cargo-shear``` to identify and remove unused dependencies. Before submitting a PR, please run:

1. Install ```cargo-shear```
```bash
cargo install cargo-shear
```

2. Run ```cargo-shear``` to find unused dependencies:
```bash
cargo shear
```

3. Handling False Positives

```cargo-shear``` is not perfect. If it incorrectly flags a necessary dependency (e.g., required by a feature flag or build script), add it to the ignored list in the relevant Cargo.toml file:

```toml
[package.metadata.cargo-shear]
# Explain *why* the dependency is needed despite not being directly used in code.
# e.g. "Required for X feature" or "Used in build.rs"
ignored = ["some-crate", "another-crate"]
```

## Reporting Issues
If you encounter a bug or have a feature suggestion, please open an issue on the GitHub repository. Provide as much detail as possible, including:
1. A clear description of the issue or feature.
2. Steps to reproduce the bug (if applicable).
3. Your operating system and Rust version.
4. Relevant logs or error messages.
