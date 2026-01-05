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
* **Platform-Specific Dependencies:** Depending on your operating system and the features you intend to work with, you might need additional dependencies. For Ubuntu 22.04, please checkout our [Dockerfile](support/docker/Dockerfile).


### Using justfile (Recommended)

The project includes a `justfile` that mirrors the CI workflow. This is the recommended way to run checks locally before submitting a PR:

```bash
# Run the full CI-aligned check (lint + clippy + build + tests)
just std-ci

# Run only lint checks (formatting + typos)
just lint

# Run embedded/no_std CI checks
just nostd-ci

# Run in release mode
just std-ci release

# Run with CUDA features
just std-ci cuda-release
```

The `just std-ci` command runs the same checks as CI, including:
- Format check (`cargo +stable fmt --all -- --check`)
- Typos check (`typos -c .config/_typos.toml`)
- Clippy with `--deny warnings`
- Build with all features
- Unit tests with cargo-nextest
- Project generation tests (debug mode only)

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

5. **Run Checks**: Before committing, ensure your code adheres to the project's standards. The easiest way is to run `just std-ci` which executes all CI checks. Alternatively, run individual checks:
    - **All checks (recommended)**: ```just std-ci```
    - **Lint only**: ```just lint``` (formatting + typos)
    - **Formatting**: ```cargo +stable fmt --all -- --check```
    - **Clippy Lints**: ```cargo +stable clippy --workspace --all-targets -- --deny warnings```
    - **Tests**: ```cargo +stable nextest run --workspace --all-targets``` (run with relevant features if applicable)
    - **Typos**: ```typos -c .config/_typos.toml```
6. **Commit and push Changes**: Commit your changes with clear and descriptive commit messages. (Consider using [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/))
    ```bash
    git add .
    git commit -m "feat: new feature"
    git push user/feat/description
    ```
7. **Create a Pull Request**: Open a pull request (PR) from your branch to the master branch of the main copper-rs repository. Provide a clear description of your changes in the PR.

## Dependency Management

We keep our dependencies minimal and up-to-date.

We use ```cargo-machete``` to identify and remove unused dependencies. Before submitting a PR, please run:

1. Install ```cargo-machete```
```bash
cargo install cargo-machete
```

2. Run ```cargo-machete``` to find unused dependencies:
```bash
cargo machete --with-metadata
```

3. Handling False Positives

```cargo-machete``` is not perfect. If it incorrectly flags a necessary dependency (e.g., required by a feature flag or build script), add it to the ignored list in the relevant Cargo.toml file:

```toml
[package.metadata.cargo-machete]
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
