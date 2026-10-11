# cargo-cunew

`cargo-cunew` is the Copper project bootstrap tool.

```bash
cargo install cargo-cunew
cargo cunew my_robot
cd my_robot
cargo run
```

By default it selects the latest stable version of each Copper dependency in the
installed tool's minor release line. Generated requirements use `~` so patch
updates remain compatible.

Templates are bundled and rendered with Liquid. Registry queries use synchronous
HTTPS with Rustls; installation needs a Rust toolchain and generation initializes
a repository with `git` (or skips it with `--no-vcs`). The first installation and
application build compile their dependencies; elapsed time depends on your
machine and existing Cargo cache.

Run `just cunew-check` to test generation, lint the tool, and enforce its
dependency budget. Run `just released-template-check` to build both templates
against crates.io and run their main applications.
It also supports:

- `--source git` for a git-based Copper dependency setup
- `--source local --copper-root /path/to/copper-rs` for a local checkout
- `--template workspace` for the multi-crate workspace scaffold
- `--target bare-metal` to omit host-only profile-guided scheduling files and recipes

Interactive generation asks whether the project targets bare metal/no_std. For
noninteractive generation, `--target host` is the default. Both templates offer
`graph[-log]` and `sched[-log]`; host templates also include the
four-step `pgs-*` workflow.
The target choice controls PGS scaffolding; embedded runtime and dependency setup
still depends on the board and must be adapted separately.

The bundled templates are also available directly for `cargo-generate` users at
`support/cargo_cunew/templates/`.
