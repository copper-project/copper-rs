# cargo-cunew

`cargo-cunew` is the Copper project bootstrap tool.

```bash
cargo install cargo-cunew
cargo cunew my_robot
cd my_robot
cargo run
```

By default it targets the latest stable Copper crates published on crates.io.
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
