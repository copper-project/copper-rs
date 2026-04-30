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

The bundled templates are also available directly for `cargo-generate` users at
`support/cargo_cunew/templates/`.
