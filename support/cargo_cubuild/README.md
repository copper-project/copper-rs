# cargo cubuild

A small Cargo extension that expands the [`#[copper_runtime]`](https://github.com/copperhq/copper) macro in-place at the call site.

It replaces the macro invocation with the actual generated code, runs `cargo check`, and reports errors directly on the expanded code. This makes debugging macro-generated compilation errors vastly easier.

After the check completes (successfully or not), your original file is automatically restored.

## ✨ Features

- In-place macro expansion of `#[copper_runtime(...)]`
- Line-accurate diagnostics on the generated code
- Automatic backup and restoration of `src/main.rs`
- Requires no changes to your copper project

## 🔧 Usage

```bash
cargo install --path .
cargo cubuild
```

## example before with a simple cargo build

```
$ cargo build
error[E0277]: the trait bound `&[u8]: Reader` is not satisfied
  --> examples/cu_caterpillar/src/main.rs:6:1
   |
6  | #[copper_runtime(config = "copperconfig.ron")]
   | ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ the trait `Reader` is not implemented for `&[u8]`
   |
note: there are multiple different versions of crate `bincode` in the dependency graph
  --> /home/gbin/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/bincode-2.0.1/src/de/read.rs:17:1
```

## example after, with cargo cubuild
```
$ cargo cubuild
error[E0277]: the trait bound `&[u8]: Reader` is not satisfied
    --> examples/cu_caterpillar/src/main.rs:2629:48
     |
2629 |             let mut decoder = DecoderImpl::new(slice, config, ());
     |                               ---------------- ^^^^^ the trait `Reader` is not implemented for `&[u8]`
     |                               |
     |                               required by a bound introduced by this call
     |
note: there are multiple different versions of crate `bincode` in the dependency graph
[...]
```
