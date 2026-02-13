# cu29-soa-derive

Proc-macro derive for fixed-size Struct-of-Arrays (SoA) layouts.

`#[derive(Soa)]` generates a `<Type>Soa<const N: usize>` companion structure
from a plain Rust struct, plus helpers for indexing and range access.

This is useful in data pipelines where SoA layout improves cache behavior or
vectorization opportunities.

## Features

- `macro_debug`: emits extra macro expansion debug output.

