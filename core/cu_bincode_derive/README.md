# cu-bincode-derive

This is a **hard fork of bincode 2 (2.0.1) from crates.io** (see first verbatim commit). Do not contact the original authors as per their wishes.

The derive crate for cu-bincode. Implements `cu_bincode::Encode` and `cu_bincode::Decode`.

This crate is roughly split into 2 parts:

# Parsing

Most of parsing is done in the `src/parse/` folder. This will generate the following types:
- `Attributes`, not being used currently
- `Visibility`, not being used currently
- `DataType` either `Struct` or `Enum`, with the name of the data type being parsed
- `Generics` the generics part of the type, e.g. `struct Foo<'a>`
- `GenericConstraints` the "where" part of the type

# Generate

Generating the code implementation is done in either `src/derive_enum.rs` and `src/derive_struct.rs`.

This is supported by the structs in `src/generate`. The most notable points of this module are:
- `StreamBuilder` is a thin but friendly wrapper around `TokenStream`
- `Generator` is the base type of the code generator. This has helper methods to generate implementations:
  - `ImplFor` is a helper struct for a single `impl A for B` construction. In this functions can be defined:
    - `GenerateFnBody` is a helper struct for a single function in the above `impl`. This is created with a callback to `FnBuilder` which helps set some properties. `GenerateFnBody` has a `stream()` function which returns ` StreamBuilder` for the function.

For additional derive testing, see the test cases in `../tests`

For testing purposes, all generated code is outputted to the current `target/generated/cu_bincode` folder, under file name `<struct/enum name>_Encode.rs` and `<struct/enum name>_Decode.rs`. This can help with debugging.


## Runtime-only fields (2.1)

`#[bincode(skip)]` omits a field from `Encode` and restores `Default::default()`
in `Decode` and `BorrowDecode`. Add `default = "path::to::function"` inside the
same attribute to call a zero-argument default function instead. Named and tuple
fields in structs and enum variants are supported. Skipped fields need no codec
traits. The `default` option requires `skip`; `with_serde` and `skip` are mutually
exclusive. See the root README for examples and compatibility implications.
