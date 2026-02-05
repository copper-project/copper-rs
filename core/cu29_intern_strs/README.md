## cu29-intern-strs is a library to manage and intern strings in Rust

(see the crate `cu29-log-derive` for example macros taking advantage of this library)

The concept is to extract strings from a source code file and intern them in an external database at compile time.
This allows for efficient string management and reduces the size of the executable image.

All the strings are stored in a database file, which is generated at compile time and replaced by a simple index
number (u32) in the executable.

## Usage

To use this library, you need to add it as a dependency in your `Cargo.toml` file:

```toml
[dependencies]
cu29-intern-strs = { version = "*", feature = ["macros_debug"] }
```

To read the index after the fact:

```rust
use cu29_intern_strs::intern_strs;
fn main() {
    let index = read_interned_strings("path/to/cu29_log_index"); // this file is generated at compile time
    println!("Read string 42: {}", index.get(42).unwrap_or(&"String not found"));
}
```