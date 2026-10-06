# cu-bincode
[![CI](https://github.com/copper-project/copper-rs/actions/workflows/general.yml/badge.svg)](https://github.com/copper-project/copper-rs/actions)
[![](https://img.shields.io/crates/v/cu-bincode.svg)](https://crates.io/crates/cu-bincode)
[![](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![](https://img.shields.io/badge/discord-join-5865F2?logo=discord&logoColor=white)](https://discord.com/invite/VkCG7Sb9Kw)

This is a **hard fork of bincode 2 (2.0.1) from crates.io** (see first verbatim commit). Do not contact the original authors as per their wishes.

A compact encoder / decoder pair that uses a binary zero-fluff encoding scheme.
The size of the encoded object will be the same or smaller than the size that
the object takes up in memory in a running Rust program.

In addition to exposing two simple functions
(one that encodes to `Vec<u8>`, and one that decodes from `&[u8]`),
binary-encode exposes a Reader/Writer API that makes it work
perfectly with other stream-based APIs such as Rust files, network streams,
and the [flate2-rs](https://github.com/rust-lang/flate2-rs) compression
library.

## [API Documentation](https://docs.rs/cu-bincode/)

## Bincode in the Wild

* [copper-rs](https://github.com/copper-project/copper-rs): copper-rs uses Bincode.
* [google/tarpc](https://github.com/google/tarpc): Bincode is used to serialize and deserialize networked RPC messages.
* [servo/webrender](https://github.com/servo/webrender): Bincode records WebRender API calls for record/replay-style graphics debugging.
* [servo/ipc-channel](https://github.com/servo/ipc-channel): IPC-Channel uses Bincode to send structs between processes using a channel-like API.
* [ajeetdsouza/zoxide](https://github.com/ajeetdsouza/zoxide): zoxide uses Bincode to store a database of directories and their access frequencies on disk.

## Example

```rust
# extern crate cu_bincode as bincode;
use bincode::{config, Decode, Encode};

#[derive(Encode, Decode, PartialEq, Debug)]
struct Entity {
    x: f32,
    y: f32,
}

#[derive(Encode, Decode, PartialEq, Debug)]
struct World(Vec<Entity>);

fn main() {
    let config = config::standard();

    let world = World(vec![Entity { x: 0.0, y: 4.0 }, Entity { x: 10.0, y: 20.5 }]);

    let encoded: Vec<u8> = bincode::encode_to_vec(&world, config).unwrap();

    // The length of the vector is encoded as a varint u64, which in this case gets collapsed to a single byte
    // See the documentation on varint for more info for that.
    // The 4 floats are encoded in 4 bytes each.
    assert_eq!(encoded.len(), 1 + 4 * 4);

    let (decoded, len): (World, usize) = bincode::decode_from_slice(&encoded[..], config).unwrap();

    assert_eq!(world, decoded);
    assert_eq!(len, encoded.len()); // read all bytes
}
```

## Selective encoding in 2.1

`Uleb128<T>` opts individual integers into ULEB128 without changing the default
format or configuration of other values. Signed integers use zigzag followed by
ULEB128. It supports all Rust integer widths, needs no allocation, and works with
`no_std`, including without the `alloc` feature.

```rust
use cu_bincode::{config, decode_from_slice, encode_into_slice, Uleb128};

let mut bytes = [0; 32];
let len = encode_into_slice((300u32, Uleb128(-150i128)), &mut bytes, config::standard()).unwrap();
assert_eq!(&bytes[..len], &[251, 44, 1, 171, 2]);
let (value, used) = decode_from_slice::<(u32, Uleb128<i128>), _>(&bytes[..len], config::standard()).unwrap();
assert_eq!(value, (300, Uleb128(-150)));
assert_eq!(used, len);
```

Use the same wrapper and integer type when decoding. The wrapper is independent
of the surrounding integer encoding and endianness. It does not switch the
encoding of IDs, lengths, or payloads alongside it. Decoding enforces integer
widths and the configured allocation budget, just like ordinary integers.

The native derives also support runtime-only fields:

```rust
# extern crate cu_bincode as bincode;
use bincode::{Decode, Encode};

fn restored_state() -> u8 { 5 }

#[derive(Encode, Decode)]
struct Record {
    id: u64,
    #[bincode(skip, default = "restored_state")]
    runtime_state: u8,
    #[bincode(skip)]
    cache: u32,
    payload: u32,
}
```

Skipped fields consume no bytes. `Decode` and `BorrowDecode` initialize them with
`Default::default()` or the named zero-argument function. This works on named and
tuple fields in structs and enum variants. Generic parameters used only in
skipped fields do not require codec traits; default-initialized field types need
`Default`. Custom default functions' bounds must be supplied by the type or the
existing derive bound overrides. `default` requires `skip`, and `skip` cannot be
combined with `with_serde`. Use Serde's own `#[serde(skip)]` separately when also
deriving Serde traits.

Ordinary encodings are unchanged in 2.1. Adding a wrapper or skipping a previously
encoded field intentionally changes that application's layout; update its readers
and format version together.

## Specification

Bincode's format is specified in [docs/spec.md](https://github.com/copper-project/copper-rs/blob/master/core/cu_bincode/docs/spec.md).

## FAQ

### Is Bincode suitable for storage?

The encoding format is stable, provided the same configuration is used.
This should ensure that later versions can still read data produced by a previous versions of the library if no major version change
has occurred.

Bincode 1 and 2 are completely compatible if the same configuration is used.

Bincode is invariant over byte-order, making an exchange between different
architectures possible. It is also rather space efficient, as it stores no
metadata like struct field names in the output format and writes long streams of
binary data without needing any potentially size-increasing encoding.

As a result, Bincode is suitable for storing data. Be aware that it does not
implement any sort of data versioning scheme or file headers, as these
features are outside the scope of this crate.

### Is Bincode suitable for untrusted inputs?

Bincode attempts to protect against hostile data. There is a maximum size
configuration available (`Configuration::with_limit`), but not enabled in the
default configuration. Enabling it causes pre-allocation size to be limited to
prevent against memory exhaustion attacks.

Deserializing any incoming data will not cause undefined behavior or memory
issues, assuming that the deserialization code for the struct is safe itself.

Bincode can be used for untrusted inputs in the sense that it will not create a
security issues in your application, provided the configuration is changed to enable a
maximum size limit. Malicious inputs will fail upon deserialization.

### What is Bincode's MSRV (minimum supported Rust version)?

Bincode 2.0 has an MSRV of 1.85.0. Any changes to the MSRV are considered a breaking change for semver purposes, except when certain features are enabled. Features affecting MSRV are documented in the crate root.

### Why does bincode not respect `#[repr(u8)]`?

Bincode will encode enum variants as a `u32`. If you're worried about storage size, we can recommend enabling `Configuration::with_variable_int_encoding()`. This option is enabled by default with the `standard` configuration. In this case enum variants will almost always be encoded as a `u8`.

Currently we have not found a compelling case to respect `#[repr(...)]`. You're most likely trying to interop with a format that is similar-but-not-quite-bincode. We only support our own protocol ([spec](https://github.com/copper-project/copper-rs/blob/master/core/cu_bincode/docs/spec.md)).

If you really want to use bincode to encode/decode a different protocol, consider implementing `Encode` and `Decode` yourself. `cu_bincode_derive` will output the generated implementation in `target/generated/cu_bincode/<name>_Encode.rs` and `target/generated/cu_bincode/<name>_Decode.rs` which should get you started.

## Native encoding descriptions

`ValueDecode` is always available, including in `no_std` builds. Types deriving
`Encode` can use `#[bincode(describe)]` to generate their description independently
of Cargo features. Enable `self-describing` alongside `derive` to generate
descriptions for all native `Encode` derives. Types containing `with_serde`
fields retain their codec implementations but need a handwritten `ValueDecode`
implementation because Serde controls their wire representation. The companion uses the encoder's
parsed declaration and field attributes, preserving field order, enum tags,
skips, and typed child references. Recipes select scalar widths and container
framing; the reader supplies the same bincode integer settings and endianness
used by the encoder.

`ValueDecode` describes owned payload types. A handwritten encoder declares the
representation it writes, for example:

```rust
use cu_bincode::{ValueDecode, ValueDecodeSpec};
struct Orientation([f32; 4]);
impl ValueDecode for Orientation {
    const DECODE: &'static ValueDecodeSpec = <[f32; 4] as ValueDecode>::DECODE;
}
```

Recipes are static and construct no value tree on the encoding path. Copper's
experimental `cu29_value::decode::ValueDecodeDescription` combines them with
reflection and typed `ValueDecode::METADATA` to interpret native payload bytes offline.
The allocation-free metadata vocabulary, scalar kinds, and record shapes are
shared through `cu29-value-types`. Metadata remains separate from native payload bytes.
Missing nested recipes produce compile errors when companion generation is
enabled. Serde adapter encodings and selective `Uleb128` encodings require
additional supported recipes.
