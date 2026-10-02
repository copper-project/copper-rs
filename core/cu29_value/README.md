# copper-value

[![license-badge][]][license]

`copper-value` provides a way to capture serialization value trees for later processing.
Customizations are made to enable a more compact representation for the structured logging of copper.

## Native payload descriptions

Enable `cu29/self-describing-logs` to describe native `Encode` bytes and decode them
into a `Value` tree offline. The feature enables `std` and reflection. `Encode`
derives supply the `ValueDecode` companion automatically; payload authors keep
their usual `Reflect` derive.

```rust
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;

#[derive(Clone, Debug, Default, Serialize, Deserialize, Encode, Decode, Reflect)]
struct Sample {
    ticks: u32,
    valid: bool,
}

let description = ValueDecodeDescription::from_type::<Sample>()?;
let config = cu29::bincode::config::standard();
let mut bytes = [0; 32];
let len = cu29::bincode::encode_into_slice(
    Sample { ticks: 42, valid: true }, &mut bytes, config,
)?;
let (tree, consumed) = description.decode(
    &bytes[..len], config, ValueDecodeLimits::default(),
)?;
```

A description implements native bincode `Encode` and `Decode`. After transporting
it as bytes, an offline reader uses only the description, payload bytes, and the
producer's codec configuration. Decoding returns the exact number of consumed
bytes, allowing sequential values to be read from one buffer.

Descriptions preserve field names, original type identities, scalar widths, and
quantity storage units. Each type declares a static, typed `ValueDecode::METADATA`
slice using Copper's metadata vocabulary, shared through `cu29-value-types`.
All supported quantities retain their coherent storage unit in both scalar widths:
length is `m`, velocity is `m s^-1`, and mass is `kg`. Copper clock values retain
nanosecond storage. Unit symbols are presentation output from typed storage units.

A handwritten type can attach existing Copper metadata:

```rust
use cu29_value::{QuantityMetadata, TimeStorageUnit, ValueMetadata};
use bincode::{ValueDecode, ValueDecodeSpec};

struct Elapsed(u64);
impl ValueDecode for Elapsed {
    const DECODE: &'static ValueDecodeSpec = <u64 as ValueDecode>::DECODE;
    const METADATA: &'static [ValueMetadata] = &[
        ValueMetadata::Quantity(
            QuantityMetadata::time(TimeStorageUnit::Nanosecond),
        ),
    ];
}
```

Portable schemas contain `metadata` entries. `schema.quantity()` returns recognized
quantity metadata; `entry.known()` returns recognized typed metadata. Entries use
permanent numeric kind IDs and length-delimited bodies. Quantity kind `1` contains
exactly eight bytes: a little-endian `u32` quantity ID followed by a little-endian
`u32` storage alternative (`1` for coherent storage, `2` for nanoseconds).
The outer kind and body length use the description's bincode configuration.
Copper assigns new IDs when its vocabulary grows and retains existing meanings.

An older reader preserves unknown kinds, quantities, and storage alternatives
verbatim. It exposes the decoded payload value and leaves unsupported units
uninterpreted. Invalid known combinations and malformed or truncated entries
return a decoding error. Metadata framing supports evolving metadata alongside
wire operations the reader already understands.

For a handwritten encoder, implement `ValueDecode` by delegating to the type
actually written. An opaque reflected wrapper encoding `[f32; 4]` declares:

```rust,ignore
impl ValueDecode for Orientation {
    const DECODE: &'static ValueDecodeSpec = <[f32; 4] as ValueDecode>::DECODE;
}
```

Structurally reflected encoded fields must be present in reflection with the same
native type. Tuple fields require reflection to retain every declaration position.
Hidden encoded fields and ambiguous tuple mappings return a description-building
error. For opaque reflection, the static encoding recipe supplies the encoded
field names, declaration positions, and enum variants.
Skipped fields are excluded from the wire description. Missing nested descriptions and
custom codec descriptions fail compilation with `self-describing-logs` enabled.

Description construction and value decoding allocate in offline tooling. Native
message encoding keeps its existing byte layout and encoding pass. The portable
IR is experimental. Enable `cu29-value/decode-catalog` for the versioned compressed
catalog reader and `cu29-build/self-describing-logs` for host packaging. The
application builder's `with_value_decode_catalog(...)` records the embedded static
blob during construction. See `examples/cu_self_describing_logs` and
`doc/self-describing-logs.md` for the host workflow and catalog format.
Run `just self-describing-logs-check` at the Copper workspace root to verify this API.

## Python Feature

With the `python` feature enabled, this crate also provides conversion helpers between
`cu29_value::Value` and Python objects via PyO3.

That bridge is used in two places:

- `cu-python-task`, where Copper temporarily converts task inputs/state/outputs so a
  Python function can mutate them
- `cu29-export`, where Copper data is exposed to Python for offline analysis

Conversion behavior is intentionally simple:

- `None` maps to `Value::Unit`
- Python lists and tuples map to `Value::Seq`
- Python dicts map to `Value::Map`
- Python integers are accepted up to 128-bit signed/unsigned range
- Python `bytes` maps to `Value::Bytes`

[license-badge]: https://img.shields.io/badge/license-MIT-lightgray.svg?style=flat-square
[license]: https://github.com/arcnmx/serde-value/blob/master/COPYING
