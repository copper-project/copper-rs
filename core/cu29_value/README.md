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
quantity storage units. All supported `cu29-units` quantities have typed storage
registrations in both scalar widths. Coherent units use SI base-unit expressions:
length is `m`, velocity is `m s^-1`, and mass is `kg`. Copper clock values
retain their nanosecond (`ns`) storage unit.

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
Skipped fields are excluded from the wire recipe. Missing nested recipes and
custom codec recipes fail compilation with `self-describing-logs` enabled.

Description construction and value decoding allocate in offline tooling. Native
message encoding keeps its existing byte layout and encoding pass. The portable
IR is experimental; compression, build embedding, and unified-log catalogue
integration follow in later PRs. Run `just self-describing-logs-check` at the Copper
workspace root to verify this API.

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
