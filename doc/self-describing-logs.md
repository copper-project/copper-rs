# Self-describing Copper logs

Status: API prototype implemented, 2026-09-30. Build packaging and recorder/export
integration remain planned.

The first implementation exposes `ValueDecodeDescription::from_type::<T>()` and
`description.decode(bytes, codec_config, limits)` through `self-describing`.
Descriptions can be bincode-serialized and transported independently of payload
types. `Encode` generates static companion recipes in the linked `cu-bincode`
checkout, using its existing parser and attribute handling. Copper supplies
quantity storage registrations, time/string recipes, and fixed-capacity container
recipes. `just self-describing-check` verifies native byte/value agreement.

This PR's owned wire/schema graph is an experimental packaging input. Its bincode
serialization is finalized as a versioned `ValueDecodeCatalog` format in PR 2. Standard
recipes currently cover the API's scalar, aggregate, array, sequence, map, optional,
and transparent representations. Additional handwritten codecs, handles, SoA,
and complete CopperList metadata are integration work. Tuple reflection that omits
declaration positions is rejected until an explicit mapping is supplied.

**Collaboration preference:** keep responses within one page; decide one thing
at a time.

## Goal and process

Read existing compact CopperList bytes without application source or a compiled
application logreader. An embedded binary recipe transforms bytes into
`cu29-value` trees, with accompanying field/type/unit descriptions. V0 requires
complete recipes and decodes sequentially, preserving the current CL byte layout.

```text
Build:  Encode recipes + Reflect schema + standard types + CL layout -> package
Record: copy embedded blob into a ValueDecodeCatalog section; encode CLs normally
Export: load bundle once; execute recipes over CL bytes -> Value trees
```

1. **Generate alongside Encode.** We control `cu-bincode`: extend its derive to
   emit a companion descriptor from the same encoding logic. Pure derived
   encodings compose through field-type descriptors, including generics/dependencies.
   Match attributes/settings exactly. `DECODE` describes the encoded representation;
   `Reflect` supplies the existing type/field/variant descriptions.
2. **Package during the build.** Join recipes with reflection and standard-type
   metadata, resolve descriptor references, deduplicate, bincode-encode, compress,
   and embed the final static blob. Startup only copies
   it. Specify a host packaging helper obtaining instantiated descriptors,
   including cross-compilation.
3. **Interpret offline.** Load the bundle once; execute recipes with a bounded
   cursor to construct CLI/Python values. Completing a CL locates the next one.
   Original native Rust types are unnecessary.

## User contract: describe encoding once

Keep the companion trait named `ValueDecode`: it describes how an offline reader
produces `Value` from the existing `Encode` bytes. The declaration is a wire recipe:

```rust
// Available with self-describing. Re-exported through the Copper prelude.
pub trait ValueDecode: 'static + Sized {
    const DECODE: &'static ValueDecodeSpec;
}
```

The trait and wire IR belong with the controlled codec; they compose independently
of reflection. Copper's packaging helper combines them with reflection at build
time. These inputs have distinct responsibilities:

| Input | Supplies |
| --- | --- |
| `Encode` + generated `DECODE` | Scalar encoding, field wire order, tags, counts, and child recipes |
| `Reflect` | Original type identity, field/variant names, and logical structure |
| Copper standard-type support | Quantity identity, coherent storage unit, and known representation |

Derived `Encode` generates the companion implementation automatically. Users keep
their existing `Reflect` derive. A handwritten encoder supplies only the wire
information the derive cannot see, usually by delegating to a supported type.

Reflection enriches a recipe; it cannot establish byte order, count prefixes,
custom tags, or a handwritten encoder's representation. Each wire operation remains
explicit, generated, or supplied by a standard implementation. A recipe may be
minimal at the declaration site but must be complete after build-time resolution.

For a derived aggregate, the codec derive emits field/variant selectors and typed
child references from the same syntax that generates encoding. Packaging resolves
those selectors against reflection and retains only schema IDs in the bundle.
Resolve by identity, checking names and types, rather than assuming reflected field
indices equal declaration indices: reflection can omit fields. An encoded field
missing from reflection needs an explicit supported schema mapping or a build error
identifying that field.

Every child binding retains its original type identity alongside its wire recipe.
Two types sharing the same scalar recipe can have different schemas. Deduplicate
the wire operation without losing those bindings.

For a manual encoder, `DECODE` defines the exported representation. An opaque
reflected wrapper can therefore export an array while retaining its wrapper type
identity. Exporting a transformed representation as named logical fields requires
an explicit mapping; reflection alone cannot infer that transformation.

### Standard quantities

Treat `cu29_units` types as standard. Copper supplies their scalar recipes and
quantity metadata, keyed by typed registrations during packaging. `Length` stores
metres, `Velocity` stores metres per second, and `f32`/`f64` selects scalar width.
Their constructors normalize input units before storage: constructing a `Length`
in centimetres still records metres.

Use the coherent **storage** unit, distinct from a debugger's preferred display
unit. Generate this metadata alongside the quantity definitions; cover every
supported quantity. For example, mass storage is kilograms even if a display
prefers grams. Recognize quantities by their registered types.

Package the quantity identity and storage unit once per reachable schema. This
makes each file sufficient for an offline reader and lets it display a known unit
even as the standard-type catalogue evolves. Applications supply no `Meaning`
wrapper, unit strings, or custom length type to describe standard quantities.

## Representative examples

These examples use `self-describing`. The companion `Encode` recipe and offline
value API are implemented; build packaging remains planned.

### Ordinary payload: nothing extra to implement

```rust
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::f32::{Length, Velocity};

#[derive(Clone, Debug, Default, Serialize, Deserialize, Encode, Decode, Reflect)]
struct WheelSample {
    ticks: u32,
    distance: Length,
    speed: Velocity,
    valid: bool,
}
```

This is an ordinary Copper payload declaration. `Encode` generates its recipe;
`Reflect` supplies `WheelSample` and its field names; Copper supplies the recipes
and units for `Length` and `Velocity`. Users implement no companion traits and
add no logging-specific annotations.

For a sample with `ticks = 42`, `distance = 1.25 m`, `speed = 0.5 m/s`, and
`valid = true`, export produces a `Value::Map` equivalent to:

```text
{ticks: U32(42), distance: F32(1.25), speed: F32(0.5), valid: Bool(true)}
```

The accompanying schema retains the quantity types and storage units, while the
values retain their scalar widths. Primitive-only structs, nested derives,
supported containers, and enums follow the same automatic path.

### Foreign type with a manual encoding

A local wrapper around `glam::Quat` chooses to encode four `f32` components in
`[x, y, z, w]` order. Reflection treats the wrapper as opaque, so the foreign type
needs no reflection implementation. Enable `glam`'s `serde` feature for the usual
payload serialization derives. Its wire representation is a fixed array:

```rust
use cu29::bincode::{self, Decode, Encode};
use cu29::prelude::*;

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Reflect)]
#[reflect(opaque)]
struct Orientation(glam::Quat);

impl bincode::Encode for Orientation {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.0.to_array().encode(encoder)
    }
}

impl<Context> bincode::Decode<Context> for Orientation {
    fn decode<D: bincode::de::Decoder<Context = Context>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let components = <[f32; 4]>::decode(decoder)?;
        Ok(Self(glam::Quat::from_array(components)))
    }
}

bincode::impl_borrow_decode!(Orientation);

impl ValueDecode for Orientation {
    const DECODE: &'static ValueDecodeSpec = <[f32; 4] as ValueDecode>::DECODE;
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, Encode, Decode, Reflect)]
struct AttitudeSample {
    orientation: Orientation,
    valid: bool,
}
```

The only self-description code the wrapper author writes is the one-line recipe
delegation: **decode exactly as `[f32; 4]`**. Its fixed-array recipe consumes 16
bytes with the selected float endianness and no length prefix, producing
`Value::Seq([F32(x), F32(y), F32(z), F32(w)])`. Packaging attaches `Orientation`'s
reflected type identity to that representation; the containing derive supplies
the `orientation` field name. Component positions follow the declared encoding.

The native `Decode` implementation reconstructs a quaternion for normal Copper
use/replay. The offline reader only executes the array recipe; the logged bundle
contains data describing that recipe. More complex custom encodings compose
supported wire operations and explicit mappings in the same way. Validate a manual
recipe against its encoder, including exact byte consumption and consecutive values.

## V0 constraints and compile-time checks

Every potentially recorded slot must have a complete recipe, recursively covering
all fields/enum branches. The controlled derive emits a companion trait
(proposed `ValueDecode`); runtime codegen requires it for each captured payload
and the actual selected codec. A guaranteed uncaptured payload needs no recipe.
Missing support fails compilation with the task, output, and offending type/field.

Supply Copper recipes for units/time, containers/handles, SoA, and standard manual
encoders. Supported ordinary derives require no additional user annotations.
Packaging checks reflected bindings and standard-type metadata for every reachable
type. Opaque reflection is sufficient when the recipe supplies the export shape.
Other manual encoders/codecs need a supported explicit recipe or must be replaced
or excluded from payload logging. Keep these bounds local to `self-describing`.
Trait checks establish availability; arbitrary handwritten encoder/recipe agreement
requires validation. Recipes generated alongside encoders share the encoding logic.

## Recipe shape

Use a bincode-encoded graph with separate wire recipes and schema bindings. For
`WheelSample` (schematic IDs; codec parameters are recorded separately):

```text
wire:
  w0: Scalar(U32, selected_integer_encoding)
  w1: Scalar(F32, selected_endianness)
  w2: Scalar(Bool)
  w3: Record(fields=[w0, w1, w1, w2])
schema:
  s0: Primitive(U32)
  s1: Quantity(type=cu29_units::si::f32::Length, storage_unit=m)
  s2: Quantity(type=cu29_units::si::f32::Velocity, storage_unit=m/s)
  s3: Primitive(Bool)
  s4: Struct(type=WheelSample,
             fields=[(ticks, s0), (distance, s1), (speed, s2), (valid, s3)])
binding:
  root: (wire=w3, schema=s4)
  fields: [(w0, s0), (w1, s1), (w1, s2), (w2, s3)]
```

`Length` and `Velocity` reuse one float operation while keeping separate schemas.
Other nodes cover tuples, fixed repeats, length-prefixed sequences/bytes, maps,
and tagged branches. Local length bindings support SoA columns sharing one count.
Bindings associate wire positions with exported names and schema references.
Fields follow wire order; schema metadata consumes no payload bytes. Preserve
numeric widths and distinguish wire rules from Rust memory/reflection layout.

## Compact bundle and compatibility

Store each reachable recipe/schema/string once, using compact tags, varint IDs, and
shared references. Never unroll arrays or duplicate schemas per slot/message.
Benchmark compression on realistic apps; select by compressed size and exporter
cost. Measure raw/compressed bundle bytes and startup copy time.

A new `ValueDecodeCatalog` section has a small fixed bootstrap header: recipe version,
compression ID, compressed/uncompressed lengths, and bundle identity. Its compressed
body contains the bincode-encoded recipe/schema graphs, their bindings, string
table, slot wiring, and codec parameters. Fix the recipe's bincode settings/tags
as a versioned wire contract.
Bind bundles to runs/missions and reuse identical blobs; rollover and archive
exports must retain the applicable bundles.

Cover the whole CL encoding: ID, presence/capture planes, timestamp deltas, and
metadata references in `copperlist_codec.rs`. Embed recipes or versioned operations
for these rules rather than assuming the exporter's current metadata decoder.

## Recording constraints

`self-describing` enables `std` and real `reflect` support for build-time schema
extraction. Startup copies the prepared bundle; recording keeps the existing single
encoding pass. V0 adds no CL offsets, lengths, patching,
or recipe execution on the recording path. Decode presence/capture metadata first,
then each captured payload in slot order. Variable-length values must carry counts
or tags understood by their recipes. Unsupported encodings are compile errors;
invalid/truncated bytes stop decoding rather than attempting opaque-slot recovery.
Value-tree allocation happens only in the exporter.

## Implementation sequence

1. Specify the wire IR, reflection bindings, standard quantity metadata, and
   compile-time bounds; prove primitives, derived structs, quantities,
   collections/enums, and supplied manual
   encoders agree with native decoding. Verify missing nested/codec recipes fail
   compilation with useful diagnostics.
2. Prototype build packaging/compression on a realistic app; measure the complete
   bundle and settle the compact format before recorder integration.
3. Add section discovery and sequential CLI/Python export, including common CL
   metadata, capture policies, appended runs, and bundle retention. Verify multiple
   consecutive CLs decode without additional framing.
4. Bound decompression, recursion, collections, execution work, and output size;
   test truncation/invalid recipes, encoding attributes, and
   compatibility fixtures across producer encoding changes.
