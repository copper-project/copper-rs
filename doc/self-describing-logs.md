# Self-describing Copper logs

Status: payload API, build packaging and startup recording implemented, 2026-10-02.
Standalone CopperList metadata decoding and CLI/Python value export remain planned.

`self-describing-logs` exposes `ValueDecodeDescription::from_type::<T>()` and
`description.decode(bytes, codec_config, limits)`. `Encode` generates static
companion descriptions in `cu-bincode`; Copper binds these to reflection,
quantity storage registrations, time/string descriptions and supported containers.

`ValueDecodeCatalog` packages a shared payload-description graph with generated
slot wiring, mission, canonical RON config and CopperList layout. A host build
script bincode-serializes it, compresses it with Brotli quality 11, and emits a
Rust static byte array. The application builder copies that prepared blob verbatim
into a dedicated unified-log section during construction. Recording retains its
existing encoding pass. `just self-describing-logs-check` verifies native decoding,
host packaging, startup recording, appended-run retention and feature isolation.

The catalog format is versioned independently of unified-log encapsulation.
Additional handwritten logging codecs, handles, SoA and standalone interpretation
of complete CopperList metadata are integration work. Tuple reflection that omits
declaration positions is rejected until an explicit mapping is supplied.

**Collaboration preference:** keep responses within one page; decide one thing
at a time.

## Goal and process

Read existing compact CopperList bytes without application source or a compiled
application logreader. An embedded binary description transforms bytes into
`cu29-value` trees, with accompanying field/type/unit descriptions. V0 requires
complete descriptions and decodes sequentially, preserving the current CL byte layout.

```text
Build:  Encode descriptions + Reflect schema + standard types + CL layout -> package
Record: copy embedded blob into a ValueDecodeCatalog section; encode CLs normally
Export: load bundle once; execute descriptions over CL bytes -> Value trees
```

1. **Generate alongside Encode.** We control `cu-bincode`: extend its derive to
   emit a companion descriptor from the same encoding logic. Pure derived
   encodings compose through field-type descriptors, including generics/dependencies.
   Match attributes/settings exactly. `DECODE` describes the encoded representation;
   `Reflect` supplies the existing type/field/variant descriptions.
2. **Package during the build.** Join descriptions with reflection and standard-type
   metadata, resolve descriptor references, deduplicate, bincode-encode, compress,
   and embed the final static blob. Startup only copies
   it. Specify a host packaging helper obtaining instantiated descriptors,
   including cross-compilation. See the implemented host workflow below.
3. **Interpret offline.** Load the bundle once; execute descriptions with a bounded
   cursor to construct CLI/Python values. Completing a CL locates the next one.
   Original native Rust types are unnecessary.

## User contract: describe encoding once

Keep the companion trait named `ValueDecode`: it describes how an offline reader
produces `Value` from the existing `Encode` bytes. The declaration is a wire description:

```rust
// Available with self-describing-logs. Re-exported through the Copper prelude.
pub trait ValueDecode: 'static + Sized {
    const DECODE: &'static ValueDecodeSpec;
}
```

The trait and wire IR belong with the controlled codec; they compose independently
of reflection. Copper's packaging helper combines them with reflection at build
time. These inputs have distinct responsibilities:

| Input | Supplies |
| --- | --- |
| `Encode` + generated `DECODE` | Scalar encoding, field wire order, tags, counts, and child descriptions |
| `Reflect` | Original type identity, field/variant names, and logical structure |
| Copper standard-type support | Quantity identity, coherent storage unit, and known representation |

Derived `Encode` generates the companion implementation automatically. Users keep
their existing `Reflect` derive. A handwritten encoder supplies only the wire
information the derive cannot see, usually by delegating to a supported type.

Reflection enriches a description; it cannot establish byte order, count prefixes,
custom tags, or a handwritten encoder's representation. Each wire operation remains
explicit, generated, or supplied by a standard implementation. A description may be
minimal at the declaration site but must be complete after build-time resolution.

For a derived aggregate, the codec derive emits field/variant selectors and typed
child references from the same syntax that generates encoding. Packaging resolves
those selectors against reflection and retains only schema IDs in the bundle.
Resolve by identity, checking names and types, rather than assuming reflected field
indices equal declaration indices: reflection can omit fields. An encoded field
missing from reflection needs an explicit supported schema mapping or a build error
identifying that field.

Every child binding retains its original type identity alongside its wire description.
Two types sharing the same scalar description can have different schemas. Deduplicate
the wire operation without losing those bindings.

For a manual encoder, `DECODE` defines the exported representation. An opaque
reflected wrapper can therefore export an array while retaining its wrapper type
identity. Exporting a transformed representation as named logical fields requires
an explicit mapping; reflection alone cannot infer that transformation.

### Standard quantities

Treat `cu29_units` types as standard. Copper supplies their scalar descriptions and
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

These examples use `self-describing-logs`. The companion `Encode` description, offline value API, host packaging and startup
recording are implemented.

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

This is an ordinary Copper payload declaration. `Encode` generates its description;
`Reflect` supplies `WheelSample` and its field names; Copper supplies the descriptions
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

The only self-description code the wrapper author writes is the one-line description
delegation: **decode exactly as `[f32; 4]`**. Its fixed-array description consumes 16
bytes with the selected float endianness and no length prefix, producing
`Value::Seq([F32(x), F32(y), F32(z), F32(w)])`. Packaging attaches `Orientation`'s
reflected type identity to that representation; the containing derive supplies
the `orientation` field name. Component positions follow the declared encoding.

The native `Decode` implementation reconstructs a quaternion for normal Copper
use/replay. The offline reader only executes the array description; the logged bundle
contains data describing that description. More complex custom encodings compose
supported wire operations and explicit mappings in the same way. Validate a manual
description against its encoder, including exact byte consumption and consecutive values.

## V0 constraints and compile-time checks

Every potentially recorded slot must have a complete description, recursively covering
all fields/enum branches. The controlled derive emits a companion trait
(proposed `ValueDecode`); runtime codegen requires it for each captured payload
and the actual selected codec. A guaranteed uncaptured payload needs no description.
Missing support fails compilation with the task, output, and offending type/field.

Supply Copper descriptions for units/time, containers/handles, SoA, and standard manual
encoders. Supported ordinary derives require no additional user annotations.
Packaging checks reflected bindings and standard-type metadata for every reachable
type. Opaque reflection is sufficient when the description supplies the export shape.
Other manual encoders/codecs need a supported explicit description or must be replaced
or excluded from payload logging. Keep these bounds local to `self-describing-logs`.
Trait checks establish availability; arbitrary handwritten encoder/description agreement
requires validation. Descriptions generated alongside encoders share the encoding logic.

## Description shape

Use a bincode-encoded graph with separate wire descriptions and schema bindings. For
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

## Host packaging and startup recording

Keep payload definitions in a shared crate compiled for both the host build script
and the application target. This preserves reflected type paths and lets the host
instantiate `ValueDecode`/`Reflect` even during cross-compilation. Payload features
and conditional representations must agree between the two builds.

The consuming build script enables `self-describing-logs` on `cu29` and `cu29-build`:

```rust,ignore
cu29::prelude::gen_cumsgs!("copperconfig.ron");

fn main() {
    cu29_build::setup();
    println!("cargo::rerun-if-changed=copperconfig.ron");
    let catalog = cumsgs::value_decode_catalog().expect("describe captured slots");
    cu29_build::catalog::write_value_decode_catalog("catalog.rs", &catalog)
        .expect("package catalog");
}
```

The generated function follows the actual flattened CopperList slot order. It
registers captured payloads, resolves one shared graph, and retains uncaptured
positions without requiring descriptions for those payloads. Repeated payload
types reuse bindings, schemas and wire operations. Configured custom logging
codecs require a description of their actual encoded representation; packaging
reports the task, output and codec when that support is needed.

The application embeds the generated source and supplies the static blob:

```rust,ignore
include!(concat!(env!("OUT_DIR"), "/catalog.rs"));

let app = App::builder()
    .with_value_decode_catalog(VALUE_DECODE_CATALOG)
    .with_log_path("logs/app.copper", Some(32 * 1024 * 1024))?
    .build()?;
```

Construction checks the bootstrap magic/version and copies the complete blob once.
The catalog has its own `UnifiedLogType::ValueDecodeCatalog` section, closed before
runtime streams are created. Appended runs retain their catalog in their physical
section range. Rollover retention and standalone CL export remain integration work.
The executable example is `examples/cu_self_describing_logs`; run `just` there.

## Compact bundle and compatibility

The catalog bootstrap header is **10 bytes**:

| Offset | Encoding | Contents |
| --- | --- | --- |
| 0 | 8 bytes | Magic `CUVDCAT\0` |
| 8 | little-endian `u16` | Catalog version, currently `1` |

Version 1 fixes the body to native bincode's standard configuration
(little-endian floats, variable-width integers), compressed with Brotli.
Compression uses quality 11 and a 24-bit window on the host. A change to the
compression format, body layout, bincode settings or encoded enum tags requires a
new catalog version. These settings also describe the current native payload codec.

The decompressed body encodes `mission`, `config_ron`, `layout`, the shared
`ValueDecodeDescription`, and `slots`, in that order. A description encodes `root`,
`bindings`, `operations`, and `schemas`. `Compact`/`Flat` and operation/scalar/shape
enums use their declared variant order as bincode tags; indices use standard
bincode variable integers. Graph references deduplicate reachable types and wire
operations without expanding arrays. Names and unit strings remain in the graph
and are compressed with the rest of the catalog.

Each dedicated section holds one complete catalog. The unified section header's
`used` field supplies its exact compressed envelope length; allocated capacity can
include padding. Static arrays carry their length in Rust. The catalog version fixes its encoding
and compression format. Interrupted/truncated catalog bodies fail offline
decompression or bincode decoding. The reader rejects
trailing input and limits both the compressed body and decompressed data to 16 MiB.

Compression is chosen by size on the wheel example, comparing Brotli quality 11,
Zstd level 22, and XZ preset 9 extreme. Keep the measured results in the example
README. Native message encoding and byte layout are unchanged.

Cover the whole CL encoding in the next integration step: ID, presence/capture
planes, timestamp deltas and metadata references in `copperlist_codec.rs`. Embed
complete descriptions or versioned operations for these rules rather than assuming
the exporter's current metadata decoder.

## Recording constraints

`self-describing-logs` enables `std` and real `reflect` support for build-time schema
extraction. Startup copies the prepared bundle; recording keeps the existing single
encoding pass. V0 adds no CL offsets, lengths, patching,
or description execution on the recording path. Decode presence/capture metadata first,
then each captured payload in slot order. Variable-length values must carry counts
or tags understood by their descriptions. Unsupported encodings are compile errors;
invalid/truncated bytes stop decoding rather than attempting opaque-slot recovery.
Value-tree allocation happens only in the exporter.

## Implementation sequence

1. Specify the wire IR, reflection bindings, standard quantity metadata, and
   compile-time bounds; prove primitives, derived structs, quantities,
   collections/enums, and supplied manual
   encoders agree with native decoding. Verify missing nested/codec descriptions fail
   compilation with useful diagnostics.
2. Build packaging/compression and startup recording are implemented on the wheel
   example. The catalog V1 format records the payload graphs and slot wiring.
3. Add section discovery and sequential CLI/Python export, including common CL
   metadata, capture policies, appended runs, and bundle retention. Verify multiple
   consecutive CLs decode without additional framing.
4. Bound decompression, recursion, collections, execution work, and output size;
   test truncation/invalid descriptions, encoding attributes, and
   compatibility fixtures across producer encoding changes.
