# Self-describing Copper logs

Target design for `gbin/self-describing-logs-save`, following the prerequisite
[metadata-aware logging and rollover PR](metadata-aware-rollover.md).

Enable the application feature and use its normal builder:

```toml
[features]
self-describing-logs = ["cu29/self-describing-logs"]
```

```rust,ignore
let app = App::builder()
    .with_log_path("logs/app.copper", Some(32 * 1024 * 1024))?
    .build()?;
```

Both generated project templates expose this feature. Payload definitions remain
in the application or their existing dependency crates. The usual build script
continues to call `cu29_build::setup()`.

The runtime macro emits static descriptions for **all compiled missions**, with
one shared payload schema graph and each mission's actual CopperList slot order.
The running application produces and saves its catalog only at startup.
The first application construction serializes these descriptions with bincode,
compresses them with Heatshrink and writes one static catalog section before
resources and runtime streams are initialized. Offline tools load that section
to decode ordinary native payload bytes into value trees.

## Payload authors

`cu-bincode` generates `ValueDecode` alongside an ordinary `Encode` derive when
its `self-describing` feature is enabled. Cargo unifies this feature across codec
users, so an external payload such as `cu_gnss_payloads::GnssFixSolution` participates
through its existing derives. Reusable payload authors can also use
`#[bincode(describe)]` to emit descriptions for individual types independently of
Cargo features. Nested fields and recursive types are followed by
typed references. Users declare the graph's output types once in RON.

```rust,ignore
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::f32::{Length, Velocity};

#[derive(Clone, Debug, Default, Serialize, Deserialize, Encode, Decode, Reflect)]
#[bincode(crate = "cu29::bincode")]
struct WheelSample {
    ticks: u32,
    distance: Length,
    speed: Velocity,
    valid: bool,
}
```

The codec recipe carries field names, original declaration positions, wire order,
scalar widths and enum branches. Static type references supply original native
type names. Copper quantities carry their quantity identity and coherent storage
unit in `ValueDecode::METADATA`; time stores nanoseconds. Catalog construction
uses these native recipes, independently of reflection. The app's task/replay
reflection behavior continues to follow its usual features.

For manually implemented `Encode`, the type author supplies a recipe matching the
wire representation. A wrapper encoding four `f32` components can reuse the
standard array recipe:

```rust,ignore
impl cu29::bincode::ValueDecode for Orientation {
    const DECODE: &'static cu29::bincode::ValueDecodeSpec =
        <[f32; 4] as cu29::bincode::ValueDecode>::DECODE;
}
```

Custom recipes must describe the encoder exactly. Validate their output against
native bytes, including complete consumption and consecutive values. Existing
Copper handles, fixed-capacity arrays, sensor payloads, CRSF and MSP wrappers
supply their recipes. An external handwritten encoder needs support from its
author or an application wrapper. Guaranteed uncaptured slots impose no recipe
bound. Captured custom logging codecs need a native recipe for their encoded
representation; startup reports unsupported task/type/codec combinations.

## Startup memory and feature boundaries

`cu29/self-describing-logs` supports `no_std`. Successful schema serialization and
compression allocate no heap memory: the producer uses a table of 256 reachable
native type references, fixed Heatshrink storage and small output buffers. Count
compressed bytes first, reserve one section, then serialize/compress into it.
The section can span backing files; no whole-catalog buffer is needed. Logger
storage follows the selected backend's allocation policy. A graph exceeding 256
distinct reachable types returns a clear startup error.

All traversal, serialization and compression finish during construction. The
real-time recording path retains its existing native encoding pass.

Host readers enable `cu29/decode-catalog`, which adds `std`, reflection and
value-tree decoding. `cu29-export/self-describing-logs` selects this host feature
automatically.

## Catalog contents and storage

The section body is one bincode value compressed with Heatshrink: standard
bincode configuration (little-endian floats, variable-width integers), Heatshrink
window bits **10**, lookahead bits **5**. Its version is an ordinary bincode field.

| Field | Meaning |
| --- | --- |
| Version | Catalog schema/encoding version |
| CopperList encoding | `Compact`: shared presence/capture planes and delta-coded common metadata; `Flat`: individual message envelopes |
| Shared description | Wire operations, bindings, type/field/variant names and typed storage-unit metadata |
| Missions | Every compiled mission's ID and ordered output slots |
| Output slot | Task/channel identity, configured message type, optional shared schema binding |

Graph references preserve shared and recursive types. Typed metadata uses permanent
kind, quantity and storage IDs; readers preserve unknown metadata. Offline readers
enforce decompression limits, decode one complete catalog and validate references.
The logger's section header supplies type and stored byte length.

## Construction, append and rollover

Application identity, version, Git information and canonical effective RON live
in the separate static `ApplicationMetadata` section. New constructions emit a
small `Instantiated` lifecycle marker. Their sections identify construction,
instance and mission; lifecycle events carry timestamps and transition details.

Matching constructions and append reuse the original static metadata. Comparison
covers application metadata and the complete catalog; mismatches fail before
writing. Start/stop/restart reuses the construction identity. Byte-based rollover
reclaims data sections while retaining static metadata. Readers reach the catalog
through its byte offset and decode surviving sections using their mission map.
See the [logging design](metadata-aware-rollover.md) for lifecycle and log maps.
