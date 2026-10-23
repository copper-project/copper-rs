# Self-describing logs

Implemented: generated builders write one startup catalog describing native
CopperList payload bytes for **all compiled missions**. Offline tools decode those
bytes into `Value` trees without application type registration. The runtime retains
its ordinary native encoding pass; schema traversal and compression happen during
construction. This builds on [static metadata and rollover](log-storage.md).

## Native recipes, not reflection

`cu-bincode` emits `ValueDecode` alongside `Encode` when its `self-describing`
feature is enabled. Cargo feature unification lets derived external payloads
participate without moving definitions into the app. `#[bincode(describe)]` opts
individual types in independently of Cargo features. The generated recipe describes
field names, declaration positions, wire order, scalar widths, enum branches, and
typed references to nested/shared/recursive types.

Recipes follow **actual encoding**, independently of task/replay reflection.
`ValueDecode::METADATA` adds Copper quantity/storage-unit identity; time uses
nanoseconds. Original native names and typed metadata survive into the catalog.
This avoids replacing payload encoding with a value-tree serialization on each cycle.

A handwritten `Encode` implementation needs an exact matching recipe. A wrapper
encoding four `f32`s can reuse `<[f32; 4] as ValueDecode>::DECODE`; native-byte parity,
complete consumption, and consecutive values must agree. Existing Copper handles,
fixed arrays, quantities, sensor payloads, and supported protocol wrappers supply
recipes. Unsupported handwritten encoders need author support or an application
wrapper. Guaranteed uncaptured slots impose no recipe bound. Captured custom codecs
need a recipe for their encoded representation; unsupported combinations report
task/type/codec context at startup.

## Feature and construction boundaries

| Feature/API | Responsibility |
| --- | --- |
| `cu29/self-describing-logs` | Generate and save startup descriptions; supports `no_std` |
| `cu29/decode-catalog` | Host-side schema/value decoding; enables `std` and reflection |
| `cu29-export/self-describing-logs` | Enable the host reader path |
| `cu29_build::setup()` | Existing application build setup, unchanged |

Both generated project templates expose `self-describing-logs`. The normal builder
and logging setup suffice; no separate registration step is needed.

The macro emits mission slot descriptions in the actual CopperList output order.
All missions share one graph of reachable native types. Before resources and runtime
streams initialize, the first construction counts compressed bytes, reserves one
static section, and serializes/compresses into it. Subsequent constructions and
append compare/reuse that catalog with the application metadata.

The successful producer path uses a fixed table of **256 distinct reachable types**,
fixed Heatshrink storage, and small buffers; schema serialization/compression needs
no heap or whole-catalog buffer. Exceeding the type bound is a startup error. The
storage backend retains its own allocation policy. Static sections may span files.

## Catalog contract

| Field | Meaning |
| --- | --- |
| Catalog version | Schema/encoding version, currently 1; independent of container version |
| CopperList layout | `Compact`: shared presence/capture planes and delta metadata; `Flat`: separate message envelopes |
| Shared description | Wire operations, type bindings, names, field/variant structure, typed storage units |
| Mission maps | Ordered output slots per compiled mission; mission names are in `ApplicationMetadata` |
| Slot | Task/channel ID, configured message type, optional shared schema binding |

The body is one standard-bincode value, compressed with Heatshrink window bits
**10**, lookahead bits **5**. The compressed stream ends with uncompressed byte count
and IEEE CRC32, each little-endian `u32`. This checksum protects catalog decoding;
it is not section-chain crash recovery. Readers cap uncompressed catalogs at
**16 MiB**, validate references and full consumption, and preserve unknown typed
metadata IDs. Graph references represent sharing/recursion without duplicating schemas.

Application identity, canonical RON, and mission names live in `ApplicationMetadata`,
which points to the catalog. Data sections carry run/instance/mission identity;
readers select the matching mission slot map even if startup lifecycle markers
rolled out. Both metadata sections stay outside the data ring. Append rejects
mismatched application metadata or catalog before writes.

## Reader boundaries and evidence

Value decoding reconstructs recorded data, not arbitrary executable task behavior.
Use a matching typed application for deterministic resimulation. Missing handle
contents or deliberately uncaptured data cannot be recovered from a schema.

| Concern | Pointer |
| --- | --- |
| Native recipe vocabulary | [`cu_bincode/src/value_decode.rs`](../../core/cu_bincode/src/value_decode.rs) |
| Derive generation | [`cu_bincode_derive`](../../core/cu_bincode_derive) |
| Generated mission/catalog binding | [`runtime_macro.rs`](../../core/cu29_derive/src/runtime_macro.rs) |
| Bounded startup writer | [`catalog_stream.rs`](../../core/cu29_value/src/catalog_stream.rs) |
| Catalog version/layout and host loading | [`catalog_format.rs`](../../core/cu29_value/src/catalog_format.rs), [`catalog.rs`](../../core/cu29_value/src/catalog.rs) |
| Wire validation and value decoder | [`decode.rs`](../../core/cu29_value/src/decode.rs), [decoder tests](../../core/cu29_value/src/decode_tests.rs) |
| Run-scoped export | [`cu29_export/src/catalog.rs`](../../core/cu29_export/src/catalog.rs), [CLI](../../core/cu29_export/src/catalog_cli.rs), [Python](../../core/cu29_export/src/catalog_python.rs) |
| No-heap writer/parity coverage | [`tests/catalog_stream.rs`](../../core/cu29_value/tests/catalog_stream.rs) |
| Append, multi-mission catalog reuse, cropped startup, corruption/size errors | [`catalog_tests.rs`](../../core/cu29_export/src/catalog_tests.rs), [CLI integration tests](../../core/cu29_export/tests/catalog_cli.rs) |
| End-to-end payload usage | [self-describing example](../../examples/cu_self_describing_logs/README.md) |
