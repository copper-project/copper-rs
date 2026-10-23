# Log storage, metadata, and run identity

Copper uses one byte-addressed log format for mmap files and SD/eMMC partitions.
Static application metadata precedes a rotating section chain. The current
encapsulation is **format v3**; the [concurrency note](concurrent-logging.md)
describes its separate descriptor and commit blocks.

## Layout

```text
MainHeader | ApplicationMetadata | optional ValueDecodeCatalog | data region
                                                              head → … → tail
```

All persisted offsets are `u64` bytes from the log origin (the partition start on
SD). Zero means absent. `MainHeader` stores alignment, metadata offset, data bounds
`[sections_begin, sections_end)`, retained head/tail, and clean-close state.
Section sizes include headers and padding; `used` counts committed payload bytes.
Readers follow links across wrap rather than infer chronology from file order.
Slabs are backing allocations; a section may cross files.

Each section reserves 1024 bytes: a 512-byte allocator descriptor, a 512-byte encoder
commit block, then native encoded entries. `SectionHeader` is a decoded view of
those blocks, not their disk encoding. Format versions cover encapsulation only;
CopperList, keyframe, payload, and codec compatibility requires a matching
application decoder or a supported self-description catalog.

## Static metadata and construction identity

| Stored once, outside the ring | Contents / reason |
| --- | --- |
| `ApplicationMetadata` | Application type/name/version, optional Git identity/dirty state, subsystem identity/code, canonical effective RON, ordered compiled mission names, catalog offset |
| `ValueDecodeCatalog` (optional) | Shared payload schema graph and ordered output-slot maps for all compiled missions; surviving ring data stays decodable |

Generated builders seal metadata **before resources and runtime streams**. The
first construction writes it; subsequent constructions compare it. Append compares
both metadata bodies before activating writes. A mismatch rejects sharing a log
between different applications, versions, configurations, or catalogs. Mission
indices use the same deterministic ordering in metadata, catalog maps, and sections.

Every data section carries `SectionContext { run_id, instance_id, mission_index }`.
A successful reconstruction gets a new run ID even if clocks or CopperList IDs
restart. Stop/start of the same construction keeps its ID. On append, the next ID
is reconstructed above the highest retained run ID; no persistent next-ID counter
exists. Standalone streams use run ID zero and seal an empty metadata region when
data starts. A section never mixes construction contexts.

This identity is in each section because a ring can discard startup markers while
retaining payloads. Run discovery groups surviving sections, resolves their mission
through static metadata, and leaves missing startup times unknown.

## Lifecycle and output ordering

| Transition | Recorded event / meaning |
| --- | --- |
| Successful construction | `Instantiated`, with timestamp and configuration source |
| Successful start | `MissionStarted`, after startup hooks |
| Successful stop | Drain output, then `MissionStopped { reason }` |
| Failed lifecycle operation | `LifecycleFailed { operation, error }`; successful transition markers are omitted |
| Iteration failure | Failure record; caller determines continuation or stop |
| Panic | Best-effort `Panic` with location/message |
| Successful teardown | Output flushes precede `ShutdownCompleted` |

Stop reasons are `Requested`, `Completed`, `Error`, and `Panic`. A stop is not the
end of a construction: restarting emits another start marker with the same context.
Logger close is separate from app teardown. Missing markers after interruption are
not synthesized. Managed [clock corrections](clock-sync.md) also use the lifecycle
stream, associated with the first CopperList using the new mapping.

CopperList, keyframe, and lifecycle streams belong to their construction. The
process-global structured-text logger routes to the most recently installed sink;
it does not switch context on every iteration. That limitation avoids adding a
context switch to the real-time path.

## Capacity and reclamation

Capacity policy is chosen at construction/append rather than persisted as a mode.

| Policy | Allocation behavior |
| --- | --- |
| `Grow` | Allocate beyond occupied extents; extend storage as needed. SD growth is bounded by its partition. |
| `OverwriteOldest` | Wrap to the data-region start, reclaim oldest sections through the last overlap, preserve static metadata. |
| `StopWhenFull` | Return a capacity error instead of reclaiming or extending. |

Append can change policy and preserve remaining history; it cannot restore discarded
data or shrink storage beneath retained sections. Reopen reconstructs allocation
geometry once. `Grow` uses the known allocation end without walking history.
The current non-`Grow` allocation path still scans retained extents to find overlaps;
oldest-only bounded scans remain an optimization opportunity.

Reclamation first claims affected writers. An active encoder blocks reuse with an
explicit error before destructive mutation. Idle partial sections can be sealed,
so sparse streams do not pin the ring indefinitely; their next append rebinds.
The allocator publishes removal before reusing bytes with a new generation.
There are 64 preallocated concurrent section leases. One mmap section can span at
most 64 backing files; size slabs accordingly.

Entries encode directly into the backend. An oversized entry or failed encoding
leaves committed length and cursor unchanged; it can leave bytes beyond the
committed prefix. SD uses fixed block buffers and discards pending failed-write
state. Native per-cycle encoding gains no catalog pass or intermediate payload copy.

## Close, inspection, and compatibility

Append requires a clean main header, closed retained sections, valid bounds/links,
and matching static metadata. Validation and comparison precede writes. Inspection
can read reachable committed data from an incomplete log, but invalid metadata
terminates traversal or reports an error. Logical commit is not a power-loss
atomicity guarantee; automatic repair/resume is not implemented.

Append-only handles must be flushed or dropped before explicit logger close.
Drop seals best-effort; storage failures poison clean close. Explicit close reports
errors. Format v3 readers reject incompatible encapsulation; the earlier v2 plan's
single 512-byte section header is superseded.

## Code and coverage

| Concern | Implementation / evidence |
| --- | --- |
| Public header/context/metadata types | [unifiedlog `lib.rs`](../../core/cu29_unifiedlog/src/lib.rs) |
| Layout, append comparison, allocation, reclamation | [`byte_log.rs`](../../core/cu29_unifiedlog/src/byte_log.rs): `ByteLogger`, `read_header`, `read_main` |
| Storage adapters | [mmap](../../core/cu29_unifiedlog/src/memmap.rs), [SD/eMMC](../../components/libs/cu_sdlogger/src/logger.rs) |
| Builder metadata and lifecycle wiring | [`runtime_macro.rs`](../../core/cu29_derive/src/runtime_macro.rs) |
| Lifecycle types and completed output | [`curuntime.rs`](../../core/cu29_runtime/src/curuntime.rs) |
| Cropped/interleaved run discovery | [`runs.rs`](../../core/cu29_export/src/runs.rs) |
| Wrap, sparse rebind, policy changes, mismatch without writes, incomplete logs, encode rollback, cross-file sections | [byte-log integration tests](../../core/cu29_unifiedlog/tests/byte_log.rs) |
| Block-device format and rollover | [SD logger tests](../../components/libs/cu_sdlogger/src/logger.rs) |
| Catalog retained after startup rolls out | [catalog tests](../../core/cu29_export/src/catalog_tests.rs): `cropped_startup_uses_retained_mission_identity_and_shared_catalog` |
