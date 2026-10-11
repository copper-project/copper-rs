# Metadata-aware logging and rollover

Design for a prerequisite PR before `gbin/self-describing-logs-save`. Continue
iterating on **unreleased format v2**. Implement the same format and section
rollover for mmap and SD/eMMC; slabs are backing allocations.

## One byte-addressed log

All persisted offsets are `u64` bytes from the beginning of the log. On SD this
origin is the partition start. The storage adapter translates bytes to mapped
files or device blocks. A section can cross backing-file boundaries.

```text
0
| MainHeader |
metadata_offset
| ApplicationMetadata section | ValueDecodeCatalog section (when enabled) |
sections_begin
| lifecycle | CopperLists | frozen tasks | structured logs | ... | free |
sections_end
```

| Header | Required information |
| --- | --- |
| Main | Format v2, allocation alignment, `metadata_offset`, `sections_begin`, `sections_end`, `head_section`, `tail_section`, clean-close state |
| Section | Type, allocated byte length, committed payload bytes (`used`), next section byte offset, open/closed state, `run_id: u64`, `instance_id: u32`, `mission_index: u32` |

`sections_begin` is the first byte available for rotating sections, after static
metadata; `sections_end` is the exclusive end of that space. `head_section` points
to the oldest retained section; `tail_section` points to the newest allocated
section. Derive the next allocation position from the tail's offset and allocated
size. A zero head/tail means an empty log. The main clean-close flag records writer
completion; the tail's zero next link identifies the end of the section chain.

Offsets identify section headers. Section sizes include alignment and headers;
`used` counts payload only. Zero represents an absent offset. The metadata region
ends at `sections_begin`. Section traversal follows byte links across wrap;
backing-file order never defines chronology.

Keep the existing **512-byte section-header reservation**. Run, instance and mission
fields add at most 19 bincode bytes inside it; payload offsets stay unchanged.
Mission names live in an ordered static table. The macro supplies the numeric
index at compile time; readers index that table. Indices 0–250 encode in one byte
with standard bincode, including as `u32`. Standalone sections use `run_id = 0`.

## Static metadata: write once, compare on append

| Section | Body |
| --- | --- |
| `ApplicationMetadata` | Application type/package identity, app version, optional Git commit/dirty state, subsystem identity/code, canonical effective RON configuration, ordered compiled mission names, catalog byte offset |
| `ValueDecodeCatalog` | Catalog version, CopperList encoding, shared payload schema/decoding graph, mission-indexed ordered output-slot maps |

Each slot records task/channel identity, message type, and optional schema
binding. Sort mission names deterministically and use the same indices in catalog
slot maps and section headers; the name table exists even with catalogs disabled.
Schema traversal is deterministic. Application metadata is ordinary bincode;
the catalog is bincode compressed with Heatshrink.
Each occupies one section, sized before writing. No whole-catalog RAM buffer is
needed. Metadata is published only after complete writes; runtime data follows.

Logger creation initializes the main header. The first app construction supplies
and seals the static metadata before resources or runtime streams are initialized.
Subsequent constructions reuse it after comparison. Independent writers can seal
an empty metadata region before writing their first data section.

Append validates the header and compares both static metadata bodies before any
write or header change. Equality covers the recorded identity, configuration and
catalog; timestamps, run IDs, instance IDs and configuration source are
per-construction information. Different missions of the same application use the
same catalog. A mismatch fails:

```text
Cannot append: static metadata does not match; appending different applications
or versions to the same log is not supported.
```

Matching append reuses the metadata offsets. Initialize an in-memory run-ID
counter above the highest ID in retained sections (start at 1 for an empty log);
increment it for each new construction, including interleaved instances. There
is no persisted next-ID counter. Static metadata remains outside the data ring.

## Construction and lifecycle

`run_id` distinguishes full reconstructions, whose message IDs or clocks may
restart, even when the mission and instance match. Each successful construction
has one run ID shared by every associated section. `Instantiated` is a timestamped
marker with configuration source. Run, instance and mission come from the section;
configuration, version and Git information come from static metadata.

| Operation | Record / outcome |
| --- | --- |
| Create logger | Initialize header; metadata awaits the first construction |
| Reopen for append | Validate clean close and matching metadata; enable writes |
| Build app successfully | New run ID; `Instantiated`; state `Initialized` |
| Build fails | Return error; no successful-construction marker or runnable app |
| Start succeeds | `MissionStarted` after startup hooks; state `Running` |
| Run an iteration | Ordinary messages/keyframes/text; same construction |
| Stop succeeds | Drain pending task output; `MissionStopped { reason }`; state `Stopped` |
| Restart stopped app | `MissionStarted`; same run ID and metadata |
| Construct another mission/app instance | Compare static metadata; new ID and `Instantiated` |
| Start/stop fails | `LifecycleFailed { operation, error }`; state `Faulted`, cleanup through stop |
| Iteration fails | `LifecycleFailed { operation: Iteration, error }`; caller decides whether to continue or stop |
| Panic | Best-effort `Panic { message, file, line, column }`; cleanup when possible |
| Finally tear down construction | Drain/close its streams; `ShutdownCompleted` on successful teardown |
| Close logger | Flush owned sections and publish clean-close state |

Every lifecycle record carries its timestamp. Stop reasons are typed:
`Requested`, `Completed`, `Error`, `Panic`. `ShutdownCompleted` ends a construction;
ordinary stop/restart does not. Logger closure is separate from app shutdown.
Failed transitions must not emit successful start/stop/shutdown records. Missing
records after a crash remain missing; readers do not invent them.

```text
static metadata (one application/configuration/catalog)
  run 1: Instantiated -> Start(A) -> data -> Stop(A) -> Start(A)
         -> data -> Stop(A) -> ShutdownCompleted
  run 2: Instantiated -> Start(B) -> data -> Stop(B) -> ShutdownCompleted
  close -> matching append
  run 3: Instantiated -> Start(A) -> data ...
```

## Rollover at section boundaries

One section-chain format serves both backends. The constructor chooses the
capacity policy; it is not persisted as a linear/ring mode:

| Policy when space is needed | Behavior |
| --- | --- |
| Grow | Extend backing storage and `sections_end`; preserve retained sections |
| OverwriteOldest | Reclaim oldest sections within `[sections_begin, sections_end)` |
| StopWhenFull | Return a space error |

Policy can change on append. Overwrite → grow preserves what remains; grow →
overwrite begins reclamation when needed. Existing byte links preserve chronology;
discarded history stays discarded. Growing SD storage is bounded by the partition.
Reject shrinking the region beneath retained sections.

1. Derive the next allocation position from the tail. For overwrite, wrap to
   `sections_begin` when the section cannot fit at the end. For grow, use new
   space at the previous `sections_end` if the next position overlaps retained
   sections or exceeds the boundary.
2. Preflight reclamation of oldest **closed** sections in order until that space is free.
   Never overwrite an open section; return an explicit space error before
   modifying the log if it blocks allocation. Persist the advanced head before
   reusing reclaimed bytes.
3. Initialize the section, then publish its link and head/tail offsets. It stays
   within the logical byte bounds, even when it crosses physical backing files.
4. Commit complete entries through `used`; close/flush a section before reclaiming
   it. Failed writes leave the committed length and writer position unchanged,
   including on SD. Reject entries larger than an empty section and sections
   larger than the capacity allowed by the constructor.

Streams must seal/rebind idle partial sections before reclamation; an idle stream
cannot pin the ring indefinitely. Lifecycle markers use promptly closed sections.
Distinguish an idle stream from a write in progress and protect the latter. Section
handles must never continue writing to reclaimed storage.

```text
before: metadata | [oldest X][Y][newest Z][free]
after:  metadata | [new W][retained Y][retained Z]
                   tail=W; head=Y
read order: Y -> Z -> W (byte links); metadata offsets stay unchanged
```

Only static metadata is permanently retained. Lifecycle events rotate alongside
payloads and keyframes. A surviving section carries run, instance and mission
identity even if its `Instantiated`/`MissionStarted` records rolled out. Readers
identify the mission immediately, group by run ID, and report a cropped lifecycle
without inventing missing start times. A section cannot mix contexts; readers use its persisted identity. The existing
process-global structured-text logger keeps its routing in this prerequisite PR:
text goes to the most recently installed sink. Per-iteration context switching
is deferred to preserve the existing real-time cost. CopperList, keyframe and
lifecycle streams belong to their individual construction.

```text
head -> CopperLists {run=7, mission=A}  # earlier start markers rolled out
     -> MissionStopped(A)
     -> Instantiated {run=8, mission=B}
     -> CopperLists {run=8, mission=B}
```

Append requires a cleanly closed log. Inspection can read committed payload from
an incomplete log and report open/invalid tails. Validate offset bounds, links,
section sizes and committed lengths; malformed state must not trigger overwrite.

## PR boundaries and acceptance

The prerequisite PR implements byte addressing, metadata sections, construction
contexts, lifecycle records, append validation, reader/run discovery, and rollover
on **both mmap and SD/eMMC**. Adapt logger/export APIs and documentation together.
The following self-describing-logs PR supplies native schemas and the compressed
catalog through that metadata interface.

Preserve native per-cycle encoding: no new payload copies, serialization passes,
heap allocations or catalog work. Metadata comparison/compression belongs to
construction. Section allocation must use bounded bookkeeping and retain direct
payload writes on mmap and fixed block buffers on SD.

Acceptance: maximum-value section headers fit 512 bytes; numeric mission indices
resolve through static metadata; matching/mismatching append without destructive
writes; capacity-policy changes on append; all lifecycle transitions and failures;
interleaved instances; sections spanning backing files; repeated byte-ring wrap
with metadata intact during a long-running mission with
sparse lifecycle/text/keyframe output; open-section protection; oversized
entries; partial-block writes and failed-write rollback; cropped runs and
incomplete tails. Exercise mmap and an in-memory SD block device with identical
traces, plus embedded/no_std and determinism checks.
