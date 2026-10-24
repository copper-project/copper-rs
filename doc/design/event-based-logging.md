# Event-based logging

**Proposal; none of the event APIs below exist yet.** Compatibility is optional.
This changes how recordings are kept and selected; the existing native payload
encoders remain the starting point.

## 1. What changes for a robot

Example graph: `camera → detector → tracker → planner`.
An operator wants every detection saved normally, plus full camera data for
**30 minutes before and 2 minutes after** an interesting detection.

| | Today | Proposed |
| --- | --- | --- |
| Ordinary recording | One configured selection of task outputs | A baseline recording, such as detections only |
| Camera history | Record images continuously or have no past images | Continuously write images into temporary disk history |
| Trigger | No event-retention API | Keep the requested history and continue keeping the next 2 minutes |
| Expiry | Rollover overwrites oldest sections | Delete old temporary recordings unless an event or reader still needs them |
| Replay input | Read an ordinary log | Read an event's selected cycles plus earlier state and inputs needed to reproduce them |

**A trigger saves already recorded data.** It cannot recover images that were
never written. This reduces permanent storage and upload volume; it still pays
continuous image encoding, disk bandwidth, and flash wear. A window longer than
RAM lives on disk, not in a queue of image handles.

## 2. The logger's layers, from messages to disk

Existing source links below are relative to this document. Proposed modules are
marked **new**.

```text
Task outputs: CuMsg<T> in a runtime-owned CopperList
    │
    │ completed cycle + optional KeyFrame (saved component state)
    ▼
Runtime output engine — curuntime.rs
    │ inline today, or one bounded async worker with async-cl-io
    ▼
Generated selection + encoding — cu29_derive
    │ choose slots, apply codecs, encode CopperList/KeyFrame records
    ▼
Typed stream — WriteStream<E> / LogStream
    │ append encoded records into sections
    ▼
ByteLogger — byte_log.rs
    │ allocate sections; own headers, links, commit lengths, reuse rules
    ▼
Storage adapter — memmap.rs or SD logger
      map backing files, or write device blocks
```

These objects are different sizes and have different owners:

| Notion | Concrete meaning |
| --- | --- |
| CopperList | One runtime cycle's messages and metadata; lives in a reusable arena slot |
| KeyFrame | Frozen task/bridge state used to resume replay; associated with a CopperList |
| Stream | A typed writer for a record family, such as CopperLists or keyframes |
| Section | A byte range allocated to a stream; its header tracks type, context, links, and committed bytes |
| Slab | One backing file/mapping; a section can cross slab boundaries |

The proposal adds an **event recorder above `ByteLogger`**. It keeps a sequence
of small independent unified logs, called **capture logs** here. Each has its own
header, application metadata, optional decode catalog, sections, and backing slabs.
A capture log is the unit deleted or retained. There is no new kind of section.

```text
Existing completed-output worker; owns ONE CopperList/keyframe pair
    ├─ generated baseline selection ─► ordinary unified log
    └─ generated capture selection  ─► current capture log
                                            │ rotate after size/time limit
                                      next capture log

Event coordinator, on a separate control thread:
    triggers ─► choose capture logs to keep ─► event manifest on disk
    expiry   ─► delete eligible closed capture logs
```

An **event manifest** is a small description of an event: requested interval,
selected profile, ordered capture-log IDs, replay starting point, gaps, and status.
It refers to existing bytes. Two overlapping events share those bytes.

### Existing modules to extend

| Module | Responsibility today → change |
| --- | --- |
| [`cu29_runtime/src/curuntime.rs`](../../core/cu29_runtime/src/curuntime.rs) | `RuntimeOutputItem` owns a completed arena-slot lease and optional keyframe; `write_completed_output` writes keyframe first → preserve that owner/order, add raw cycle time and route both selections before returning the slot |
| [`cu29_runtime/src/config.rs`](../../core/cu29_runtime/src/config.rs) | `LoggingConfig` describes ordinary logging → add baseline selection and event-profile declarations |
| [`cu29_derive/src/lib.rs`](../../core/cu29_derive/src/lib.rs) | `build_compressed_culist_tuple_encode` generates slot/codec selection → generate independent baseline and capture encoders |
| [`cu29_derive/src/runtime_macro.rs`](../../core/cu29_derive/src/runtime_macro.rs) | Builder, `prepare_log_metadata`, output wiring → attach recorder, prepare catalogs, request continuous keyframes, bind typed profile handles |
| [`cu29_export/src`](../../core/cu29_export/src/lib.rs) | Read sections/runs and decode records → add event reader across ordered capture logs, then reuse decoders |

### New modules and their owners

| Proposed location | Owns |
| --- | --- |
| `core/cu29_runtime/src/eventlog.rs` | Fixed-size request/ticket/profile types and preallocated trigger handles; shared definitions use `core` types |
| `core/cu29_eventlog/src/recorder.rs` | Host `CuEventRecorder`, control thread, trigger admission, event status, reader leases, deletion decisions |
| `core/cu29_eventlog/src/store.rs` | Prepared/current/closed capture logs, file mappings, rotation, bounded disk index/cache |
| `core/cu29_eventlog/src/manifest.rs` | Versioned event references, acceptance/deletion journal, restart recovery |
| `core/cu29_export/src/events.rs` | Event iteration and standalone export; borrows a recorder lease while reading |

`cu29_eventlog` is a proposed host-only crate depending on unifiedlog and shared
runtime types. The runtime does not depend on that host crate: generated host
wiring connects it through internal output hooks. The ordinary `no_std` logger
keeps its existing path. An SD implementation of event retention is later work.

The **output worker exclusively owns the current capture logger and its streams**.
The **control thread owns manifests, admission, prepared-log creation, and deletion**.
They exchange prepared logs and closed-log range notices through bounded queues;
the encoder does not take the coordinator's lock for each record.

## 3. Application API and configuration

All code in this section is **proposed**, not a working example today.

### Before and after at the application boundary

```rust
// Today: one ordinary recording.
let mut app = App::builder()
    .with_log_path("logs/robot.copper", Some(64 * 1024 * 1024))?
    .build()?;
```

```rust
// Proposed: baseline stays ordinary; a separate store holds event history.
let mut recorder = CuEventRecorder::open("logs/events", limits)?;
let mut app = App::builder()
    .with_log_path("logs/baseline.copper", Some(64 * 1024 * 1024))?
    .with_event_recorder(&mut recorder)?
    .build()?;

// AppEventProfile is generated from this app's declared profiles.
let clock = app.clock();
let mut trigger = recorder.trigger_handle(AppEventProfile::Perception)?;
let ticket = trigger.try_trigger(CuEventRequest {
    sequence: 1,
    anchor: clock.raw_now(),
    reason: CuEventReason::Manual,
})?;
```

`limits` is a deployment-supplied `CuEventStoreLimits`: history bytes, saved-event
bytes, maximum capture-log bytes/duration, maximum encoded cycle size, maximum
active events, and fixed queue/cache/reader capacities. Paths and device capacity
belong here; task selection belongs in RON.

Proposed replacement for the logging block, for the example graph:

```ron
logging: (
    baseline: (outputs: ["detector"], every_n_cycles: 1),
    events: (
        max_trigger_lateness_s: 5,
        max_checkpoint_age_s: 10,
        profiles: [
            (
                id: "perception",
                before_s: 1800,
                after_s: 120,
                replay_tasks: ["detector", "tracker"],
            ),
        ],
    ),
    keyframe_interval: 100,
),
```

`baseline` replaces the old single archive-selection decision. An empty `outputs`
list requests no baseline payloads. Existing codec declarations still specify
payload encoding. The profile compiler derives camera output as the recorded
input at the selected replay boundary, validates every other incoming edge, and
includes execution context and keyframes. All event profiles share the union of
required inputs, at full rate with a lossless encoding per slot. A baseline
exclusion cannot disable event capture.

The first API uses fixed per-profile windows; it has no per-request override.
Names resolve to typed slot indices/profile IDs at generation, never on the
trigger caller. Changing the compiled capture selection begins new history;
it cannot supply a past window from before it was armed.

### Trigger and control signatures

Signature sketch; ID, status, and lease types are opaque public types:

```rust
pub struct CuEventRequest {
    pub sequence: u64,
    pub anchor: CuInstant,
    pub reason: CuEventReason,
}

pub enum CuEventReason { Manual, Remote, Model, DatasetNovelty }

impl CuEventTrigger {
    pub fn try_trigger(&mut self, request: CuEventRequest) -> CuResult<CuEventTicket>;
}

impl CuEventRecorder {
    pub fn open(path: impl AsRef<Path>, limits: CuEventStoreLimits) -> CuResult<Self>;
    pub fn trigger_handle(&mut self, profile: impl Into<CuEventProfileId>) -> CuResult<CuEventTrigger>;
    pub fn request_status(&self, ticket: CuEventTicket) -> CuResult<CuEventRequestStatus>;
    pub fn event_info(&self, event: CuEventId) -> CuResult<CuEventInfo>;
    pub fn lease(&mut self, event: CuEventId) -> CuResult<CuEventLease>;
    pub fn delete(&mut self, event: CuEventId) -> CuResult<()>;
    pub fn close(self) -> CuResult<()>;
}
```

The generated enum converts to `CuEventProfileId`; binding checks that it belongs
to the attached app. New user-facing items are experimental; generated attachment
hooks are hidden. No new task trait is required.

`try_trigger` only enqueues a fixed-size request. Queue full returns an error
immediately. A ticket means **queued**, not accepted. `request_status` returns
`Queued`, `Accepted(event_id)`, or `Rejected(reason)`. Rejection reasons include
expired history, unavailable replay state, invalid time, capacity, and closing.

A trigger handle binds one profile, producer ID, and recording session at setup.
Each handle has one bounded producer queue; allocate extra handles before the
control loop for independent producers. Bound ticket retention and the retry
period at setup; repeated sequences return the same decision within that period,
and expired retries are rejected. Requests contain no strings,
images, or embedding vectors. Model evidence remains ordinary logged payloads
identified by producer sequence/cycle. A task receives its handle through existing
resource injection; manual, remote, ML, and novelty producers use this same API.

### Internal output boundary

Today's two independent `WriteStream` sinks cannot coordinate rotation of a
keyframe/CopperList pair. Extend `write_completed_output` with one internal
pair-aware consumer, rather than rotating in either leaf sink independently:

```rust
// Proposed internal hook in curuntime.rs; not an application task trait.
#[doc(hidden)]
pub trait CompletedOutput<P: CopperListTuple> {
    fn write_pair(
        &mut self,
        culist: &CopperList<P>,
        keyframe: Option<&KeyFrame>,
        admitted_at: CuInstant,
    ) -> CuResult<()>;
}
```

The generated consumer reserves the configured worst-case pair size, rotates if
needed, then invokes both encoders using these borrows. No sizing encode pass is
required. Preserve existing byte counters and sink-error
reporting in the implementation. Build the composition statically; expose only
hidden hooks needed by generated code. Builder attachment transfers owned/shared
connections, so the running app does not borrow the `CuEventRecorder` control handle.

## 4. Lifecycle and ownership

### Startup → recording → shutdown

1. **Open store.** Recover manifests before enabling deletion; preallocate queues,
   caches, and spare capture logs. The generated builder attaches immutable
   metadata/catalogs and one recording-session identity. Arming never constructs
   another robot or initializes its resources.
2. **Record cycles.** Runtime freezes state at configured intervals, then hands
   its completed slot to the existing async worker. The worker writes keyframe
   before CopperList into each selected output. After both outputs finish, it
   returns the slot and image-pool handles to the runtime.
3. **Rotate capture log.** At a completed-pair boundary, close all its streams,
   seal/flush the log, publish its time/cycle range, and switch to a prepared log.
   Preserve session/run/instance/mission IDs. Opening a new `ByteLogger` must not
   reset construction identity. A keyframe/CopperList pair stays together.
4. **Expire history.** The coordinator deletes a closed capture log only when
   it is outside required history, no event references it, and no reader lease
   holds it. Release every section handle/mapping before deleting its slab family.
5. **Stop and close.** App stop drains runtime output. Recorder close rejects new
   requests, finalizes or marks unfinished events incomplete, persists manifests,
   flushes and joins control work. Stop/start retains construction identity;
   teardown precedes recorder close. Drop is best effort; explicit close reports errors.

Event recording initially **requires `std` + `async-cl-io`**. Synchronous apps do
not acquire event encoding on their task thread.

### One trigger, concretely

A trigger at robot time **12:00:00** requests **11:30:00–12:02:00**.

1. The coordinator validates the request and temporarily prevents deletion of
   candidate history while looking up its ranges. It rejects missing history
   rather than silently shortening the interval.
2. It locates the last usable keyframe at or before 11:30:00, protects all needed
   inputs from that point, and reserves capacity through 12:02:00. It journals
   acceptance and references before allowing those bytes to be reclaimed.
3. As output finishes, it adds new capture-log references to the manifest.
   Overlapping events keep separate IDs/reasons and share the same logs.
4. It finishes only after the runtime has advanced past 12:02:00 **and** every
   admitted cycle through that point has been written or explicitly marked lost.
   A timer firing while output is backlogged is insufficient.
5. Deleting an event first persists its deletion, then releases its references.
   Another event, history requirement, or export lease may still keep the data.

### What the reader receives

`CuEventInfo` reports requested/actual bounds, known gaps, collection state,
durability (`Buffered` or `Synced`), and `replay_ready` separately. An event may
contain inspectable records and still lack enough state for replay.

Time-window membership uses a raw `CuInstant` sampled when the runtime admits
a cycle. This is when the runtime observes data; sensor `tov` remains available
for offline exposure-time queries. A session ID prevents comparison across clock
resets. Remote requests default to local receipt time; externally timestamped
requests need an explicit clock mapping. Delayed detectors may name earlier raw
times within the lateness bound. Control-queue delay consumes that bound.

For detector/tracker replay, restore their preceding keyframe, inject recorded
camera and other boundary inputs, and execute the prefix before 11:30:00 without
showing prefix results in the event view. Keep inputs through the whole prefix;
a trigger-time snapshot cannot restore past state. Initially retain full component
keyframes. Held inputs from still earlier cycles also need retention or a snapshot.
`keyframe_interval` counts cycles; the configured maximum age must also hold in
elapsed time. Missing state/inputs makes `replay_ready` false. Matching executable,
codec behavior, clock corrections, and scheduling context are still required.

## 5. Storage choice and failure behavior

Today's [`.rollover(size)`](../../core/cu29_unifiedlog/src/memmap.rs) uses
`OverwriteOldest`: one linked section chain reclaims its oldest prefix through
the next overlap. It cannot keep an arbitrary old event while reusing later
bytes. Adding a flag to one section does not solve that allocator constraint.

**Keep that allocator for ordinary logs. For events, rotate independent capture
logs using append-only allocation inside each.** Enforce size/time limits above
`ByteLogger`; reserve headroom for the largest allowed pair. Closed logs are
immutable, so any eligible log can be deleted independently. Metadata/catalog
repetition buys independent decoding. An event retains whole logs, potentially
more tasks/time than requested; export filters and repacks the selected records.

A capture-log ID contains persistent store identity and sequence/generation;
a reused filename is insufficient. The disk index stores ranges/checkpoint
locations. Manifests store ordered references in pages. Both use bounded caches:
long windows must not require all references or mappings in RAM.

| Failure | Required behavior |
| --- | --- |
| Capacity or active-event limit exhausted | Reject new event; preserve accepted events and the history/rotation reserve |
| Output queue full or oversized pair | Drop pair together, record a gap, clear affected replay guarantee; return owned buffers |
| Capture write failure or no prepared log | Report recorder failure/gaps; stop claiming complete coverage; never grow queues indefinitely |
| Optional baseline fails | Report separately; allow healthy capture to continue and return slots |
| Crash/restart | Recover ownership before deletion; ambiguous logs remain protected; interrupted future windows become incomplete |

Durable acceptance needs a journal ordering data flushes, range publication,
event references, and deletion. File/directory synchronization and crash fault
injection are required before advertising `Synced`. Existing mmap flush and
`clean_close` alone do not provide that guarantee.

Structured text logging in `cu29_log_runtime` remains caller-side and uses the
most recently installed process-global sink. Leave it in its normal archive;
event completeness initially covers runtime capture, not every text message.
Replay-required lifecycle/clock records need explicit context-bound capture.

## 6. Performance: what this costs

| Operation | Today → with events | Thread / consequence |
| --- | --- | --- |
| Finish a cycle | Existing handoff → plus raw-time stamp and small metadata | Runtime; new latency must be measured and approved before implementation |
| Freeze state | Only when required by current consumers → continuously required for armed replay profiles | Existing capture path; extra frequency/state work can affect control latency |
| Encode outputs | One local selection → capture plus baseline selections | Output worker; common slots encode twice, images excluded from baseline encode once |
| Hold payloads | Until existing output finishes → until both outputs finish | Longer worker service increases arena/image-pool pressure and drop risk |
| Keep an event | No API → index lookup, journal, references, future-byte reservation | Control thread; no past-window payload copy on trigger |

Handle-content policies and codec caches must be scoped to each selection.
Today's generated encoder can mutate handle logging policy; running baseline
`none` before capture must not suppress the capture image. Fix and test both
encoding orders without cloning images.

For encoded capture rate `R`, use bounded measured maxima, not average bitrate:

```text
history duration >= largest before + trigger lateness
                  + oldest required checkpoint/held input age
                  + output lag + capture-log rounding allowance
history bytes    >= R × history duration + state/metadata/alignment overhead
store bytes      >= history bytes + unique saved-event bytes
                  + reserved future bytes + rotation/admission reserve
```

At **80 MiB/s**, 30 minutes alone is **140.625 GiB**, another 2 minutes is
**9.375 GiB**, and continuous capture writes about **6.59 TiB/day**. These are
arithmetic examples, not measured Copper performance. Add baseline writes,
keyframes, metadata, and storage headroom. Overlapping events share physical
storage; unrelated saved events keep consuming quota until deletion/offload.

RAM holds in-flight slots, codec buffers, fixed queues, and bounded caches.
It does **not** hold `R × 30 minutes`. Event references grow on disk. Prepared
logs and mappings must remain bounded across repeated rotations.

No performance result is claimed. Before implementation is accepted, compare
ordinary logging, capture only, and capture plus baseline on identical input:
producer p50/p99/max, freeze cost, worker throughput, queue/pool occupancy, drops,
encoded bytes, rotation latency, and sustained disk writes. Extending the existing
[`cu-async-cl-io-bench`](../../benchmarks/cu_async_cl_io_bench) is the starting point.
Additional hot-path allocation, copies, serialization, or latency needs explicit
approval; this proposal does not grant it.

## 7. Migration: exactly what changes where

This is an implementation sequence for the logger, not a rewrite of task code
or a conversion of old recordings. Each row has a concrete result to review.

| Step | Code change | Evidence before proceeding |
| --- | --- | --- |
| 1. Rotate capture logs | Add `cu29_eventlog::store`; reuse `ByteLogger`/mmap, add explicit context wiring in `runtime_macro.rs` | One continuous capture rotates and deletes whole expired logs; IDs survive rotation; mapping/cache counts remain bounded |
| 2. Keep an event | Add `eventlog.rs`, recorder/coordinator, journal/manifests, builder attachment | A manual request retains the past/future interval across rotations; overlapping requests share bytes; restart cannot delete accepted data |
| 3. Produce both selections | Extend `LoggingConfig`, generated encoders/catalogs, `OutputRequirements`, and `write_completed_output` | Detections persist in baseline; event history has lossless camera inputs; policy isolation and queue/sink failures pass; overhead measured |
| 4. Read and replay events | Add `cu29_export::events`, leased reads, prefix/checkpoint selection, gap-aware status | Stateful detector/tracker replay matches recorded outputs; standalone export decodes independently; gaps prevent a replay-ready claim |
| 5. Expose complete workflow | Add trigger resource binding and a manual/ML example; test shutdown and crash publication points | The example captures the stated window, exports it, deletes it, and releases bytes; shared runtime still builds without `std` |

Application migration is concrete: replace one logging selection with baseline
and profile declarations; add recorder setup/attachment; inject a trigger handle
only into producers that need it; stop/drain the app before closing the recorder.
Existing source/task/sink lifecycle methods do not change.

Reader migration is concrete: event-aware tools open a manifest plus its referenced
logs, instead of opening one log base path. Standalone export rewrites section
links/offsets; concatenating slab files is invalid. Existing per-log decoders are
reused. Keep container/catalog versions only while their actual layouts match;
version the new manifest independently. No dual writer or legacy dispatch is
required solely for compatibility. Old recordings stay with their matching tools.

The first usable release needs all five steps, including conservative recovery
and leased export. Partial steps are engineering milestones. Upload, dataset
novelty scoring, and SD event retention are separate follow-up implementations.
