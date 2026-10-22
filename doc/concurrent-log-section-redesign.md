# Concurrent log section redesign

[Visual comparison: ownership, locks, cycle costs, and design tradeoffs](concurrent-log-section-redesign.svg).

Replace shared header mutation with explicit ownership of allocation metadata,
section contents, and completed runtime output. Preserve direct native encoding,
preallocated memory, bounded handoff, deterministic replay, and `no_std` support.

Backward compatibility is not required. Logger APIs and the container layout may
change; update in-tree writers, readers, adapters, and generated code together.
Do not carry compatibility shims into the new design. Record semantics and replay
correctness remain requirements, independently of byte-format compatibility.

## First delivery

Deliver the ownership split, append-only CAS removal, and runtime worker
consolidation as one required PR chain without adding a crash-recovery subsystem.
The format PR can retain the existing workers and leases as an intermediate
state, but the first delivery is complete only after the performance changes land.
Preserve capacity policies, native payload encoding, preallocated storage, and
explicit flush guarantees throughout the chain. The SVG shows the required
ownership and performance goals for this delivery.

The initial format needs allocator-owned descriptors, encoder-owned commit
metadata, section identity/generation, and bounds validation. Use separate physical
blocks for the two writable metadata owners. Use the existing format-version
field to reject incompatible logs; a future format version can introduce stronger
recovery. Do not add checksums, journals, duplicate headers, automatic repair, or
per-record durable flushes in this delivery.

Require correct concurrent operation, failed-encode rollback of the committed
length, clean close/reopen, and propagation of explicit flush errors. Inspection
of an unclean log may expose its valid reachable committed prefix, stopping or
reporting an error at invalid metadata. Do not promise recovery of every committed
record after a crash or power loss. Continue rejecting append to unclean logs;
automatic repair and resume are outside this delivery.

## Review evidence

The review covered branch `gbin/fix/logger-concurrent-sections` at
`a18706a00ddb57f1150ef9d18ba80b4e696b2863` on 2026-10-20.

The 11 unified-log integration tests initially passed. Repeating the existing
`concurrent_stream_rollover_preserves_every_record` test failed on run 46 with
`Section lease is sealed or busy`. The test uses `CapacityPolicy::Grow`; free
capacity does not prevent the failure. No test or implementation changes were
needed to reproduce it.

Source inspection identified these problems:

- Lease selection reads the same atomic state twice. Observing `BUSY` on the
  first read and `IDLE` on the second satisfies the predicate for a reusable
  slot, allowing a live generation to be replaced.
- `LogStream` checks whether its section is sealed before separately attempting
  acquisition. Allocation can set `LINK_PENDING` between those operations.
  Acquisition then reports a generic encoding failure, and an async output
  worker stops on the sink error.
- Allocators update successor links while encoders update committed lengths in
  the same serialized header. Deferred links, pending bits, claims, generations,
  and release-time header repairs coordinate these overlapping responsibilities.
- Section allocation scans the retained chain even under `Grow`. Main-header
  publication also reaches a flush of the entire backing mapping. Rollover cost
  therefore depends on log history and storage behavior.

The repeated test confirms a failure symptom. It does not identify which exact
interleaving caused that run; the two lease defects require separate regression
tests.

Implementation anchors:
[section allocation and leases](../core/cu29_unifiedlog/src/byte_log.rs),
[stream rollover](../core/cu29_unifiedlog/src/lib.rs),
[mapped storage](../core/cu29_unifiedlog/src/memmap.rs),
[runtime output workers](../core/cu29_runtime/src/curuntime.rs), and
[concurrency tests](../core/cu29_unifiedlog/tests/byte_log.rs).

## Supported concurrency

`async-cl-io` selects where completed CopperLists and keyframes are serialized.
It does not make other logging callers single-threaded when disabled.

Structured logs run on their callers' threads, including background tasks and
parallel runtime workers. Independent streams share an allocator. Ring overwrite
can also revoke an idle stream's section in a sequential application, so stale
handle protection is not exclusively a threading concern.

Use one allocator per physical log and one mutable writer per stream. Support
concurrent independent streams while keeping allocation decisions serialized.
Separate three concerns: runtime handoff, shared-stream serialization, and
storage reclamation. Feature flags should not substitute for an ownership model.

## Ownership and invariants

| State | Owner | Coordination |
| --- | --- | --- |
| CopperList during execution | Runtime | Exclusive slot lease |
| Completed CopperList and associated keyframe | Output worker, or inline output in sync mode | Ownership transfer through bounded queues |
| Structured-log stream | Serialized structured-log sink | Existing sink synchronization |
| Allocation geometry, chain links, retained range | Allocator | Short allocator critical sections |
| Section payload and commit metadata | Encoder while writing | Exclusive section access |
| Reclaimed section | Allocator after acquiring reclamation rights | Generation change before storage reuse |

Enforce these invariants:

1. Allocating a successor never requires modifying encoder-owned bytes of its
   predecessor.
2. An active encoder's storage cannot be reclaimed, rebound, or assigned another
   generation.
3. A stale handle acquires no write access. It receives an explicit rebind
   outcome before touching storage.
4. Failed encoding does not advance the committed payload length or cursor.
5. Readers expose complete committed entries and validate bounds and generations
   before following reused storage.
6. Successful shutdown means output was drained and the requested flushes
   completed. An earlier error does not bypass worker cleanup.
7. Retained records keep their run, instance, mission, message IDs, and replay
   ordering. Missing records remain observable as gaps.

Keep unsafe storage access behind the narrowest interface that enforces these
rules. Document each unsafe implementation in terms of the ownership invariant
it relies on.

## Storage layout and publication

Separate allocator-owned descriptors from encoder-owned commit metadata. The
allocator owns section identity, byte extent, successor links, and the retained
head/tail. The encoder owns its payload cursor and committed length while it
holds writing rights. An allocator may seal encoder metadata only after acquiring
exclusive reclamation rights.

Place independently writable metadata in separate physical blocks. Distinct byte
fields within one SD sector are insufficient when adapters perform whole-block
read/modify/write operations. Specify the block layout and publication order
before implementing either adapter; use the same logical format for mmap and
SD/eMMC.

This split removes the need for `LINK_PENDING`, shared successor-link atomics,
and encoder-side repairs of allocator metadata. Keep allocator bookkeeping in
allocator-owned structures rather than sharing mutable geometry with encoders.

Define the in-process allocation, commit, and reclamation ordering together:

- Initialize a section before publishing a reachable descriptor.
- Publish committed length only after a complete successful entry write.
- Publish removal from retained history before reusing reclaimed bytes.
- Reject invalid or incomplete metadata during inspection instead of interpreting
  uncommitted bytes as records.

Logical commit and durable flush are separate operations. Specify what an
explicit successful flush guarantees using the existing adapter behavior. The ordering
above does not establish durable write ordering or power-loss atomicity. Keep
interrupted-log inspection bounded by the validation rules in First delivery;
stronger crash recovery can be designed in a later format revision.

Version 3 reserves 1024 bytes per section: a 512-byte allocator descriptor,
a 512-byte encoder commit block, then native payload bytes. Extents and origins
remain aligned to 512 bytes. Descriptors carry origin, generation, type, context,
extent, and successor origin/generation; commits carry origin, generation,
committed payload length, and open state. Readers assemble `SectionHeader` as a
view rather than decoding that view directly from disk. Matching identities and
bounds are mandatory before exposing payloads or following links.

Initialization commits the empty encoder block, writes its descriptor, then
publishes the predecessor link and main retained range. Reclamation claims every
affected encoder, seals its commit block, publishes removal, then reuses bytes
with a new generation. Link publication writes only the descriptor block.
Explicit section flush commits closed state and flushes its backing region;
clean logger close seals remaining idle sections and flushes the main header.
The adapters retain their existing flush behavior and durability limits.

## Section acquisition and reclamation

Replace the precheck/acquire sequence with one authoritative acquisition
operation. Return a typed result that distinguishes acquired access from a
section that needs rebinding. Normal revocation is not an encoding or storage
failure.

For overwrite mode, specify one small state machine for idle, writing,
reclaiming, and sealed/reused sections, with a generation protecting stale
handles. Acquisition and reclamation compete through an atomic ownership
transition. Read a state once when classifying it; use compare-exchange when
claiming it. RAII releases writing rights on encoding errors and unwind.

An idle partial section may be revoked and sealed so a sparse stream cannot pin
the ring indefinitely. Its next append rebinds through the allocator. A writing
section cannot be revoked. A blocked oldest section produces a controlled
capacity/overload outcome before destructive mutation, without spinning on the
real-time thread.

Append-only policies do not reclaim live storage. After removing shared header
mutation, eliminate their per-entry reclamation CAS entirely: ordinary appends
under `Grow` and `StopWhenFull` must perform zero reclamation CAS operations.
Retain only synchronization needed for publication, faults, and shared stream
access. Overwrite mode retains the atomic writer/reclaimer ownership transition
and generation protection. Select this behavior from the existing capacity
policy; do not add a user-facing concurrency setting. CAS removal is required
within the first delivery, not a benchmark-dependent follow-up.

For append-only policies, the section handle owns its commit block until explicit
flush or drop. Ordinary appends use that exclusive ownership directly and perform
no reclamation atomic transitions. Explicit logger close rejects live unflushed
append-only handles; owners must flush or drop them first. Dropping such a handle
best-effort seals and flushes it, with storage failures poisoning clean close.
Overwrite retains per-entry acquisition and allocator revocation of idle handles.

## Runtime output

Keep preallocated CopperList slots and bounded SPSC handoff. Move ownership of a
completed slot rather than cloning payloads or constructing an intermediate
encoded buffer. Encode directly into the storage adapter in one native pass.

Preserve compile-time typed wiring and keep host worker machinery outside
`no_std` code. Use the same output engine in both modes: synchronous output invokes
it inline; asynchronous output transfers work to one runtime output worker.

Consolidate CopperList and associated keyframe output into one worker and one
bounded pending/completion SPSC pair, following the existing distributed-path
precedent. Reuse the current preallocated keyframe storage through ownership
transfer. Consolidation must not introduce payload copies or additional
serialization passes. This is a required PR in the first delivery. Measure
equivalent-work throughput, producer latency, queue pressure, and drops to catch
and resolve regressions; measurements validate the implementation rather than
decide whether to defer consolidation.

Concurrent structured logs retain their own serialized stream. Independent
streams retain independent encoding; do not hold the allocator mutex across a
large encode. The design does not require routing every record through one
global worker. Invoke user callbacks outside logger locks.

Keep lifecycle ordering explicit: drain the relevant output before publishing
successful stop/shutdown markers. Cleanup must signal, wake, and join workers
even after an output error, reclaim or release every owned slot, and explicitly
flush sinks. Preserve the original failure while reporting cleanup failures.
`Drop` remains best-effort cleanup; explicit close reports success or failure.

## Bounded costs and overload

For `Grow`, allocate from known geometry without scanning retained history,
including after reopening a log that previously wrapped. Keep enough allocator
metadata to distinguish occupied extents from free space. For overwrite mode,
inspect only the oldest sections needed to satisfy the request. Reconstruction
of allocation metadata belongs at open/recovery time.

Preallocate stream bookkeeping and runtime handoff storage. Move mapping growth
and durable flush work away from the async runtime producer. Account separately
for structured-log rollover, since those calls still run on their callers'
threads. Avoid whole-mapping flushes in routine allocation where a bounded
metadata update suffices. Any change in durability guarantees must be explicit.

Define queue exhaustion, reclamation blocked by an encoder, full storage, encode
failure, and storage failure as distinct outcomes. Preserve bounded execution
without retry loops on the real-time producer. Under sustained overload, bounded
memory, lossless output, and nonblocking production cannot all be guaranteed.

Start from the current async drop-newest behavior, account for rejected IDs and
keyframes, and expose gaps to inspection/replay. A combined handoff must define
what happens to both a CopperList and its associated keyframe when rejected.
Keep transient capacity outcomes separate from faults that stop the worker.

## Future extension: event capture

A future recorder may keep a continuous baseline (for example, highly compressed
video and selected sensors) alongside a rolling buffer of higher-detail data.
An event preserves a window before its trigger and continues capturing for a
window afterward, while a fresh buffer starts the next capture. This is outside
the first delivery; it adds no event API, format fields, or retention machinery
to this chain.

Keep baseline retention independent from event-buffer overwrite. Separate physical
logs, each with its own allocator, are one possible implementation; a shared log
would need explicit retention domains. Switching the whole log from
`OverwriteOldest` to `Grow` would also retain unrelated streams and does not by
itself implement repeated event capture.

Keep write ownership and retention distinct. A sealed section has finished
writing; that does not imply it is eligible for reclamation. Future preservation
of a section generation belongs to the allocator, separately from an encoder's
short writing lease. Event capture could rotate preallocated buffers and transfer
ownership of a completed recording to an archive worker, without copying payloads
on the runtime producer. Overlapping events must share or extend retained windows,
or consume a bounded buffer pool with an explicit exhaustion policy; an archive
worker cannot leave the producer waiting indefinitely for a reusable buffer.

Window selection must use record timestamps, not byte capacity alone: variable
video bitrate and sensor rates make a fixed-size ring an approximate time window.
Preserve the decoding prerequisites before the selected window, including video
codec reference frames, Copper keyframes, schemas, and application metadata. A
window also needs stable run/instance/mission and message identities across buffer
rotation, observable gaps, and explicit completion only after its post-trigger
output and requested flushes finish. The descriptor/generation split and output
ownership transfer in this design support those later decisions without reserving
new fields now. Trigger evaluation and archive callbacks stay outside logger locks.

## Implementation sequence

Each row is a separately mergeable PR in the required chain. The first row is
already represented in the current source; retain its deterministic tests as the
baseline. Intermediate PRs may retain costs removed by later PRs; completing the
header split alone does not complete the delivery.

| PR | Change | Review boundary and acceptance |
| --- | --- | --- |
| 1. Race fixes | Classify one lease snapshot and acquire through a typed append/rebind outcome. | Existing format and workers; deterministic slot-selection and revocation regressions. Already present in current source. |
| 2. Grow allocation | Allocate from known geometry without walking retained history; reconstruct geometry at open. | Existing format, leases, flushes, and workers. Cover clean reopen after a wrapped log and verify allocation visits do not grow with history. |
| 3. Header ownership | Specify and implement the minimal descriptor/commit block split, bump the format version, and remove deferred-link repairs. | Update shared byte storage, mmap, SD/eMMC, readers, exporters, and affected API snapshots together. Retain per-entry leases and existing workers. Cover concurrent rollover, overwrite/stale generations, partial encode, clean reopen, and interrupted-log rejection/inspection. |
| 4. Allocator lock scope | Move the first encode after rollover outside the allocator lock. | Acquire writing rights to the new section before unlocking so overwrite cannot reclaim it in the gap. Verify another stream can allocate during a paused encode; retain bounded capacity outcomes. |
| 5. Append-only CAS removal | Remove per-entry reclamation acquire/release CAS under `Grow` and `StopWhenFull`. | Zero reclamation CAS per ordinary append; retain overwrite ownership transitions and stale-generation protection. Cover all capacity policies and compare small-record costs at equivalent persisted work. |
| 6. Runtime output consolidation | Use one output engine inline or in one async worker for a completed CL and its associated keyframe; replace two SPSC pairs with one. | Move existing preallocated buffers without payload copies or added serialization passes. Define paired rejection/gap accounting, drain before successful lifecycle markers, wake/join on failure, conserve slots, and propagate flush errors. Move callbacks outside logger locks where required. Validate throughput, producer latency, queue pressure, drops, and failure cleanup. |

PR 3 is the coordinated format migration. It does not include runtime worker
consolidation or append-only CAS removal; PRs 5 and 6 deliver those required
benefits within the same chain. Bounded oldest-only ring scans and a new flush
policy can be reviewed separately. If a format invariant requires moving code
from another PR, move only that prerequisite and document why.

After these PRs, consider the remaining improvements independently:

- Bound overwrite scans to the oldest sections needed for the request.
- Bound routine metadata flushes while preserving explicit durability guarantees.
- Add checksums or stronger crash recovery only for an agreed failure model and
  recovery requirement. This may introduce a later format version.

Keep each stage reviewable. Do not introduce an abstraction unless it expresses
an ownership boundary or removes duplicated behavior. Run focused correctness
and equivalent-work performance checks in the PR that changes the relevant path;
do not defer validation to the end of the chain.

## Validation and acceptance

Use deterministic scheduling hooks or a small exhaustive interleaving model for
the ownership protocol. Repeated stress tests supplement these checks; passing
a stress run is not a proof of correctness.

Cover:

- Acquisition versus revocation; a writer finishing during slot selection;
  generation reuse with a stale handle; allocation while a tail encoder runs.
- Concurrent stream rollover under all three capacity policies, sparse-stream
  reclamation, tiny rings, and sections crossing backing files.
- Partial encoding, oversized entries, encode panic, storage errors at each
  publication step, clean close/reopen, and interrupted-log validation and
  rejection of unclean append. Torn-write recovery and automatic repair are
  follow-up work.
- Worker failure with pending input/completions, queue exhaustion, shutdown
  during failure, slot conservation, and propagation of flush errors.
- Sync and async output, serial and parallel runtimes, structured logs from
  background threads, mmap and SD/eMMC, debug and release, and `no_std` builds.
- Record identity and ordering, keyframe association, explicit dropped ranges,
  metadata retention across wrap, and deterministic record/resim behavior.

Benchmark small and large payloads, frequent keyframes, concurrent structured
logs, long growing logs, and repeated ring wrap. Measure persisted records and
bytes, rejected records, CPU, memory, and p99/p999 producer latency. Include drain
and flush time when comparing end-to-end throughput. Use equivalent overload and
durability guarantees: iteration throughput alone can reward dropping more data.

Accept the first delivery when the protocol has a small documented state space,
normal contention does not terminate logging, the writable metadata owners are
separate, and allocation avoids history-dependent scans on growing logs. Require
zero per-entry reclamation CAS under `Grow` and `StopWhenFull`, one runtime output
worker with one pending/completion SPSC pair on the async path, and the same
output engine inline on the sync path. Output cleanup must report errors and
conserve slots; paired CL/keyframe rejection must remain observable. Stronger
crash recovery does not gate acceptance. Require no new hot-path allocations,
payload copies, or serialization passes, and no unexplained latency or throughput
regression at equivalent completed work. Any proposed additional real-time cost
requires explicit approval before implementation.
