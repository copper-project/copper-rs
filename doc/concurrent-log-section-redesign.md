# Concurrent log section redesign

Replace shared header mutation with explicit ownership of allocation metadata,
section contents, and completed runtime output. Preserve direct native encoding,
preallocated memory, bounded handoff, deterministic replay, and `no_std` support.

Backward compatibility is not required. Logger APIs and the container layout may
change; update in-tree writers, readers, adapters, and generated code together.
Do not carry compatibility shims into the new design. Record semantics and replay
correctness remain requirements, independently of byte-format compatibility.

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

Define allocation, commit, reclamation, and recovery ordering together:

- Initialize a section before publishing a reachable descriptor.
- Publish committed length only after a complete successful entry write.
- Publish removal from retained history before reusing reclaimed bytes.
- Reject invalid or incomplete metadata during recovery instead of interpreting
  uncommitted bytes as records.

Logical commit and durable flush are separate operations. Specify what survives
a process crash and what is guaranteed after explicit durable flush. Preserve
inspection of the committed prefix of an interrupted recording. Do not imply
power-loss atomicity from atomics in RAM or from a multi-block write sequence;
choose and test the required recovery mechanism when defining the format.

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
mutation, evaluate eliminating their per-entry reclamation CAS entirely. Retain
only synchronization needed for publication, faults, and shared stream access.
Select this behavior from the existing capacity policy; do not add a user-facing
concurrency setting.

## Runtime output

Keep preallocated CopperList slots and bounded SPSC handoff. Move ownership of a
completed slot rather than cloning payloads or constructing an intermediate
encoded buffer. Encode directly into the storage adapter in one native pass.

Use the same output engine in both modes: synchronous output invokes it inline;
asynchronous output transfers work to a worker. Preserve compile-time typed
wiring and keep host worker machinery outside `no_std` code.

Start with one runtime output worker handling a CopperList and its associated
keyframe, following the existing distributed-path precedent. Reuse the current
preallocated keyframe storage through ownership transfer. Consolidation must not
introduce payload copies or additional serialization passes. Retain separate
encoders only if equivalent-work benchmarks show a throughput requirement.

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

## Implementation sequence

1. Capture deterministic regressions for the double-read slot selection and
   acquisition/revocation races. Record performance and loss baselines before
   changing the implementation.
2. Specify the ownership transitions, block layout, publication order, recovery
   behavior, and overload outcomes. Review the protocol independently of the
   adapter implementation.
3. Implement the shared allocator/section model and both storage adapters.
   Remove deferred-link machinery and redundant header ownership. Update readers
   and exporters with the new format; reject incompatible files clearly.
4. Consolidate runtime output, completion accounting, lifecycle ordering, and
   error cleanup. Update generated code and in-tree callers together.
5. Validate correctness and equivalent-work performance, remove superseded APIs,
   and update API snapshots and logging/replay documentation as applicable.

Keep each stage reviewable. Do not introduce an abstraction unless it expresses
an ownership boundary or removes duplicated behavior.

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
  publication step, interrupted recordings, clean close, and reopen/append.
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

Accept the redesign when the protocol has a small documented state space, normal
contention does not terminate logging, and allocation avoids history-dependent
scans on growing logs. Require no new hot-path allocations, payload copies, or
serialization passes, and no unexplained latency or throughput regression at
equivalent completed work. Any proposed additional real-time cost requires
explicit approval before implementation.
