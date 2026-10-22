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

The consolidated implementation sends a slot lease and, for serial capture, its
existing boxed keyframe buffer through one pending/completion pair. Distributed
captures remain attached to their arena slot. Both placements call the same
native output function, writing the keyframe before its CopperList. No worker
owns a second keyframe queue. Serial capture retains its two preallocated spare
buffers; when these are exhausted, freezing is skipped and the associated CL is
rejected too. Both counters advance, and the missing CL ID exposes the gap.

On a sink error or unwind, the worker latches the original error, returns every
pending item without further sink calls, and remains available for shutdown.
Explicit close drains completions, signals and wakes the worker, joins it, and
reports sink flush errors while attempting both sinks. Generated stop paths drain
even after component errors; failed stop cleans up output without publishing a
successful stop marker. Successful shutdown follows explicit output flushes.
Structured-log notifications snapshot the listener registry and release both
registry and sink locks before invoking callbacks.

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

## One-off Mandelbrot comparison (2026-10-22)

This compares the parent of #1543 (`e7649fc5158`) with the tip of #1549
(`cce9ff3abf0`), covering the complete stack. A temporary harness exercised
`cu-parallel-mandelbrot`'s `log_only` mission. Only these results are maintained;
no benchmark executable, script, recipe, or example configuration change is
included in the PR.

Both revisions used release builds with `parallel-rt` and `cu29/async-cl-io`,
identical resolved dependencies, and separate target directories. The host was an
AMD Ryzen 9 9950X3D2 (16 cores, 32 threads), Linux 7.2.10-1-cachyos, Rust 1.99.0,
and the same NVMe filesystem. CPU boost remained enabled on the live desktop
host. Each scenario had one 16-frame warmup per revision and five alternating
before/after pairs of 192 frames: 1920 × 1080 RGB, 512 iterations, 32 compute
bands, 64-row stripes, 32 in-flight CopperLists, 64 arena slots, and 64 source
pool slots. Stdout was redirected; the live viewer was unused.

- **Normal:** existing compact source codec and completed-image logging,
  keyframe interval 1,000,000, 64 MiB sections, and 512 MiB slabs.
- **Pressure:** keyframe interval 17 and one structured record per compute-band
  invocation, with the same section and slab sizes.
- **Rollover:** pressure settings with 16 MiB sections and 64 MiB slabs.

The temporary harness ran the existing cold-path keyframe sizing and arena
preparation before entering `run()`. Both revisions otherwise recorded no
keyframes through this parallel entry point: `run()` takes arena leases before
startup attempts preparation through the manager's free pool. This shared
startup issue is not fixed by this report. The workaround enabled actual
keyframe output in both measured revisions without changing runtime source.

Completed time starts before `run()` and ends after output drain, worker shutdown,
stream flushes, application drop, and explicit logger close. Logger construction,
the harness's cold preparation, and offline verification are excluded. Default
mmap flushing is retained; `mmap-fsync` is disabled on both sides. Logged latency
runs from source process start to frame-assembler process end for each of the
3,264 source stripes; it excludes subsequent async output completion. Values
below are medians of the five trials, with full completed-time ranges shown.

| Scenario | Completed s before → after | Change | Trial range s before / after | Logged p99 ms before → after | Logged p999 ms before → after |
| --- | ---: | ---: | --- | ---: | ---: |
| Normal | 4.348 → 4.306 | −1.0% | 4.283–6.375 / 3.838–6.531 | 48.484 → 48.082 | 50.847 → 50.518 |
| Pressure | 8.433 → 10.342 | +22.6% | 5.345–16.820 / 7.211–19.764 | 218.264 → 148.402 | 229.904 → 227.963 |
| Rollover | 16.416 → 11.323 | −31.0% | 7.650–19.755 / 9.115–19.853 | 182.644 → 181.957 | 215.209 → 214.995 |

Median user+system CPU seconds were 73.461 → 72.860, 77.650 → 79.329, and
85.768 → 82.117 for normal, pressure, and rollover respectively. Median peak RSS
MiB was 1,296 → 1,297, 2,916 → 2,905, and 2,869 → 2,970, including mapped logs.

Every timed run persisted all 192 frames and all 3,264 source stripes in order,
with identical per-frame digests across all 30 runs; the final frame digest was
`0x6089b61036d4bcaa`. Both revisions' matching readers decoded every CopperList,
keyframe, and structured record, and all 30 `fsck` checks passed. No CopperLists
or keyframes were dropped; CopperList IDs were contiguous and every keyframe had
its associated CopperList. Normal runs recorded 192 structured records and one
keyframe; pressure and rollover recorded 104,640 structured records per run.

Shutdown admits a small variable tail of empty work: normal recorded 3,318–3,319
CopperLists, pressure 3,314–3,326, and rollover 3,317–3,322. Pressure had 195–196
keyframes before versus 196 after; rollover had 196 on both sides. Thus frame and
source work match exactly, while one pressure trial has one fewer tail keyframe.
Used record bytes were approximately 1.204 GB for normal and 2.414–2.420 GB for
pressure/rollover. Normal used three slabs, pressure seven, and rollover 59 before
versus 55 after.

This stress run supports concurrent logging correctness at the tested load. It
does not establish a general throughput improvement: normal is effectively flat,
pressure has a slower median completed time despite a lower logged p99, and the
rollover median improves with widely overlapping trial ranges. The pressure
difference cannot be isolated to runtime costs from disk-backed measurements
alone; see the tmpfs control below. The shared keyframe startup issue also needs
separate follow-up. Existing microbenchmarks remain evidence for their specific
costs, rather than an application-wide speedup claim.

### tmpfs control (2026-10-22)

Reused the exact before/after binaries, workload, keyframe preparation, timing
boundaries, verification, and five alternating pairs per scenario above, writing
the log to tmpfs instead of NVMe. Each revision again received a 16-frame warmup
per scenario. No rebuild or runtime/configuration change was needed.

A single private directory under `/tmp` held one log family at a time. The runner
enforced a 4 GiB logical file-size limit, including the preallocated base file,
and a 4 GiB free-space/memory reserve. Actual peak tmpfs allocation was 2.615 GiB;
free tmpfs headroom stayed above 27.51 GiB. Each log was removed after decoding
and `fsck`; only the small result/verification files were retained on NVMe. The
tmpfs directory was removed at completion, restoring the initial free space.

| Scenario | Completed s before → after | Change | Trial range s before / after | Logged p99 ms before → after | Logged p999 ms before → after |
| --- | ---: | ---: | --- | ---: | ---: |
| Normal | 3.164 → 3.161 | −0.1% | 3.128–3.184 / 3.126–3.181 | 36.356 → 35.871 | 46.170 → 46.576 |
| Pressure | 3.161 → 3.154 | −0.2% | 3.135–3.170 / 3.139–3.191 | 36.104 → 35.870 | 46.492 → 47.703 |
| Rollover | 3.195 → 3.170 | −0.8% | 3.178–3.204 / 3.150–3.196 | 36.446 → 36.323 | 46.942 → 49.131 |

Median user+system CPU seconds were 71.917 → 71.884, 72.260 → 72.269, and
72.336 → 72.362 for normal, pressure, and rollover respectively. Median peak RSS
MiB was 1,287 → 1,287, 2,851 → 2,853, and 2,852 → 2,848.

All 30 tmpfs runs passed decoding, digest/order checks, and matching-reader
`fsck`, with zero CL/keyframe drops and no orphan keyframes. Every frame digest
matched the original NVMe runs. Pressure recorded 196 keyframes and 104,640
structured records in every run on both revisions; normal recorded one keyframe
and 192 structured records. Rollover recorded 195–196 keyframes and 104,640
structured records on both sides. Empty shutdown work remained variable, with
3,314–3,319 total CopperLists across the scenarios; all 3,264 source stripes and
192 images were persisted in every run.

The +22.6% disk-backed pressure increase did not reproduce on tmpfs: completed
time and CPU cost are effectively unchanged between revisions, with much tighter
trial ranges. This strongly implicates storage/flush interaction in the earlier
result. It does not identify NVMe hardware, filesystem behavior, or the code's
flush pattern as the specific cause. The disk-backed data remains relevant to
storage-backed deployments, but it is not evidence of a general 23% runtime
slowdown. The tmpfs measurements also do not demonstrate an application-wide
throughput improvement.
