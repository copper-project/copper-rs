# Concurrent logging and runtime output

Implemented: format v3 separates allocator and encoder metadata; append-only
streams avoid reclamation CAS; completed CopperLists and keyframes share one
output engine, inline or in one async worker. [Ownership diagram](concurrent-logging.svg).
For metadata, identities, and capacity semantics, see [log storage](log-storage.md).

## Why the ownership split

The earlier implementation let allocation update successor links while an encoder
updated committed length in the same serialized header. Deferred links and lease
repairs coordinated overlapping ownership. Independent streams can encode at once,
even without `async-cl-io`: structured logs execute on caller threads, and background
and parallel tasks may log concurrently.

Review at `a18706a00ddb57f1150ef9d18ba80b4e696b2863` also found two lease races:
classifying a slot from two state reads could replace a live generation, and
revocation between a sealed precheck and acquisition could turn normal rebind into
an encoding error. A repeated Grow rollover test failed on run 46; that symptom
alone did not identify which interleaving occurred. Deterministic regressions now
cover both cases. Atomic ownership acquisition, rather than a separate precheck,
is authoritative.

## Owners and publication

| State | Owner / synchronization |
| --- | --- |
| Executing CopperList | Runtime holds an exclusive arena slot lease |
| Completed CopperList + keyframe | Inline output or async worker owns the submitted slot/buffer |
| Allocation geometry, links, retained range | One serialized allocator per physical log |
| Section payload and commit | Exclusive encoder; reclaimable sections acquire writing rights |
| Structured-log stream | Its own serialized sink; independent from runtime output |

A section has two independently writable **physical blocks**, not just separate
fields: SD adapters perform whole-sector read/modify/write.

| Offset within section | Owner | Stored fields |
| --- | --- | --- |
| 0–511 | Allocator descriptor | Origin/generation, type/context, extent, successor origin/generation |
| 512–1023 | Encoder commit | Origin/generation, committed payload length, open state |
| 1024 onward | Encoder | Native entry bytes |

Initialization writes the empty commit, then descriptor, then publishes predecessor
link and main retained range. Successor allocation changes only descriptor bytes.
Readers require matching origin/generation and valid bounds before exposing bytes
or following a link. Encode failure preserves committed length and cursor.
Reclamation claims affected writers, seals commits, publishes removal, then reuses
storage under a new generation. Explicit section flush seals and flushes the region.
These are logical ownership/publication rules; existing backend flush behavior does
not establish torn-write recovery or power-loss atomicity.

## Access and allocation costs

`Grow` and `StopWhenFull` never revoke live handles. A handle owns its commit block
until flush/drop, with **zero reclamation CAS per ordinary append**. Explicit logger
close rejects live unflushed append-only handles; drop attempts sealing and records
storage failure so clean close cannot silently succeed.

`OverwriteOldest` uses generation-tagged idle/writing/sealed lease states. Writer
acquisition and reclamation compete atomically; RAII returns writing rights on
errors and unwind. A stale/revoked handle gets typed `Rebind` before touching bytes.
An idle partial stream can be revoked; an active writer blocks reclamation without
spinning. Lease-slot classification uses one snapshot.

New sections reserve writing rights while holding the allocator lock, then release
that lock **before the first encode**. Another stream can allocate while that encode
is paused, and overwrite cannot steal the reserved section in the gap. Free-capacity
allocation does not wait for unrelated encoders.

`Grow` uses geometry reconstructed at open and performs no history walk, including
after a previously wrapped log. Non-Grow overlap detection still scans the retained
chain. Routine metadata publication and adapter flushes can still incur storage
costs; bounded overwrite scans and narrower flush policy are remaining work.

## One completed-output engine

`write_completed_output` writes the associated keyframe before its CopperList.
Sync calls it inline. `async-cl-io` moves a `RuntimeOutputItem` through one bounded
pending/completion SPSC pair to one worker. The item owns a slot lease and, for
serial capture, its existing boxed keyframe buffer. Distributed captures remain
attached to the slot. Both paths use the same native output pass without cloning
payloads or adding serialization.

Serial capture keeps two preallocated spare buffers. Queue or keyframe-buffer
exhaustion rejects the CopperList/keyframe pair; counters and missing CopperList IDs
expose the gap. This preserves bounded producer memory and nonblocking handoff under
sustained overload. Structured logs still serialize on their callers' threads;
runtime async output does not move every logging cost off the producer.

On sink error or unwind, the worker latches the original error and returns pending
items without more record writes. Explicit close drains completions, signals/wakes,
joins, and attempts both sink flushes while preserving failures. Generated stop
paths drain even after component errors; successful stop/shutdown markers follow
output completion. Drop is best-effort; use explicit close for a reported result.
Structured-log listeners are snapshotted, then invoked outside registry/sink locks.

## Evidence and limits

| Invariant | Regression anchor |
| --- | --- |
| No append-only reclamation CAS; metadata fits separate blocks; Grow avoids history walks; one-snapshot slot selection; stale rebind and failed-encode rollback | [byte-log unit tests](../../core/cu29_unifiedlog/src/byte_log.rs) |
| Concurrent rollover preserves records; free allocation proceeds during encode; first rollover encode releases allocator lock while retaining rights | [byte-log integration tests](../../core/cu29_unifiedlog/tests/byte_log.rs) |
| Keyframe precedes CopperList; slot returned only after both outputs; failed output drains/joins/flushes and conserves buffers | [runtime tests](../../core/cu29_runtime/src/curuntime.rs): `slot_output_writes_keyframe_then_copperlist_before_recycling`, `failed_output_drains_pending_pairs_joins_flushes_and_conserves_all_buffers` |
| Adapter block writes and rollback | [SD/eMMC implementation/tests](../../components/libs/cu_sdlogger/src/logger.rs) |

Historical application measurement, 2026-10-22: `e7649fc5158` (parent of #1543)
versus `cce9ff3abf0` (#1549), release `parallel-rt` + `async-cl-io`, Ryzen 9
9950X3D2, Linux 7.2.10-1-cachyos, Rust 1.99.0. Each scenario used a 16-frame
warmup and five alternating pairs of 192 1080p RGB Mandelbrot frames, 512
iterations, 32 bands, 64-row stripes, 32 in-flight lists, and 64 arena/source slots.
Identical dependencies and separate target directories were used.

Normal: compact source codec, completed-image logging, keyframe interval 1,000,000,
64 MiB sections/512 MiB slabs. Pressure: interval 17 plus a structured record per
band invocation. Rollover: pressure with 16 MiB sections/64 MiB slabs.

| Scenario | NVMe completed seconds, before → after (trial ranges) | tmpfs completed seconds, before → after (trial ranges) |
| --- | --- | --- |
| Normal | 4.348 → 4.306 (4.283–6.375 / 3.838–6.531) | 3.164 → 3.161 (3.128–3.184 / 3.126–3.181) |
| Pressure | 8.433 → 10.342 (5.345–16.820 / 7.211–19.764) | 3.161 → 3.154 (3.135–3.170 / 3.139–3.191) |
| Rollover | 16.416 → 11.323 (7.650–19.755 / 9.115–19.853) | 3.195 → 3.170 (3.178–3.204 / 3.150–3.196) |

Values are five-trial medians; timing includes run, drain, shutdown, flush, app drop,
and logger close, excluding construction and offline verification. `mmap-fsync` was
disabled. A temporary harness prepared keyframes before `run()` on both revisions
to bypass a shared parallel-startup issue; this was not unmodified application
startup. The harness was not retained, so these are historical observations rather
than a maintained reproducible benchmark.

All 60 NVMe/tmpfs runs decoded and passed matching-reader `fsck`, with all 192 frame
digests and 3,264 source stripes matching, zero CopperList/keyframe drops, contiguous
IDs, and no orphan keyframes. Pressure persisted 104,640 structured records per run;
empty shutdown work and tail keyframe counts varied slightly.

The disk pressure median rose 22.6% and rollover fell 31.0%, with overlapping ranges.
The pressure increase did not reproduce on tmpfs; CPU/completed time there was
nearly unchanged. This implicates storage/flush interaction without isolating its
cause. The results establish neither a general runtime slowdown nor a general
application speedup.

## Remaining boundaries

Automatic repair, unclean append, checksums/journaling for section recovery, and
per-record durable commits are not implemented. Inspection is limited to a valid
reachable committed prefix. A future event-window recorder would also need separate
retention policy, timestamp-based window selection, and decoding prerequisites;
sealing a section alone does not preserve it against overwrite.

Implementation: [storage protocol](../../core/cu29_unifiedlog/src/byte_log.rs),
[stream allocation/rebind](../../core/cu29_unifiedlog/src/lib.rs),
[mmap adapter](../../core/cu29_unifiedlog/src/memmap.rs),
[runtime handoff/output](../../core/cu29_runtime/src/curuntime.rs),
[generated lifecycle cleanup](../../core/cu29_derive/src/runtime_macro.rs).
