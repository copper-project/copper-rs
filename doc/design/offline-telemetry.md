# Offline telemetry plan

Status: proposed; no implementation changes in this document.

Replay should deliver every selected observation in order, slowing down whenever
the visualizer cannot keep up. Introduce a `std`-only `offline-telemetry` path that
blocks throughout the pipeline. Blocking is intrinsic to this path, with no
separate backpressure policy.

```text
replay -> capture/encode -> bounded handoff -> TCP -> bounded receiver -> consumer
```

## Contract and compatibility

Each stage waits for downstream capacity rather than dropping or expiring data.
Memory stays bounded. Waiting affects wall-clock duration, not replay timestamps
or task ordering. Success means the receiver accepted every selected observation
into its visualization consumer; it does not promise rendering or durable storage.
Disconnects, invalid records, and encoding failures fail replay explicitly.

Preserve existing live UDP behavior, public method signatures, saved log formats,
and `no_std` builds. Select offline wiring explicitly during construction, with
no per-iteration mode dispatch added to live execution.

## Implementation plan

1. **Blocking TCP transport.** Reuse `CuStreamTx`; revise its documented
   nonblocking requirement to permit explicitly offline implementations. Keep
   `try_send` for compatibility: offline success means the entire frame was
   written, and temporary socket pressure waits internally. Live implementations
   retain their immediate-return contract. Add versioned, length-prefixed framing
   with a maximum frame size and explicit handling of partial reads and writes.
2. **Offline sender.** Reuse capture/schema and record encoding, sending framed
   records directly through the transport. Bypass the live sender's packet
   scheduler, FEC, recovery repetition, queue shedding, and latency expiry. Any
   retained buffer handoff blocks when full. Include selected CopperLists and
   associated keyframes; lossless structured text logging is outside this scope.
3. **Replay integration.** Add an explicit offline builder attachment, conceptually
   `with_offline_telemetry(...)`, and wait for output after each replay iteration
   through the typed lifecycle. Waiting must reach the blocking sink before
   another iteration can exhaust the runtime output pool. Keep the existing
   `wait_for_output()` contract; final remote completion is a separate operation.
4. **Receiver and completion.** Feed decoded records into a bounded consumer and
   stop reading when it fills. Send an end-of-stream marker with the final sequence;
   acknowledge it only after all preceding observations reach the consumer.
   Explicit finish waits for that acknowledgment. Cancellation interrupts socket
   and queue waits; errors wake all waiters. Do not silently reconnect or retry a
   partially delivered session.
5. **Validation.** Replay into a deliberately slow receiver and assert bounded
   memory, complete ordered observations, and longer wall-clock execution. Cover
   runtime pool exhaustion, partial I/O, disconnects, cancellation while blocked,
   and final acknowledgment. Verify unchanged live behavior and `no_std` builds.

## Impact

The required changes span transport, output construction, lifecycle integration,
and receiver delivery. Most delivery logic should be new offline code; the live
scheduler and its drop behavior remain intact. Paths below are relative to the
repository root; names marked **new** are proposed.

### 1. Transport and protocol

| Location | Required change |
| --- | --- |
| [`core/cu29_logstream/src/stream.rs`](../../core/cu29_logstream/src/stream.rs), `CuStreamTx::try_send` | Relax the immediate-return/no-wait documentation for explicitly offline transports. Preserve signatures and live semantics. Audit callers so a blocking transport is installed through offline wiring only. |
| **New** `components/res/cu29_logstream_tcp/{Cargo.toml,src/lib.rs,src/tests.rs}` | Own TCP connection/listener, bounded frame buffers, length framing, full-frame writes, partial reads, and interruptible waits. Provide a cancellation handle able to shut down the socket without waiting for the writer's lock. |
| **New** `core/cu29_logstream/src/offline.rs` | Define the versioned session header, schema/context exchange, record sequencing, end marker, and final acknowledgment. Bound lengths before allocation; reuse existing record digests. Keep finish/ack control outside `CuStreamTx`, whose successful send only establishes local submission. |

Reuse [`record.rs`](../../core/cu29_logstream/src/record.rs) (`encode_record`,
`decode_record`), [`capture.rs`](../../core/cu29_logstream/src/capture.rs)
(`CaptureDataSet`, `encode_capture_record_into`, `decode_capture`), and
[`copper.rs`](../../core/cu29_logstream/src/copper.rs)
(`encode_copperlist_record_into`). Reuse `ApplicationSchema` from
[`manifest.rs`](../../core/cu29_logstream/src/manifest.rs), but give offline
sessions a header without live FEC/link requirements. No change to saved record
formats or the live `SessionManifest` is intended.

### 2. Sender and generated output wiring

| Location | Required change |
| --- | --- |
| **New** `core/cu29_logstream/src/offline.rs`, output sinks | Implement blocking `WriteStream<CopperList<P>>` and `WriteStream<KeyFrame>` sinks using reusable encoding buffers and one serialized connection owner. Prefer writing on the existing output worker rather than adding another queue/thread. Preserve keyframe/list associations and supply initial reconstruction state before dependent captures. |
| [`core/cu29_derive/src/runtime_macro.rs`](../../core/cu29_derive/src/runtime_macro.rs), `builder_logstream_field`, `builder_with_logstream_method`, `build_with_resources_logstream_param`, `copperlist_output_graph` | Add the offline builder attachment; carry it through builder transitions and construction. Attach offline sinks to the existing local-output fanout, enable their required outputs even without local logging, and select the generated capture encoder. Reject conflicting live/offline attachment in v1. |
| [`core/cu29_logstream/src/lib.rs`](../../core/cu29_logstream/src/lib.rs), its `Cargo.toml`; `core/cu29/{Cargo.toml,src/lib.rs}`, `core/cu29_derive/Cargo.toml`, root `Cargo.toml` | Export the offline entry points, add `std`-requiring feature wiring and the TCP workspace member/dependency. Keep implementation-only exports hidden; account for generated APIs in the V1 contract. |

[`worker.rs`](../../core/cu29_logstream/src/worker.rs) is the behavior to bypass:
`Inbox::submit` drops when buffers run out and `scheduled_sinks` creates the live
sender. Installing TCP through existing `with_logstream` would still use that
path. Leave `SenderCore::accept_record`/`poll` in
[`pacing.rs`](../../core/cu29_logstream/src/pacing.rs) unchanged. Existing
[`fanout.rs`](../../core/cu29_runtime/src/fanout.rs) should compose the blocking
sinks without changing `WriteStream` or adding a delivery-policy enum.

### 3. Runtime completion and replay

| Location | Required change or dependency |
| --- | --- |
| [`core/cu29_runtime/src/app.rs`](../../core/cu29_runtime/src/app.rs) and `runtime_macro.rs` | Prerequisite: the typed `wait_for_output()` capability from #1556 is absent in this checkout. Bring it in before integration; do not use raw runtime access as the public offline API. Retain separate explicit session finish/cancellation handles. |
| [`core/cu29_runtime/src/curuntime.rs`](../../core/cu29_runtime/src/curuntime.rs), `AsyncCopperListsManager::{try_submit,finish_pending}`, synchronous counterpart | Reuse the completion barrier so a blocked sink holds output completion and replay waits before the next iteration. `try_submit` can drop before the barrier runs: validate sufficient CopperList/keyframe capacity at offline construction, including the async reserved free slot. If admission can still drop, fix offline admission rather than accepting the loss. Keep live admission unchanged. |
| Replay caller, e.g. [`examples/cu_logstream_demo/src/resim.rs`](../../examples/cu_logstream_demo/src/resim.rs) | Attach the offline transport, wait after each iteration, and explicitly finish after draining output. Route cancellation to the session before joining output workers, preventing shutdown deadlocks. Start with serial replay; distributed replay is outside v1. |

### 4. Receiver and downstream application

| Location | Required change or reuse |
| --- | --- |
| **New** `core/cu29_logstream/src/offline_session.rs` | Read semantic records directly, validate schema/sequences, decode captures/keyframes, and hand them synchronously to a fallible consumer or a blocking bounded queue. Acknowledge completion after the consumer accepts the final observation. Reuse `LiveTwin::reconstruct` from [`twin.rs`](../../core/cu29_logstream/src/twin.rs) when reconstruction is needed; propagate reconstruction errors. |
| [`twin_session.rs`](../../core/cu29_logstream/src/twin_session.rs), `CuTwinBuilder::spawn`; [`twin.rs`](../../core/cu29_logstream/src/twin.rs), `TwinWorker::send`; [`telemetry.rs`](../../core/cu29_logstream/src/telemetry.rs), `TelemetryPublisher::publish` | These live paths cannot be reused unchanged: they use packet recovery, drop replay work on a full queue, and overwrite unread frames. Bypass them with the offline session/consumer; preserve their live behavior rather than adding blocking switches. |
| [`examples/cu_logstream_demo/src/receiver.rs`](../../examples/cu_logstream_demo/src/receiver.rs) | Add an explicit TCP/offline receiver entry point and a slow-consumer example. |

### 5. Regression coverage and public surface

Add **new** `core/cu29_logstream/tests/offline.rs`, TCP component tests, and
`core/cu29/tests/offline_telemetry.rs`. Cover slow-consumer pressure from receiver
to runtime, minimal pool sizes, keyframe ordering, schema/framing errors,
disconnects, cancellation, and final acknowledgment. Use existing
`core/cu29/tests/logstream_runtime.rs` and `core/cu29_logstream/tests/pacing.rs`
as live regression coverage. Update `api/v1/` snapshots and
[`v1-api-surface.md`](v1-api-surface.md) for intended public additions; run focused
tests with and without `async-cl-io`, feature-disabled/`no_std` checks, and
`just api-check`.

The deliberate tradeoff is that a stalled consumer stalls replay until it resumes,
fails, or the user cancels. The principal review risks are output admission before
the wait, initial reconstruction state, and cancellation while an output worker
owns the connection. These must be resolved before claiming complete delivery.

Related output barrier: [Copper #1556](https://github.com/copper-project/copper-rs/pull/1556).
