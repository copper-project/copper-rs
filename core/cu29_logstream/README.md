# Copper log streaming

`cu29-logstream` gets a robot's execution log to another machine while the robot
is running. Use it to collect logs at a ground station over a link that can lose
packets, or when the robot's onboard log is hard to retrieve.

It sits between Copper's generated runtime and a packet transport:

```text
Robot runtime → logstream sender → UDP / custom transport
                                         ↓
Logreader / replay ← .copper archive ← logstream receiver
```

The sender protects log records with recovery data. The receiver reconstructs
what it can and records explicit gaps for what it cannot. Received archives use
Copper's native format, so your application's normal logreader and replay tools
can read them. Sender work runs off the real-time task path.

## Using it

For a runnable sender/receiver pair, start with the
[UDP demo](../../examples/cu_logstream_demo). Its default `just` command verifies
the received archive against the onboard log and runs the ordinary logreader and
recorded replay. Loss, outage, late-start, receiver-restart, and idle recovery
scenarios are included.

Enable `cu29/logstream` and configure a `log_streaming` destination in the app's
RON config. Bind a transport implementing `CuStreamTx`; the
[`cu29-logstream-udp`](../../components/res/cu29_logstream_udp) resource supplies UDP
sender and receiver endpoints.

On the receiving side, `SessionRouter` discovers the sender's configuration from
its manifest. Feed its events to `NativeArchive<P>`, where `P` is your application's
generated dataset type. The archive checks that the sender's schema matches and
preserves the received payloads and timestamps.

## What to expect

- Recovery uses bounded memory. Packet loss beyond those bounds leaves gaps.
- A late receiver can resume from a verified keyframe, but cannot recover expired
  history. Replay refuses to cross a gap without a matching keyframe.
- Native archival requires the matching application. `NativeArchive` handles
  full captures; `CaptureArchive` retains selective captures for live/offline replay. Use a separate archive path for each sender session.
- Generated senders enforce one bitrate/burst budget across continuous data,
  repairs, and recovery packets. The budget counts Copper packet bytes, excluding
  UDP/IP or other carrier overhead. Replay and recovery share byte-deficit
  scheduling with weights 3:1; unused capacity is available to either lane.
- Manifest and latest complete keyframe/anchor/boundary recovery repeat on a
  250 ms local deadline, with overlapping requests coalesced. A pending bundle
  finishes before a newer one replaces it. New anchors wait for older queued
  source packets to be attempted or expired, preventing avoidable receiver gaps. No new task capture is required.
- Pacing uses `RobotClock` and requires no robot/receiver time synchronization.
  Generated real-link senders select a running clock when application time is
  mocked. Direct driver tests can supply a mock clock.
- Feedback packet traits and static one-way/separate-endpoint adapters are
  available. A duplex resource may implement stream TX and feedback RX on one
  carrier. Feedback protocol, negotiation, and adaptation remain deferred.

Run `just logstream-receiver-check` from the repository root to test reception,
archival, and replay continuity. See the Rust API docs for receiver limits and
event handling.

## Ground-side telemetry

The std-only `telemetry` module provides a single-publisher, single-reader
circular buffer with overwrite-oldest behavior. Feed it the typed value returned
after `NativeArchive::accept()` succeeds; do not decode again or clone payloads.
The transport and archive remain owned by the receiving worker.

```rust,ignore
let (mut publisher, mut reader) = telemetry_channel(capacity, initial_status);
// Receiving worker, after successful archival:
publisher.publish(frame); // Include the frame's session identity in its type.
publisher.set_status(current_status);

// User-owned thread/task:
reader.ready().await; // Or wait_timeout(duration) / register_waker(&waker).
let status = reader.status();
while let Some(update) = reader.try_read() {
    my_widgets.consume(update.frame, update.missed);
}
```

Status uses an independent coalesced slot and a small `Copy` value. Reading it
does not consume frames. Notifications cover unread frames, changed status, and
publisher closure; consume/acknowledge these before waiting again. A registered
waker only schedules work or unparks a thread, never blocks or processes data.

The ring allocates at construction and holds at most its capacity plus one
reader-owned in-flight frame. A borrowed frame stays valid while publication
continues. Payload-owned allocations are additional. Crossbeam queue operations
use atomics and never wait for user processing or free capacity; this host
exchange is not an RT wait-free primitive. Consumer loss is independent of
network/archive gaps. A disconnected reader cannot backpressure recording.
Both still share a process failure boundary.

The [Ratatui demo](../../examples/cu_logstream_demo#native-telemetry-screen)
shows typed robot outputs and a pause control. Live deterministic task
reconstruction and generated mission dispatch are subsequent steps; displaying
captured payloads alone does not recover task state. Run
`just logstream-telemetry-check` for the integration and regression checks.

## Sender storage and lifecycle

`scheduled_sinks` creates one worker owning the transport and FEC state, a pool
of four encoded CL buffers and two encoded keyframe buffers, and fixed packet
storage. Encoding writes directly into these buffers on the existing output
workers. Runtime CopperLists and keyframe capture objects are released before
transmission. Packet staging and recovery retention add bounded copies only on
the background sender path; repeated transmissions borrow the retained packets.

The destination memory budget covers the record pool, continuous encoder, packet
queues, retained packets, and their explicitly counted storage. Thread stacks,
channel/allocator bookkeeping, and RaptorQ's temporary codec allocations are
additional. RaptorQ input is capped by `max_object_bytes`; the scheduled sender
requires that maximum to fit one source block. This is a buffer budget, not a
whole-process allocator ceiling. Unsupported bounds fail during construction.

Four pending CL boundaries and two pending keyframes accommodate independently
ordered output workers. Under sustained skew, oldest pending entries are replaced
and counted; the last complete bundle remains usable. Ordinary data expires after
`max_latency_ms`, measured from encoded-record admission. Retained recovery stays
useful after that deadline and is repeated until replaced or stopped.

Pool exhaustion, packet-queue overflow, expiry, carrier backpressure, and shutdown
shedding are counted. `SenderMonitor` exposes final counters and failure state;
the worker also writes its shutdown statistics and failures to Copper structured
logging. Dropping both sinks stops repetition and drains only until the configured
latency deadline. An independent running RobotClock bounds teardown of a frozen
test clock. Production real-time handoff is unchanged.

`SenderCore` is available without `std`; callers provide `CuTime` from their
RobotClock and drive `poll` themselves. The std driver owns thread wakeups.
Immediate `ContinuousCopperListSink` and `KeyFrameAnchorSink` remain available
for codec tests/custom integration and do not enforce pacing themselves.

Run `just logstream-pacing-check` for scheduler, worker lifecycle, and UDP demo
checks, including recovery after the entire initial bootstrap transmission is lost.

## Live Copper twin

The demo graph is `counter -> sum -> derived`. The ordinary `Derived` Copper task
computes `sum % 256` on the robot and in generated ground-side replay. Its payload
is never transmitted, including repeated anchor boundaries and FEC repairs. The
robot's onboard log contains the full output for comparison. Ratatui explicitly
labels the derived value **reconstructed locally; payload not transmitted**.

Declare the static contract in the same RON used by the robot and ground build:

```ron
(id: "derived", type: "tasks::Derived",
 streaming: (replay: reconstruct, replay_abi: 1)),
```

The task implements `CuCrossPlatformDeterministic` with `REPLAY_ABI = 1`. This is
an explicit promise of deterministic behavior and no external side effects.
Sources and bridge receives stay captured. Reconstruction currently supports
ordinary synchronous tasks using the lossless native compressed codec; background,
anytime, custom codec and selective handle policies are rejected for this path.

A ground station declares `#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]`
and uses its generated `LiveReplay` implementation. Create `TwinWorker::spawn` with
a bounded queue and the existing `TelemetryPublisher`, then process each ordered
router event with:

```rust,ignore
let capture = archive.accept(&event)?; // CaptureArchive<GeneratedDataSet>
twin.accept(&event, capture)?;
```

The archive writes the received native capture bytes, original-presence metadata,
reconstruction policy and omission proofs before handing ownership to replay. It
never stores synthesized outputs or waits for replay/UI consumption. `NativeArchive`
remains the full-capture API; use `CaptureArchive` for this selective view. Existing
native sections retain proofs in `StreamContinuity`; the container format is unchanged.
Manifest version 2 binds the per-output replay ABI. Build matching robot and ground
code; numeric mission dispatch remains separate work.

The worker joins independently arriving keyframes and captured boundaries, restores
state, injects captured messages and executes only reconstructible tasks. It restores
sender metadata before downstream tasks run. Generated simulation does not bind
configured stream transmitters; the twin disables local logging and uses no log file.
The worker retains at most the configured number of queued events and pending
captures, one anchor and one executing frame, plus the presentation ring. Payload
allocations and thread/runtime storage are additional. Source gaps and replay queue
overflow require another matching anchor; UI overwrites only discard display samples.

Per-output digest checking is **off by default**, including debug Rust builds.
Enable `cu29/logstream-verify` (or the demo's `verify-reconstruction`) at development
time to check reconstructed payloads. All digest encoding/hashing happens on the
existing sender output worker and the ground replay worker. No task-path hashing,
serialization, allocation or copying is added. Ordinary packet/anchor integrity
checks remain; they do not independently prove behavioral determinism.

The UI reports Waiting/Recovering, Reconstructed, Verified (developer checks), or
Diverged. Only checked frames are called Verified. With checks enabled, a mismatch
withholds reconstructed frames until another matching anchor. Capture recording
continues through divergence and UI pause. Use `just dashboard-verify` and
`just sender-verify` in separate terminals to run the developer checks.

`just resim` reconstructs the capture archive offline into full native CopperLists.
Run `just logstream-twin-check` at the repository root for the focused checks.
