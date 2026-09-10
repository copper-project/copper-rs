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
can read them. Framing, FEC, and transmission run on background workers.

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

On the receiving side, `SessionRouter` discovers the decoder requirements from
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
  repairs, recovery packets, and structured logs. The budget counts Copper packet bytes, excluding
  UDP/IP or other carrier overhead. Replay, recovery, and structured logs share byte-deficit
  scheduling with weights 3:1:1; unused capacity is available to any ready lane.
- Manifest and latest complete keyframe/recovery point/boundary recovery repeat on a
  250 ms local deadline, with overlapping requests coalesced. A pending bundle
  finishes before a newer one replaces it. New recovery points wait for older queued
  source packets to be attempted or expired, preventing avoidable receiver gaps. No new task capture is required.
- Pacing uses `RobotClock` and requires no robot/receiver time synchronization.
  Generated real-link senders select a running clock when application time is
  mocked. Direct driver tests can supply a mock clock.
- Optional feedback reports receiver health and adjusts future continuous FEC within explicit bounds.
  A duplex resource may share stream TX and feedback RX; one receive owner consumes each logical endpoint.

Run `just logstream-receiver-check` from the repository root to test reception,
archival, and replay continuity. See the Rust API docs for receiver limits and
event handling.

## Structured log records

Configured and injected runtime destinations automatically forward `debug!`,
`info!`, `warn!`, and `error!` entries alongside local logging. The generated
static fan-out copies the bytes produced by the existing local serializer into
one preallocated buffer per destination. Framing, hashing, FEC, and transmission
run on sender workers. Message templates and parameter names are represented by
interned numeric IDs; receivers render text with the producing application's
`cu29_log_index`. Parameter values retain their native types, including strings
when an application explicitly logs a string value.

The sender reserves four structured-record buffers per destination, each capped
at 4096 bytes including the existing record header, or `max_object_bytes` when
smaller. Size overflow, exhausted pools, and stopped destinations increment
`inbox_drops` while the local record remains intact. These buffers and their
queue storage count toward `memory_budget_kib`. The demo uses 576 KiB to cover
its CL/keyframe pools, structured buffers, recovery retention, and packet queues.
A section rollover resets the copied entry before the local serialization retry;
only a successful local write admits the completed entry for transmission.

`SenderCore::accept_record` accepts `RecordKind::StructuredLog` records containing
one native bincode-encoded `CuLogEntry`. Assign increasing object IDs within each
sender session. The scheduler uses the existing structured-log lane and RaptorQ
object framing, shares the destination bitrate/burst limit, and reserves a bounded
packet queue so log traffic cannot fill the replay queue. Object FEC and size bounds
come from the destination's finite-object policy. Expiry and queue shedding count
in the sender's ordinary drop counters. Loss beyond object-FEC coverage leaves
missing entries; the onboard log remains available for complete history.

`SessionRouter` recovers these records independently of CopperList continuity and
emits `SessionEvent::Object` after the session manifest. Pre-manifest packets share
the bounded startup buffer. A 64-record window accepts reordering and suppresses
duplicates; older records are discarded. Structured-object decoder storage uses a
separate instance of the receiver's finite-object limits.

Pass these events to `NativeArchive` or `CaptureArchive` to retain the original
entry bytes in `StructuredLogLine` sections. Archived timestamps, levels, origins,
parameter values, and interned string IDs come from the sender. Entry validation
uses a 4 MiB bincode decode budget. Use the producing application's string index
with `extract-text-log` to reconstruct readable text.

A generated live twin exposes `twin.take_log_reader()` once. Its independent
64-entry ring publishes `ReceivedStructuredLog` after archival succeeds, moving
the already decoded entry into the display channel. Read `update.frame.entry`
and render with `rebuild_logline`; `CuTwinStatus::structured_logs` counts archived
entries. Pausing or dropping either display reader never delays recording.
Custom archive consumers can obtain the same owned value with
`archive.take_structured_log()` immediately after `accept`.

Run `just logstream-structured-check` for binary preservation, allocation, local
section rollover, loss/reordering, shutdown, generated wiring, and telemetry UI
coverage.

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
of four encoded CL buffers, two encoded keyframe buffers, four structured buffers, and fixed packet
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
shedding are counted. `SenderMonitor::snapshot()` exposes live counters and feedback state at 10 Hz;
the worker also writes its shutdown statistics and failures to Copper structured
logging. Dropping both sinks stops repetition and drains only until the configured
latency deadline. An independent running RobotClock bounds teardown of a frozen
test clock. Structured logging adds the bounded byte copy described above; CL and keyframe handoffs retain their existing behavior.

`SenderCore` is available without `std`; callers provide `CuTime` from their
RobotClock and drive `poll` themselves. The std driver owns thread wakeups.
Immediate `ContinuousCopperListSink` and `RecoveryPointSink` remain available
for codec tests/custom integration and do not enforce pacing themselves.

Run `just logstream-pacing-check` for scheduler, worker lifecycle, and UDP demo
checks, including recovery after the entire initial bootstrap transmission is lost.

## Live Copper twin

The demo graph is `encoders -> kinematics`. The `Kinematics` Copper task computes
elbow and fingertip positions from shoulder/elbow angles on the robot and in
generated ground-side replay. Its payload is omitted from captures, including
repeated recovery boundaries and FEC repairs. The robot's onboard log contains
the full output for comparison. The telemetry screen labels the arm pose as
**reconstructed locally; payload not transmitted**.

Declare the static contract in the same RON used by the robot and ground build:

```ron
(id: "kinematics", type: "cu_logstream_demo::tasks::Kinematics",
 streaming: (replay: reconstruct)),
```

The task implements `CuCrossPlatformDeterministic`. This is
an explicit promise of deterministic behavior and no external side effects.
Sources and bridge receives stay captured. Reconstruction currently supports
ordinary synchronous tasks using the lossless native compressed codec; background,
anytime, custom codec and selective handle policies are rejected for this path.

A ground station declares `#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]`.
The generated application exposes a twin builder:

```rust,ignore
let (mut twin, mut frames) = Ground::twin(rx)
    .with_log_path("logs/received.copper")
    .spawn()?;

// On the UI or analysis thread:
frames.wait_timeout(std::time::Duration::from_millis(50));
while let Some(update) = frames.try_read() {
    render(&update.frame.copperlist);
}
let status = twin.stop()?;
```

`rx` is any `CuStreamRx`, such as the receive half of a UDP resource. Copper owns
session routing, native recording, the bounded replay worker, status publication,
and shutdown. The caller owns the frame reader and presentation. Pausing or dropping
that reader never blocks recording. Dropping the twin stops and joins its workers;
`stop()` also reports receiver errors and final counters. `archive_only()` records
without running a twin. Each handle accepts one sender session and a fresh log path.
The default receiver supports the 1200-byte-MTU, 64-symbol streaming profile, with
4 KiB records and 64 KiB recovery objects. It retains 32 replay events, 32 pending
captures, one recovery point, one executing frame and 64 display frames; payload storage and
thread/runtime allocations are additional. `with_frame_capacity` changes display retention.

Production sends the native CopperList format with selected payloads omitted.
CopperLists carry `id` followed by `msgs`, without runtime lifecycle state.
There are no transmitted version fields in packets, records, RLC fragments, or
session manifests. Always use the receiver/logreader built for the producing
application version. Archived unified logs retain encapsulation version **1**;
that version describes file/section layout only, never encoded content.
The compressed metadata bytes are unchanged by moving ULEB128 timestamp-delta
and backreference encoding into `cu-bincode` 2.1.

Headers are packed without reserved alignment bytes or a redundant fixed header
length field. All multi-byte header fields use big endian encoding:

| Layer | Header fields, in wire order | Bytes |
| --- | --- | ---: |
| RLC source packet | magic (4), lane (1), record kind (1), FEC scheme (1), symbol kind (1), session ID (16), sender ID (4), source payload ID (4) | 32 |
| RLC repair packet | magic (4), lane (1), record kind (1), FEC scheme (1), symbol kind (1), session ID (16), sender ID (4), repair payload ID (8) | 36 |
| RaptorQ packet | magic (4), lane (1), record kind (1), FEC scheme (1), symbol kind (1), session ID (16), sender ID (4), object ID (8), compact OTI (11), payload ID (4) | 51 |
| Record | magic (4), kind (1), object ID (8), BLAKE3 digest (32) | 45 |
| RLC fragment | magic (4), object ID (8), record length (4), fragment index (4) | 20 |

Compared with the original headers, this saves 40 bytes per RLC source packet,
36 per RLC repair packet, 21 per RaptorQ packet, 11 per record, and 12 per RLC
source fragment, plus the manifest savings described below.
Packet sequence counters are not transmitted; recovery and deduplication use
FEC symbol identifiers and record identities. RLC packets omit the outer object ID
and fragment count; protected source fragments carry record identity, record
length, and fragment index. Receivers derive fragment count and payload length
from that geometry and the configured symbol capacity. Fragment kind is implicitly
CopperList; the reassembled record must still match that kind and identity and
pass digest verification. Repairs span a window of fragments. Only the active
RLC FEC ID bytes are transmitted.
RaptorQ OTI omits the reserved zero byte at index 5 of the library's 12-byte
representation. The wire carries transfer length (5), symbol size (2), source
blocks (1), sub-blocks (2), and alignment (1). Decoding restores the zero byte
before receiver geometry validation and RaptorQ decoding. Encoding rejects a
nonzero reserved byte instead of silently discarding it.
Record payload length is derived from the complete reassembled record extent,
saving eight bytes per record. The BLAKE3 input remains kind, object ID, derived
payload length as a big endian u64, and payload. Recovery-point references retain
the same digests for the same semantic records. Truncated headers are rejected;
truncated payloads and appended bytes fail digest verification. Receiver allocation
bounds still apply to the complete framed record before assembly.
Session manifests encode identity, `ReceiverRequirements`, and application schema
using standard bincode encoding. Requirements carry only symbol size, RLC field,
window symbols, maximum complete CopperList record bytes, and optional feedback
requirements (report interval and destination key). Receivers validate
this geometry and enforce their own symbol, window, record, and buffering limits
before constructing a decoder. RaptorQ geometry comes from packet OTI and remains
bounded by receiver-local finite-object limits.
Destination ID, MTU, bitrate, sender memory budget, latency, burst allowance,
repair cadence/density/count, recovery interval, and sender object bounds stay in
`LogStreamPlan`; they are not repeated in manifests. For the `ground` test profile
(1128-byte symbols, GF(256), window 64, 65536-byte records), requirements occupy
11 bincode bytes instead of the 40-byte sender plan, saving 29 bytes per manifest
in addition to the previously removed version byte. Savings vary with sender
policy values and destination-name length. Schema strings and reconstruction ABI
checks remain intact. The record digest still binds the exact manifest bytes;
recovery points reference that digest. Sender-only policy changes with identical
requirements, identity, and schema now produce identical manifest records.
Savings inside record and fragment headers free symbol payload capacity and can
reduce fragment counts. Packet-header savings directly shorten each datagram.
Existing symbol storage capacity is unchanged: a 1200-byte MTU uses at most 1128-byte symbols,
producing RLC source packets up to 1160 bytes, RLC repair packets up to 1164 bytes,
and RaptorQ packets up to 1179 bytes. `PACKET_HEADER_LEN` is the maximum header
length for buffer sizing; encoding returns the exact length for each packet.
Shared symbol sizing still reserves room for the largest (RaptorQ) header.
Packet payload length is derived from the complete packet extent supplied by
`CuStreamRx`; serial/transparent-radio adapters must frame the byte stream into
complete packets before decoding. The packet header carries no payload length.
Both data packets and feedback reports require transport-provided packet integrity;
neither carries an inner CRC. Delivery, ordering, and uniqueness are not guaranteed.
UDP supplies integrity through the network stack. Raw serial supplies neither packet
boundaries nor integrity. The framing adapter between LogStream and raw serial
appends a four-byte big endian CRC32C before delimiter escaping, then verifies and
strips it before delivery.
The adapter discards damaged frames and resynchronizes at the next delimiter, turning
corruption into packet loss before FEC. Both serial peers must use this framing;
the older adapter that relied on the common CRC is incompatible.
Carrier framing/checksum overhead is outside the configured packet MTU and must
be included when sizing adapter buffers and budgeting physical-link throughput.
Direct decoder callers must supply complete, carrier-verified packets too.
Record BLAKE3 digests and recovery-point digest references remain unchanged;
these bind reconstructed content and do not replace carrier integrity checks.

The native codec already carries original/captured presence. There is no proof envelope,
per-list verification allocation, or new continuity record. The archive writes the
received native bytes before replay and never stores synthesized outputs. The
session manifest binds the reconstruction ABI to the graph; it is not a content
version or a substitute for the matching application decoder.
Packet framing, FEC and recovery from a recovery point are unchanged.

Copper restores keyframes, injects captured inputs, executes reconstructible tasks and
restores sender metadata before downstream tasks run. Existing source gaps and replay
queue overflows require a matching recovery point. These continuity checks are separate
from checking whether deterministic task code produced the right result. The generated
ground runtime disables its own logging and transport transmitters; its archive is owned
by the twin receiver.

Reconstruction correctness checks are **entirely opt-in**, even in debug Rust builds.
Enable `cu29/logstream-verify` (the demo calls it `verify-reconstruction`) on both ends
for development. Only this feature compiles in hashing and a fixed 32-byte digest trailer
covering the omitted outputs and their payload presence. Hashing runs on the existing
sender output worker and the ground replay worker, using borrowed payloads with no
intermediate allocation. Production captures have no trailer or digest storage. A normal
receiver rejects debug trailers explicitly; a verification receiver also accepts normal
captures and labels them Reconstructed, never Verified.

Only debug captures that pass comparison are labeled Verified. A mismatch suppresses
reconstructed frames until the next matching recovery point; native recording continues. Debug
digests are consumed live and do not add archive sections or change offline log readers.
Use `just telemetry-verify` and `just sender-verify` to run the development checks.
`just resim` reconstructs ordinary capture archives offline; the demo's verification
command compares the result against the full onboard log.

Run `just logstream-twin-check` for allocation/native-format checks, both verification
modes, worker lifecycle, reader isolation, and the UDP loss/recovery/replay scenarios.

## Optional receiver feedback

Add this to a `log_streaming.destinations` entry; the referenced resource must implement `CuFeedbackRx`:

```ron
feedback: (
    transport: (type: "cu29_logstream_udp::CuUdpLogStreamRx", resource: "network.rx"),
    report_interval_ms: 500,
    timeout_ms: 2000,
    adaptation: (
        min_repair_every_source_symbols: 1,
        max_repair_every_source_symbols: 16,
    ),
),
```

On the receiver, explicitly supply the return transmitter with `Twin::twin(rx).with_feedback(tx)`.
Configure its destination address in the resource, not from incoming traffic. Standalone receivers can
use `FeedbackReporter` with `SessionRouter::feedback_counters`. Programmatic senders use
`scheduled_feedback_sinks(SeparateFeedback { tx, feedback_rx }, config, clock)`.

Omitting feedback preserves one-way operation. The unversioned manifest advertises optional feedback
capability, destination key, and report cadence. Timeout and adaptation bounds remain in the local
sender configuration. Reports are also unversioned and require the matching
application decoder. Reports carry a `CUFB` prefix followed by the fixed-integer bincode payload,
with integrity supplied by the transport under the same contract as data packets. Reports carry
cumulative counters, receiver identity/sequence, finalized source outcomes,
receiver progress/pressure, and an optional request for the latest retained recovery bundle. No data ACKs.

Omit `adaptation` for reports only. Otherwise the existing repair interval is the startup/fallback baseline
and must lie between the explicit bounds. Lower intervals mean more repairs. Finalized loss is smoothed
with weight 1/4 for the new sample; the target repair/source ratio covers estimated loss plus two percentage
points. Unrecovered symbols halve the interval, bounded by the target and minimum. Three healthier reports
allow one step toward less redundancy. Reports without finalized progress do not reduce protection.

A receiver binds on its first valid report; another can bind after timeout. Invalid, duplicate, reordered,
or excessive reports cannot refresh health. After timeout, state is stale and the interval moves one step
per report period toward baseline. Bitrate, burst, latency, memory, FEC window/field/density, and object FEC
stay fixed. Feedback failure never stops capture or autonomous recovery; snapshots retain failure state.

Loss excludes the active coding window and unseen history/tails. Reports and adaptation stay on stream workers. Run `just logstream-feedback-check`.

### TUI bandwidth panel

The console monitor's BW tab includes `Telemetry / TX` panels for each configured destination.
One-way panels show actual submitted Copper bytes/packets, the configured budget, drops by cause,
recovery activity, and baseline FEC. These are local submissions, not delivery acknowledgements.

Two-way panels additionally show feedback state/age, receiver throughput, finalized source loss and
FEC recovery, receiver buffer/progress counters, and the effective repair interval. Waiting, stale,
and failed feedback show `n/a` for receiver measurements. Arrow keys or `hjkl` scroll the BW content
horizontally and vertically on small terminals or when several destinations are configured.

Generated apps attach read-only worker handles through `CuMonitoringRuntime`. Console and Bevy monitors
pass those handles to the shared TUI model; custom monitor frontends can use `runtime.log_streams()`.
Snapshots are published on sender workers and sampled by the presentation thread. The task path gains
no monitoring callbacks, serialization, or synchronization. Run `just logstream-monitor-check`.
