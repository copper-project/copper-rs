# UDP log streaming demo

Run a deterministic Copper graph in one process and collect its native execution
log in another over localhost UDP. The graph is `counter → sum → derived`. Counter and sum are transmitted; derived
is reconstructed live from the same Copper task implementation.

From this directory:

```sh
just
just run loss
just run outage
just run late
just run restart
just run idle
```

`just` builds the tools, runs the clean scenario, compares the received archive
against the onboard archive, runs the ordinary logreader's `fsck`, and replays
the received CopperLists. Python 3 orchestrates process lifetimes; Rust reads
and verifies the native logs. Each run creates a fresh directory under this
example's `logs/` and prints its path.

| Scenario | What it exercises |
| --- | --- |
| `clean` | All 256 CopperLists match the onboard payloads and metadata. |
| `loss` | Discard the source packets for CopperList 20; RLC repairs recover it with no semantic gap. |
| `outage` | Discard a contiguous arrival interval beginning at CopperList 32 and ending before 160, including control traffic. Missing history remains explicit and a verified anchor enables recovery. |
| `late` | A separate probe receiver archives through CopperList 64 and exits; a fresh receiver then starts with no prior session state. Its archive contains a `LateJoin` prefix gap. |
| `idle` | Drop all traffic for the first 300 ms, produce only CL0, and keep the sender alive with no new captures. Periodic recovery restores the complete one-record archive. |
| `restart` | Close the first receiver after CopperList 64, leave it offline briefly, and launch a new process while the sender continues. The new archive reports unavailable history. |

Loss injection runs at the receiver's packet boundary after real UDP reception,
before FEC decoding. It preserves arrival order and adds no sender task work.
The source-loss and outage intervals are selected by wire record IDs. Late start
and restart use real process lifetimes. Late start waits for observed stream
progress instead of a launch-time sleep, which could join while CopperList 0
is still recoverable from retained history. Exact recovery IDs still depend on
host scheduling; verification checks the resulting continuity.

The headless status display reports received packets, the latest archived
CopperList, latest verified anchor, gap count, and demo packet drops. Received
files have separate paths for each receiver lifetime.

## Native telemetry screen

From this directory, start `just dashboard`, then `just sender` in another
terminal. The manual sender runs for about a minute; automated scenarios keep
256 iterations. Choose fresh log paths when repeating a run:

```sh
just dashboard 127.0.0.1:7447 logs/dashboard-2.copper
# In another terminal:
just sender 127.0.0.1:7447 logs/sender-2.copper
```

The Ratatui screen shows counter/sum values, a counter chart, packet and frame
age, archive progress, verified anchors, source gap ranges, and reader overwrites.
**Space** pauses consumption; wait a second and resume to see missed samples
while the archive count keeps advancing. **q**, Escape, or Ctrl-C closes the
receiver and finalizes its archive. The sender is a separate process. After the
sender finishes, the screen remains open until you quit.

The receiver moves the already decoded, successfully archived CopperList into a
64-frame circular buffer. The UI pulls on its own thread through
`cu29_logstream::telemetry::telemetry_channel`. It waits for a payload-free wake
with a 50 ms keyboard/age timer; no UI callback runs on the receiving thread.
On overrun it resumes at the oldest retained frame with an exact local missed
count. Source gaps and reader misses are distinct. Status stays available while
the UI is paused. The UI owns its bounded chart history and uses generated
`get_counter_output()` / `get_sum_output()` accessors.

This step displays captured outputs; it does **not** yet execute a live
Copper runtime on the ground to deterministically reconstruct omitted outputs
or task state. That next step must feed this same archive/pull boundary after
keyframe restore, ordered execution, and verification. This demo has one mission;
numeric mission dispatch remains a separate protocol/codegen milestone.

## Run the processes yourself

Start the receiver in one terminal, then the sender in another:

```sh
just receiver
just sender
```

These use `127.0.0.1:7447` and write `logs/received.copper` and
`logs/sender.copper`. Use fresh output paths for another run:

```sh
just receiver 127.0.0.1:7447 logs/received-2.copper
just sender 127.0.0.1:7447 logs/sender-2.copper
```

The manual sender runs 6000 iterations at roughly 100 Hz; the final `just sender`
argument selects another count. The receiver exits after one
second without a datagram, or fails if no traffic arrives within 15 seconds.
Socket addresses are explicit CLI arguments; stream policy and static sender
resource binding are in `copperconfig.ron`.

## Read and replay

Use the printed log base from an automated run with these recipes:

```sh
just cl logs/received.copper
just fsck logs/received.copper
just resim logs/received.copper logs/replay.copper
just resim-debug logs/replay.copper logs/debug-replay.copper
```

The replay binary uses Copper's standard `--log-base`, `--replay-log-base`, and
`--debug-base` contract. Remote debug creates separate replay outputs per session.
Offline replay injects captured outputs and reconstructs the omitted derived output.
The verification tool compares full reconstructed outputs and sender metadata against
the onboard log.
Offline replay drains pending output before each iteration and aligns generated
CopperList IDs at recovery boundaries. Production task execution remains nonblocking.
The sender/replay slabs are 16 MiB to accommodate the runtime's existing 10 MiB
native keyframe sections.

## Check the milestone

From the repository root, run `just logstream-demo-check`. This checks Clippy,
builds the opt-in binaries, and exercises all six scenarios. `just check` in this
directory runs all scenarios. Normal workspace builds leave streaming disabled;
`demo` enables it and `replay` also enables the remote-debug replay tools.

The configured 2 Mbps budget includes Copper packet headers and FEC/recovery
traffic, excluding UDP/IP overhead. A bounded background sender enforces the
bitrate, eight-packet burst allowance, and 250 ms ordinary-data queue deadline.
Manifest and complete recovery bundles repeat on a 250 ms local deadline while
the sender remains alive, including when application time is paused. The demo's
mock application timestamps stay deterministic; physical pacing uses a running
RobotClock. Receiver time synchronization is not required.

Run `just logstream-pacing-check` at the root for deterministic rate/burst,
overload, and worker lifecycle checks alongside these scenarios. The sender
buffer budget excludes thread stacks, channel/allocator bookkeeping, and bounded
RaptorQ scratch allocations; see the [sender docs](../../core/cu29_logstream).
Memory-bounded recovery cannot recover expired history, and receiver shutdown
does not establish the sender's unobserved tail. Full-capture native archival
requires the matching application schema. Feedback, mission dispatch, and serial remain later milestones. The host telemetry buffer and optional `tui` feature are available
now. Run `just logstream-telemetry-check` at the root for notification/overrun
tests, archive comparisons with fast/stalled/disconnected readers, terminal
rendering, and all existing loss/recovery/replay scenarios. Feedback interfaces
permit a shared bidirectional carrier or separate explicitly configured endpoints.

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
