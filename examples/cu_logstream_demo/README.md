# UDP log streaming and live twin demo

Get a robot's execution log onto a ground station while it runs, even over a
lossy link. The received `.copper` archive works with the application's usual
logreader and replay tools, so you can inspect a run without retrieving onboard
storage. Streaming and packet recovery run on background workers.

This example also demonstrates a **live Copper twin**: the ground station runs
selected deterministic tasks from the same application to reconstruct outputs
that were never transmitted. The graph is `encoders → kinematics`;
shoulder and elbow angles travel over UDP, while elbow and fingertip positions
are reconstructed locally by the same kinematics task.
For an application, this trades ground-side computation for less payload traffic
and keeps the display and offline analysis tied to the robot's task code.

Recovery is bounded: repair packets can recover short losses, but longer outages
leave explicit gaps. A receiver can join or restart at a later keyframe (a task
state snapshot); it cannot retrieve expired history. A paused or slow display
can miss frames without interrupting archival. See the
[log streaming overview](../../core/cu29_logstream/README.md) and
[live twin contract](../../core/cu29_logstream/README.md#live-copper-twin) for details.

## Try it

From this directory:

```sh
just dag             # Show the task graph
just                 # Clean run, archive comparison, fsck, and offline replay
just run loss        # Repair a dropped CopperList
just run outage      # Resume after a longer interruption
just run late        # Join an already running sender
just run restart     # Restart the receiver during a run
just run idle        # Recover even after captures stop
```

Each automated run creates a fresh directory under this example's `logs/` and
prints its path. Python 3 coordinates the processes; Rust verifies the received
payloads, reconstructed outputs, metadata, and original structured entries against the onboard log.

## Native telemetry screen

Start `just telemetry`, then `just sender` in another terminal. The sender runs
for about a minute. The screen shows captured shoulder/elbow angle traces and a
robot arm drawn from locally reconstructed kinematics outputs. Its fingertip trail
clears across gaps. **1** selects Live, **2** selects Health, and **Tab** cycles
between them; arrow keys or **hjkl** scroll Health details.

The **Robot logs · received over UDP** pane displays an `info!(ctx, ...)` entry
from the robot once per second: `Simulated encoder health: temperature_c=… supply_mv=…`.
These simulated temperature and supply-voltage diagnostics are carried only by
structured logs; the task message carries joint angles.
Only interned IDs and numeric values cross the link for this statement. The
telemetry process reconstructs text with `cu29_log_index` beside its executable;
`--log-index <path>` selects the producing build's index when running elsewhere.
The received archive retains the original binary entries and task origin.
Log display has its own bounded ring and missed-entry counter; **Space** pauses
both display readers while recording continues. On very small terminals, the
compact view prioritizes robot and recording status.

The sender opens Copper's native task monitor with DAG, latency, bandwidth, and
memory tabs. Automated scenarios remain headless.

**Space** pauses the view; resume after a second to see missed display samples
while recording continues. Network gaps and display misses have separate counters.
**q**, Escape, or Ctrl-C closes the telemetry and finalizes its archive; it stays
open after the sender finishes.

Sender, receiver, and telemetry replace logs at the selected base on each run.
Choose different paths to retain earlier runs:

```sh
just telemetry 127.0.0.1:7447 logs/telemetry-2.copper
# In another terminal:
just sender 127.0.0.1:7447 logs/sender-2.copper
```

For a headless receiver, substitute
`just receiver 127.0.0.1:7447 logs/received-2.copper`. It exits after one second
without a datagram, or fails if no traffic arrives within 15 seconds. The final
`just sender` argument optionally changes the default 6000 iterations.

## Use it in your application

The reusable integration is the configured sender plus the generated twin
builder. The impairment wrapper, scenario launcher, and comparison code exercise
failure modes; they are demo machinery.

1. **Configure the sender.** Enable `cu29/logstream` and add
   `cu29-logstream-udp`; see [Cargo.toml](Cargo.toml). Adapt the `logging`,
   `resources`, and `log_streaming` blocks in [copperconfig.ron](copperconfig.ron)
   to your graph and network. Set the destination address and size the link,
   record, and recovery budgets for your payloads. `recovery_interval` counts
   CopperLists and must be a nonzero multiple of `logging.keyframe_interval`.
   Keep your normal application clock and execution loop; the mock clock and
   fixed iteration loop in [src/lib.rs](src/lib.rs) make this demo repeatable.
2. **Build the ground runtime from the same graph and task types.** Follow
   `mod twin` in [src/lib.rs](src/lib.rs), using
   `#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]`.
   Open a UDP receive endpoint and pass it directly to the generated builder:

   ```rust,ignore
   use cu29_logstream_udp::CuUdpLogStreamConfig;

   // Twin is your generated simulation application type.
   let (_, rx) = CuUdpLogStreamConfig::new("0.0.0.0:7447".parse()?).open()?;
   let (mut twin, mut frames) = Twin::twin(rx)
       .with_log_path("logs/received.copper")
       .spawn()?;
   ```

   Copper owns reception, recovery, recording, and replay workers. Keep the
   handle alive; call `twin.stop()?` at shutdown to finalize the archive and
   report errors. Use a fresh archive path per sender session.
3. **Consume frames in your UI or analysis loop.** Use `frames.wait_timeout(...)`,
   `frames.try_read()`, and `frames.status()` as in
   [src/telemetry.rs](src/telemetry.rs). Read typed outputs through generated
   accessors such as `get_encoders_output()` / `get_kinematics_output()`, and account for `update.missed`.
   The display retains 64 frames by default; `.with_frame_capacity(...)` changes
   that bound. For recording alone, use `.archive_only()` before `.spawn()`.
4. **Optionally omit reconstructible outputs.** Keep outputs captured initially.
   To save payload bandwidth, mark suitable tasks with
   `streaming: (replay: reconstruct, replay_abi: 1)` and implement
   `CuCrossPlatformDeterministic` with matching `REPLAY_ABI`, as
   [Kinematics](src/tasks.rs) does. This promises deterministic behavior across
   platforms and no external side effects. Sources and bridge receives stay
   captured; reconstruction currently supports ordinary synchronous tasks with
   the lossless native compressed codec.

Skip `ImpairedRx`, `ImpairmentStats`, readiness files, scenario stop conditions,
[run.py](run.py), and the demo's `verify` command when integrating. Ratatui and
[src/telemetry.rs](src/telemetry.rs) are optional presentation code. Retain
Copper's FEC (forward error correction) and recovery configuration: those handle
real packet loss.

The demo's 2 Mbps limit includes Copper headers and repair/recovery traffic,
excluding UDP/IP overhead. Memory limits bound sender buffers, not total process
memory. Review [sender storage](../../core/cu29_logstream/README.md#sender-storage-and-lifecycle)
and [receiver limits](../../core/cu29_logstream/README.md#live-copper-twin) before
scaling payloads. This demo covers one mission over UDP; feedback, mission
dispatch, and serial transport remain future work.

## Read, replay, and verify

Use a received log base (or the path printed by an automated run):

```sh
just cl logs/received.copper
just fsck logs/received.copper
just resim logs/received.copper logs/replay.copper
just resim-debug logs/replay.copper logs/debug-replay.copper
```

The received archive stores captured data; reconstructed outputs appear in live
frames and replay output, not in that archive. Replay needs the matching
application schema and a matching keyframe to resume across a gap. Remote debug
uses Copper's standard replay CLI and creates separate outputs per session.

To check reconstruction against the sender live, run `just telemetry-verify`
and `just sender-verify` in separate terminals. These enable
`cu29/logstream-verify` on both ends: optional digest traffic and comparison work
label matching frames **Verified**. Ordinary runs label them **Reconstructed**.
A mismatch suspends reconstructed frames until the next matching recovery point;
recording continues.

For regression checks, `just check` runs all six scenarios. From the repository
root, `just logstream-demo-check` also builds and runs Clippy;
`just logstream-twin-check` adds twin and verification coverage. Streaming is
opt-in: this crate's `demo`, `tui`, and `replay` features enable the corresponding
binaries and tools.
