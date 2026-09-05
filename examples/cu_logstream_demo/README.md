# UDP log streaming demo

Run a deterministic Copper graph in one process and collect its native execution
log in another over localhost UDP. The graph is `counter → accumulator`: both
tasks freeze their state, and every output retains the sender's timestamps.

From this directory:

```sh
just
just run loss
just run outage
just run late
just run restart
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
| `late` | Start the receiver after the sender is already running. Its archive contains a `LateJoin` prefix gap. |
| `restart` | Close the first receiver after CopperList 64, leave it offline briefly, and launch a new process while the sender continues. The new archive reports unavailable history. |

Loss injection runs at the receiver's packet boundary after real UDP reception,
before FEC decoding. It preserves arrival order and adds no sender task work.
The source-loss and outage intervals are selected by wire record IDs. Late start
and restart use real process lifetimes, so their exact recovery IDs depend on
host scheduling. Verification checks the resulting continuity rather than
assuming those IDs.

The status display reports received packets, the latest archived CopperList,
latest verified anchor, gap count, and demo packet drops. Task data stays in the
native log. Received files have separate paths for each receiver lifetime.

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

The sender runs 256 iterations at roughly 100 Hz. The receiver exits after one
second without a datagram, or fails if no traffic arrives within 15 seconds.
Socket addresses are explicit CLI arguments; stream policy and static sender
resource binding are in `copperconfig.ron`.

## Read and replay

Use the printed log base from an automated run with these recipes:

```sh
just cl logs/received.copper
just fsck logs/received.copper
just resim logs/received.copper logs/replay.copper
just resim-debug logs/received.copper logs/debug-replay.copper
```

The replay binary uses Copper's standard `--log-base`, `--replay-log-base`, and
`--debug-base` contract. Remote debug creates separate replay outputs per session.
Recorded replay preserves captured outputs and checks missing CopperList ranges
against keyframes before advancing. It does not perform hybrid reconstruction.
The replay output is compared with the received CopperLists byte for byte.
Offline replay drains pending output before each iteration and aligns generated
CopperList IDs at recovery boundaries. Production task execution remains nonblocking.
The sender/replay slabs are 16 MiB to accommodate the runtime's existing 10 MiB
native keyframe sections.

## Check the milestone

From the repository root, run `just logstream-demo-check`. This checks Clippy,
builds the opt-in binaries, and exercises all five scenarios. `just check` in this
directory runs all scenarios. Normal workspace builds leave streaming disabled;
`demo` enables it and `replay` also enables the remote-debug replay tools.

This is a low-rate demonstrator. The configured bitrate, burst, and latency
policy are not yet enforced by a pacer. Manifests and anchors are emitted on
qualifying keyframe captures; independent retained-object repetition is deferred.
Memory-bounded recovery cannot recover expired history, and receiver shutdown
does not establish the sender's unobserved tail. Full-capture native archival
requires the matching application schema. Feedback, hybrid reconstruction, the
live twin API, and serial are later milestones.
