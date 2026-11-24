# A customer telemetry protocol over UDP

This example connects a Copper robot to its live twin with one record per UDP
datagram. Copy the two implementations in [src/lib.rs](src/lib.rs) to start your
own protocol. The robot implements `CuTelemetryTx`. The ground station implements
`CuTelemetryRx`. Both use Copper’s native capture and keyframe payloads.

Run the loopback example from the workspace root with a fresh output directory.

```sh
cargo run -p cu-udp-telemetry --features host -- target/plain-udp-run
```

It starts a ground station on an ephemeral port, runs 48 robot iterations, and
checks the final reconstructed arm pose is present. The output directory contains
the robot and ground archives. [src/main.rs](src/main.rs) shows where to split
the two processes. On the robot call
`with_telemetry_tx(move |_| Ok(PlainUdpTx::new(tx)), buffers)`.
On the ground call `Twin::from_telemetry(PlainUdpRx::new(rx))`.
Use the same graph and encoding features on both ends.

## Packet layout

| Bytes | Contents |
| --- | --- |
| 0–3 | ASCII `CUT1` |
| 4–19 | Sender’s 16-byte session ID |
| 20 | Kind (0 session, 1 CopperList, 2 keyframe, 3 structured log) |
| 21–28 | Big-endian u64 ID (zero for session, cycle for list/keyframe, sequence for log) |
| 29 onward | Complete native payload, or standard bincode session descriptor |

The 1200-byte cap leaves 1171 bytes per record. An oversize record is an error.
Choose a cap suitable for your link or implement fragmentation for larger records.
The protocol sends the session description at startup and before each keyframe.
The listener binds to the first valid session and discards duplicates, old
records, foreign sessions and unknown packet kinds. It treats a jump in cycle ID
as definitive loss. A delayed older list is discarded. A missing final datagram
is observable only if a later cycle arrives. A full socket drops the attempted
datagram. Delivery uses UDP checksums and has no repair or feedback traffic.

## Embedded and host checks

The protocol library is `no_std`. Its fixed packet buffers live in each endpoint.
Session decoding uses allocation bounded by a bincode decode limit. Provide an
embedded datagram endpoint implementing `CuStreamTx` or `CuStreamRx` to use the
same packet code. The socket example and generated twin run on a host.

```sh
cargo check -p cu-udp-telemetry --lib --no-default-features --target thumbv8m.main-none-eabihf
cargo test -p cu-udp-telemetry --features host
```

The loopback regression withholds cycle 17, checks the archive’s missing cycle,
and verifies the twin produces the final frame after the keyframe at cycle 32.

## Send to two telemetry destinations

Pass both protocols to the same builder slot. The worker hands each branch
the same session, capture, keyframe and structured-log bytes. The FEC sender
constructs its manifest from the session and uses its own packet pacing.

```rust
use cu29_logstream::{FecTelemetryTx, TelemetryBufferConfig};
use cu29_logstream::protocol::{BestEffort, Fanout};

let buffers = TelemetryBufferConfig {
    capture_count: 1,
    keyframe_count: 1,
    structured_log_count: 1,
    ..TelemetryBufferConfig::new(MAX_PAYLOAD, MAX_PAYLOAD)
};
let app = Robot::builder()
    .with_telemetry_tx(
        move |clock| {
            let fec = FecTelemetryTx::new(fec_transport, fec_plan, clock)?;
            let udp = BestEffort::new(PlainUdpTx::new(udp_transport), report_udp_error);
            Ok(Fanout(fec, udp))
        },
        buffers,
    )
    .build()?;
```

With 1171 bytes per record, the example reserves 3513 bytes for payloads
across both destinations. The default counts reserve 11,710 bytes. Queue
metadata and protocol-owned storage are additional. A new record drops when
all buffers of its kind are busy.

Resolve `fec_plan` with `LogStreamPlan::resolve` from the FEC destination.
Choose shared capture and keyframe limits that both protocols can carry.
`Fanout` attempts both outputs and propagates the first error. `BestEffort`
can wrap either destination to report its first error and disable that output. Sends, polls and
the error handler share a worker and must all be nonblocking.

## Threadless telemetry

Run the same 48-cycle robot and ground twin with the threadless sender
over loopback UDP.

```sh
cargo run -p cu-udp-telemetry --features threadless -- target/threadless-udp-run
cargo test -p cu-udp-telemetry --features threadless
```

Enable `cu29/telemetry` on the serial firmware build. The same feature
works on a host built with synchronous CopperList I/O. The `cu29/logstream`
feature selects the threaded host sender.

```rust
let robot = Robot::builder()
    .with_telemetry_tx(
        boot_session_id,
        move |_| Ok(PlainUdpTx::new(endpoint)),
        TelemetryBufferConfig::new(MAX_PAYLOAD, MAX_PAYLOAD),
    )
    .build()?;
let mut robot = robot.start()?;
while running() {
    robot.run_one_iteration()?;
}
drop(robot.stop()?);
```

Supply a unique 16-byte session ID for each construction. After a successful
task-graph iteration, generated code delivers up to four pending native
records and polls the protocol once. This budget accommodates a session, a
keyframe, a CopperList and a structured log. Protocol implementations control
the number of datagrams per record and must bound both callbacks. Include that
time in the loop period.

The sender reserves one active CopperList, one complete dataset per capture
slot, spare keyframe storage sized during task startup, a shared encoding
buffer, and fixed structured-log buffers. Full queues drop new output while
the control loop continues. Firmware copies structured-log bytes as the local
encoder writes them. It transfers CopperLists and keyframes by ownership.

With an `AppRunner`, register a sender factory on the runner builder. The generated managed loop uses the same service pass before the rate limiter. The same loop drives readiness and drain waits.
