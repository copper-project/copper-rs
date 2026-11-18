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
`with_telemetry_tx(move |_| Ok(PlainUdpTx::new(tx)), MAX_PAYLOAD, MAX_PAYLOAD)`.
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
