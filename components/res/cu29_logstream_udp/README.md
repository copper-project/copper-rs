# UDP logstream resources

`cu29-logstream-udp` provides std-only `CuUdpLogStreamTx` and `CuUdpLogStreamRx` endpoints
implementing the existing `CuStreamTx` and `CuStreamRx` packet traits. Linux and
macOS are the primary hosts; Windows supports the basic carrier.

The carrier submits each datagram once, nonblockingly. It does not retry,
acknowledge, pace, encode FEC, or interpret manifests. `Ok(())` means the local
socket accepted the datagram, not that a receiver received it. UDP can silently
drop traffic when kernel queues fill. `WouldBlock` is returned immediately when
the OS reports it; an interrupted operation is also returned without retry.

## Sender resource

Add `cu29-logstream-udp` to the application and enable `cu29/logstream`. Configure the
socket in `resources`, then bind its concrete `tx` endpoint in `log_streaming`:

```ron
resources: [
    (
        id: "telemetry_udp",
        provider: "cu29_logstream_udp::CuUdpLogStreamResources",
        config: {
            "bind_addr": "0.0.0.0:0",
            "remote_addr": "127.0.0.1:7447",
            "send_buffer_bytes": 262144,
            "ttl": 1,
            "dscp": 46,
        },
    ),
],
```

The destination's transport binding is:

```ron
transport: (
    type: "cu29_logstream_udp::CuUdpLogStreamTx",
    resource: "telemetry_udp.tx",
),
```

[The integration fixture](tests/configured_sender.ron) contains a complete
configuration, including the required link, FEC, content, and record bounds.
MTU and stream policy belong there, not in the UDP resource. The current sender
carries bitrate/burst/latency policy in its manifest but does not yet enforce
pacing; link scheduling and optional feedback are a later iteration.

## Receiver resource

Use the same provider with an explicit listen address and optional kernel buffer:

```ron
resources: [
    (
        id: "listen",
        provider: "cu29_logstream_udp::CuUdpLogStreamResources",
        config: {
            "bind_addr": "127.0.0.1:7447",
            "recv_buffer_bytes": 262144,
        },
    ),
],
```

Take the `listen.rx` resource as `CuUdpLogStreamRx` and pass it, with a caller-owned
packet buffer, to `SessionRouter::try_receive`. Standalone receivers can instead
construct `CuUdpLogStreamConfig::new(listen_addr)` and call `.open()` to obtain the
same endpoints. There are no environment-variable fallbacks or DNS lookups.

The bundle always registers `rx`; it registers `tx` only when `remote_addr` is
present. Both slots share one bound, unconnected socket. Endpoint clones share
that socket without a userspace mutex. Receive clones compete for packets;
they do not broadcast copies. A receive endpoint accepts datagrams from any
sender and does not automatically transmit responses.

## Socket options and bounds

| Key | Meaning |
| --- | --- |
| `bind_addr` | Required numeric IPv4/IPv6 address and port; port zero requests an ephemeral port. |
| `remote_addr` | Optional transmit destination, with the same address family and a nonzero port. |
| `send_buffer_bytes` | Optional OS send-buffer request, in 1..=2147483647. |
| `recv_buffer_bytes` | Optional OS receive-buffer request, in 1..=2147483647. |
| `ttl` | Optional IPv4 unicast TTL / IPv6 unicast hop limit, in 1..=255. |
| `dscp` | Optional six-bit DSCP value, in 0..=63; encoded with ECN bits zero. |

Absent options keep OS defaults. Kernel buffer requests may be rounded or
clamped by the OS. Unknown keys, numeric overflow, invalid address families, and
out-of-range values are rejected during setup. IPv6 sockets use IPv6-only mode.
IPv6 DSCP is supported on Linux, macOS, and Android; explicitly requesting it on
other targets fails during setup. Explicit DSCP configuration is rejected on
Windows for both address families: [Winsock requires its separate QoS API](https://learn.microsoft.com/en-us/windows/win32/winsock/ipproto-ip-socket-options)
to set the traffic class. Omit `dscp` for the basic Windows carrier.

Reception writes directly into the supplied slice without an internal packet
allocation or an extra payload copy. `Ok(None)` means no data, and `Ok(Some(0))`
means an empty datagram. An oversized datagram is consumed and returns
`BufferTooSmall`, never successful partial data; ignore buffer contents on error.
Linux/Android return the original packet length as `needed`. Other supported
hosts return a conservative sufficient capacity of 65,535 bytes because their
receive API does not report the original length after truncation. Ordinary UDP
datagrams are supported; IPv6 jumbograms are outside this contract. Configured
streams should stay within their transport MTU, normally 1200 bytes on unknown
IP paths, to avoid IP fragmentation.

## Verification and iteration boundary

From the repository root, run `just logstream-udp-check`. It checks Clippy and
tests IPv4/IPv6 loopback, empty/exact/oversized datagrams, resource options,
invalid configuration, shared endpoints, queue pressure, and error/no-retry
behavior. A deterministic submission test covers `WouldBlock`: real UDP queue
pressure often drops packets silently instead of returning backpressure.

The recipe enables the crate's `runtime-integration` test feature explicitly.
Normal carrier builds do not enable `cu29/logstream` or its asynchronous runtime
features for other workspace apps. The root PR check and host CI run the
integration test separately from the general workspace tests.

The generated-runtime integration test receives actual UDP traffic in socket
arrival order, discovers its manifest without out-of-band FEC settings, and
decodes correctly ordered application-typed CopperLists and recovery objects.
The current router discards packets preceding manifest discovery; this test
does not sort packets to hide that startup limitation or assert a complete
session archive. Automatic verified anchor recovery and native receiver
archival are the next PR, followed by a runnable two-process demonstrator.
