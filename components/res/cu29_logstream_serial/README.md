# Serial LogStream transport

`SerialLogStreamTx<S, N>` and `SerialLogStreamRx<S, N>` adapt a
`cu_serial::SerialIo` resource to Copper's packet-oriented `CuStreamTx`/`CuStreamRx`.
`SerialLogStreamTxResources<S, N>` consumes a `serial` input and exports `tx`;
`SerialLogStreamRxResources<S, N>` consumes a `serial` input and exports `rx`.
Each provider owns its serial resource. Choose a direction for each carrier.

The framing format is a `0x7e` delimiter, escaped LogStream packet bytes, then a
`0x7e` delimiter. Bytes `0x7e` and `0x7d` are escaped as `0x7d` followed by the
byte XOR `0x20`. CRC verification uses the LogStream packet CRC. Invalid,
oversized, interrupted and CRC-invalid frames are discarded, and the next
delimiter resynchronizes reception. Both serial peers must use this framing.

`N` is the compile-time frame capacity (default 514), allowing packets up to
`(N - 2) / 2` bytes (default 256) even when every byte requires escaping. TX retains
one encoded frame. A busy transmitter returns `WouldBlock`; retry that packet
after the current frame drains. `Ok(())` means one complete packet was accepted
into the buffer. Encoding runs in the configured LogStream worker.

The scheduled sender services `CuStreamTx::poll_pending()` while idle and while
draining at shutdown. Applications driving endpoints directly, including
`no_std` applications, must keep polling `poll_pending()` until it returns false.
Each poll performs at most one nonblocking UART write.

RX reads at most 64 bytes per call and returns at most one packet. A too-small
caller buffer returns `BufferTooSmall` and retains that packet for a larger buffer.
Feed successful packets into `SessionRouter`. LogStream's configured limits,
recovery and FEC policies apply to these packets.

Configure MTU within the adapter bound and pace below the serial link's usable
throughput, accounting for escaping and serial start/stop bits. Use a burst of
one for slow links.
