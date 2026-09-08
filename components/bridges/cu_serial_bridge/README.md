# Raw serial bridge

`SerialBridge<S>` consumes an owned `serial` resource implementing
`cu_serial::SerialIo`. Wire `bytes_tx` and `bytes_rx` channels using `ByteChunk`
(`CuArrayVec<u8, 256>`). The bridge supports both `std` and `no_std` builds.

RX reads directly into the Copper payload. Each message contains the bytes
returned by one serial read, up to 256 bytes. An idle read clears the payload.
Applications assemble these chunks according to their wire protocol.

TX copies each accepted payload into one fixed 256-byte buffer and attempts one
nonblocking write. Partial writes and zero progress retain the unsent suffix;
`preprocess` advances it with at most one write per cycle. Successful `send`
means the bridge accepted the payload for transmission. While bytes remain
queued, another nonempty payload returns a buffer-full error. Applications
handle this backpressure through their monitor policy.

An initial write error rejects the new message. A later lifecycle write error
retains the accepted message's unsent bytes for the next drain attempt. `stop`
attempts one final write and reports an error if bytes remain, preserving them
for a subsequent start.

Snapshots save and restore the queued TX bytes. In simulation mode, the runtime
supplies recorded boundary messages.
