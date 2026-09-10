# cu-image-codec-demo

This example generates one synthetic RGB image stream and logs it twice:

- once through the PNG log codec
- once through the FFV1 log codec
- using pooled host image buffers so the source behaves like a real Copper image producer

That keeps image codec experiments separate from viewer/debugger tooling,
rather than mixing them with log codec benchmarking.

## Run

```bash
just run
just fsck
just log-stats
```

## ConsoleMon variant

This variant wires in `cu_consolemon::CuConsoleMon` and keeps the runtime
running until the monitor exits so you can inspect the codec pipeline
interactively.

Run it from an interactive terminal. The binary exits early when stdin or
stdout is not a TTY so it does not hang headless.

```bash
just run-consolemon
just fsck-consolemon
just log-stats-consolemon
```

## Payload framing

Copper surrounds each present codec payload with a fixed four-byte little-endian
encoded length. It reserves the header, encodes directly into log storage, then
fills in the byte count. On decode, the codec receives a reader limited to that
payload, keeping adjacent messages and metadata intact. Codecs must consume the
complete frame. Writers used for codec logging must support bincode's `position`
and `overwrite` operations; Copper's memory-mapped writer supports both.

Read logs with the logreader built for the application version that recorded them.
The codec payload framing changes the encoded content; the unified-log file and
section header version remains unchanged.

Run `just codec-framing-check` from the repository root to check codec boundaries,
PNG pixel roundtrips, generated replay, and the shared `no_std` build.
