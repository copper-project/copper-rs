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
