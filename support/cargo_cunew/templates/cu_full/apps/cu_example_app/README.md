This is the primary Copper application for the workspace (`cu_example_app`). Local tasks and messages live under `src/`.

The app ships with three host-side binaries:

- `cu_example_app`: normal runtime execution.
- `cu_example_app-logreader`: offline log export and inspection.
- `cu_example_app-resim`: replay target that can run once or expose the remote debug API with `--debug-base`.

## Monitors

To enable the `cu-memmon` per-task heap-allocation monitor, uncomment the
`cu-memmon` dep and `memmon` feature in this `Cargo.toml` and the matching
entry in the workspace `Cargo.toml`, then add to `copperconfig.ron`:

```ron
monitor: (
    type: "cu_memmon::CuMemMon",
    config: { "realtime_strict": false, "summary_every": 100 },
),
```

Build with `cargo run --features memmon`. See the `cu_memmon` crate README
for tuning knobs and output format.

## Self-describing logs

Run `cargo run --features self-describing-logs` to include a compressed payload
catalog in the application log. The normal builder writes it at startup using
the existing payload `Encode` derives, including payloads from dependencies.
Custom encoders provide a matching `cu29::bincode::ValueDecode` implementation.
