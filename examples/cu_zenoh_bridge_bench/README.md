# cu-zenoh-bridge-bench

Minimal Zenoh bridge latency benchmark with just two Copper tasks:

- one TX source task emitting one message every 10 ms
- one RX sink task computing one-way latency and printing rolling stats once per second

The source runtime runs at `100 Hz` so the emission cadence is fixed at 10 ms.
The receiver runtime is intentionally left unbounded so it polls the Zenoh subscriber as fast as possible instead of adding a second scheduler quantization term.

The benchmark uses high-precision system wall-clock timestamps carried inside the payload.
It assumes sender and receiver clocks are synchronized; running both processes on the same host is the intended setup.

## Run

Terminal 1:
```bash
just rx
```

Terminal 2:
```bash
just tx
```

The RX process prints lines like:

```text
[latency] samples 100 min 87.000us p50 122.000us p95 184.000us p99 231.000us max 264.000us mean 128.000us stddev 29.000us | jitter p95 43.000us max 58.000us | dropped 0 max_gap 1
```

Logs are written under `examples/cu_zenoh_bridge_bench/logs/` by default.
Use release mode (`cargo run -r ...`) on both sides to keep benchmark overhead reasonable without the long `screaming` rebuilds.
Both binaries print startup and ready lines to stdout, including the effective transport and endpoint.

## Transport Selection

Shared memory is the default transport, so `just rx` / `just tx` is the local shmem baseline.

For back-to-back transport comparisons, use matching pairs:

```bash
just rx
just tx
```

```bash
just rx-udp
just tx-udp
```

```bash
just rx-tcp
just tx-tcp
```

The binaries also accept `--transport shmem|udp|tcp` directly:

```bash
cargo run -r -p cu-zenoh-bridge-bench --bin zenoh-bench-rx -- --transport tcp
cargo run -r -p cu-zenoh-bridge-bench --bin zenoh-bench-tx -- --transport tcp
```

The embedded `tx_config.ron` / `rx_config.ron` stay on the shmem baseline for DAG rendering and no-override runs, but the benchmark normally selects transport by applying a config override at startup.

For TCP, UDP, and shmem direct-connect presets, the TX side is configured as a Zenoh `peer` with `connect.timeout_ms = -1` and `connect.exit_on_failure = false`, so starting TX before RX is expected to work without a Copper-side retry loop.

## Local Snapshot

On the same host, your current measurements put the three transports in this rough order:

- shmem: best median and tail, typically around `p50 ~= 50-70 us`
- tcp: close behind shmem, typically around `p50 ~= 56-78 us`
- udp: slower and a bit noisier here, typically around `p50 ~= 66-90 us`

Those numbers already suggest scheduler noise is part of the remaining tail, so it is reasonable to try Linux realtime scheduling on both processes.

## Realtime Scheduling

The example now ships with `chrt --fifo 80` launch recipes:

```bash
sudo just rx-rt
sudo just tx-rt
```

```bash
sudo just rx-udp-rt
sudo just tx-udp-rt
```

```bash
sudo just rx-tcp-rt
sudo just tx-tcp-rt
```

These recipes build the normal release binaries first and then launch the binaries themselves under `SCHED_FIFO`, so the expensive part stays in plain `cargo build -r`.
They require `CAP_SYS_NICE` or `sudo`.

What this benchmark measures:

- TX task timestamping
- Copper bridge TX and RX execution
- Zenoh transport latency
- RX task dispatch latency

What it intentionally avoids:

- an extra fixed RX rate-target quantization term
