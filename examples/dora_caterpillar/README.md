## Dora caterpillar example

This is a dora-rs version of the caterpillar chain used for Copper vs ROS comparisons.
It matches the Copper constraints: 17 tasks (1 source + 8 propagate + 8 GPIO sinks) and
16 connections (8 chain + 8 GPIO). Each node reports its tick duration to a stats node,
which prints a summary table once per second in the same format as horus.

### Run

```
cd examples/dora_caterpillar
dora run dataflow.yml
```

### Install dora

```
cargo install dora-cli --version 0.4.0 --locked
```

### Notes

- Requires `dora-cli` 0.4.0 or newer (matches `dora-node-api`).
- The dataflow uses `dora/timer/millis/1` to drive the source node.
- End-to-end latency reports src -> gpio-7 in the `End2End` row.
- Per-task rows report hop latency: `receiver_now - sender_timestamp`.
