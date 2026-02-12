# cu-caterpillar

## Overview

`cu-caterpillar` is a runnable Copper example.

## Prerequisites

- Rust stable toolchain
- Any hardware or services referenced by this example

## Run

```bash
cargo run -p cu-caterpillar
```

## Expected Output

Expect successful startup logs and normal task-loop execution without panics.

## Links

- Example path: `examples/cu_caterpillar`
- Build/deploy guide: <https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application>

## Additional Notes

### Cu-caterpillar: full example for Copper

This is an example for the Copper project running on a Raspberry Pi flipping in sequence 8 GPIO pins.
It allows to gauge the latency and performance of the Copper runtime with minimal user code.

See the crate cu29 for more information about the Copper project.

### Justfile commands

- `just cl` — dump the Copper list from a log at `logs/caterpillar.copper`.
- `just logreader` — extract logs from `logs/caterpillar.copper` into `../../target/debug/cu29_log_index`.
- `just resim` — rerun the logged mission from `logs/caterpillar.copper`.
- `just dag-logstats` — generate logstats and open an annotated DAG SVG for the current `copperconfig.ron`.
