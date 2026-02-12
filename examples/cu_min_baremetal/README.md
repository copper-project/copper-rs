# cu-min-baremetal

## Overview

`cu-min-baremetal` is a runnable Copper example.

## Prerequisites

- Rust stable toolchain
- Any hardware or services referenced by this example

## Run

```bash
cargo run -p cu-min-baremetal
```

## Expected Output

Expect successful startup logs and normal task-loop execution without panics.

## Links

- Example path: `examples/cu_min_baremetal`
- Build/deploy guide: <https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application>

## Additional Notes

### Cu-min-baremetal: a minimal / template for using Copper in a no-std environment

This is mainly a test for compiling copper on a no-std target (here arm64 MCUs) with no extra dependencies on a specific platform.
Check out our examples like the rp2350 for an actual targeted platform.
