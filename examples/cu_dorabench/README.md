# cu-dorabench

## Overview

`cu-dorabench` is a runnable Copper example.

## Prerequisites

- Rust stable toolchain
- Any hardware or services referenced by this example

## Run

```bash
cargo run -p cu-dorabench
```

## Expected Output

Expect successful startup logs and normal task-loop execution without panics.

## Links

- Example path: `examples/cu_dorabench`
- Build/deploy guide: <https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application>

## Additional Notes

### Cu-dorabench: let's drag race vs dora-rs

This is to compare apple to apple with dora-rs another rust middleware for robotics.

This mimics the implementation of https://github.com/dora-rs/dora-benchmark

Usage: `cargo run -r --profile screaming`

Warning: say goodbye to your disk space, we are logging to disk 40MiB at every message as fast as your storage allows us to do.
