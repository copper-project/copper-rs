# benchmarks

External benchmark and comparison workloads for Copper.

This directory holds side-by-side and interoperability-oriented comparisons
that are useful to keep around, but are intentionally checked separately from
the default Copper workspace test matrix.

This directory currently contains:

- `benchmarks/cu_async_cl_io_bench`: CopperList logging and async CL I/O benchmark
- `benchmarks/cu_dorabench`: Copper-side benchmark matching `dora-rs/dora-benchmark`
- `benchmarks/cu_zenoh_bridge_bench`: Zenoh bridge transport latency benchmark
- `benchmarks/dora_caterpillar`: DORA version of the caterpillar benchmark
- `benchmarks/horus_caterpillar`: Horus version of the caterpillar benchmark
- `benchmarks/ros_caterpillar`: ROS 2 comparison workspace
- `benchmarks/ros_zenoh_caterpillar`: ROS 2 + Zenoh comparison workspace

The Copper benchmark crates use local workspace dependencies. CI runs them via
the root `just check-benchmarks` target.

## Links

- Main Copper runtime and SDK: [`copper-project/copper-rs`](https://github.com/copper-project/copper-rs)
- Copper documentation: <https://copper-project.github.io/copper-rs>
- Copper book: <https://copper-project.github.io/copper-rs-book/>

## Checks

```bash
just check
```

CI checks the Copper benchmark crates plus the standalone DORA and Horus
comparison crates. The ROS 2 workspaces under `benchmarks/ros_caterpillar` and
`benchmarks/ros_zenoh_caterpillar` are kept here as comparison material and are
not built by the default benchmark job.

## License

This repository is licensed under the Apache License, Version 2.0.
