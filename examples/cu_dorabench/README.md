## Cu-dorabench: let's drag race vs dora-rs

This is to compare apple to apple with dora-rs another rust middleware for robotics.

This mimics the implementation of https://github.com/dora-rs/dora-benchmark

Usage: `cargo run -r --profile screaming`

Warning: say goodbye to your disk space, we are logging to disk 40MiB at every message as fast as your storage allows us to do.

The payload in this example is a tiny newtype around `CuHandle<Vec<u8>>`.
The generated `CuStampedDataSet` now carries a runtime-only IO cache beside the
message tuple.

- Location: `CuStampedDataSet(..., CuMsgIoCache<N>)`
- Contents: resident bytes, encoded bytes, and handle-backed bytes for each message slot
- Update point: during the normal CopperList bincode encode path
- Purpose: reuse the measured sizes for live monitoring and generated payload-size queries instead of running a second size-only encode pass
