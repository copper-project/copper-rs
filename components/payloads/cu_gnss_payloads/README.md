# cu-gnss-payloads

Standardized GNSS payload types for Copper.

This crate defines reusable GNSS data models (fix status, signals, satellites,
epoch data, and related events) shared by GNSS sources and downstream tasks.

## Features

- `std` (default)
- `textlogs`
- `reflect`


Enable `rerun` to inspect `GnssFixSolution` and `GnssNavEpoch` in Copper's map
view or log them directly to Rerun. Fixes are colored by correction
quality. Invalid or unavailable position fixes emit an empty point set so a
previous valid position does not linger. `GeodeticPosition` also exposes a map
representation through `cu-spatial-payloads/rerun`.

Record an example with `cargo run -p cu-gnss-payloads --features rerun --example rerun_gnss -- gnss.rrd`,
then open `gnss.rrd` in Rerun.
