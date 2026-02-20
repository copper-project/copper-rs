# cu-feetech

Copper bridge for Feetech STS/SCS serial bus servos (e.g. STS3215 in SO-100/SO-101 arms).

- **Rx `positions`**: present joint positions from all configured servos.
- **Tx `goal_positions`**: goal positions written via sync-write.

## Config

In `copperconfig.ron`: bind a serial resource and set servo IDs (`servo0`, `servo1`, …). Optionally set `units` to `"raw"` (default), `"deg"`, `"rad"`, or `"normalize"`; for deg/rad/normalize add `calibration_file` (path to JSON from `feetech-calibrate`). For deg/rad, `ticks_per_rev` (raw units per 360°) is model-dependent and optional (default 4096). Use `"normalize"` for leader–follower so both arms share the same [-1, 1] scale per joint.

## Calibration

Run the calibration binary, move each servo through its range, then press Enter. Optionally pass the output path as the last argument (default: `calibration.json`):

```sh
cargo run -p cu-feetech --bin feetech-calibrate -- /dev/ttyACM0 1 2 3 4 5 6
cargo run -p cu-feetech --bin feetech-calibrate -- /dev/ttyACM0 1 2 3 4 5 6 calibration_leader.json
```

## Example

See the [cu-feetech-demo](../../examples/cu_feetech_demo) example for a minimal app that reads and logs positions.
