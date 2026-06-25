# cu-feetech-demo

Example using the [cu-feetech](../../components/bridges/cu_feetech) bridge with SO-100/SO-101 Feetech STS3215 arms.

## Missions

| Mission           | Description |
|-------------------|-------------|
| **arm_publisher**  | Read positions from one arm and log them. |
| **leader_follower** | Leader arm (serial0) is moved by hand; follower arm (serial1) copies its positions. Each arm uses its own calibration file. |

## Setup

- Connect one or two arms via USB–serial (`/dev/ttyACM0`, `/dev/ttyACM1`).
- To determine the correct port, after plugging the USB-C cable, run the following command:

  ```sh
  dmesg | tail -n 20
  ```
- Edit `copperconfig.ron`: set `serial0_dev` (and `serial1_dev` for leader_follower) and `servo0`…`servo5` to match your bus.

## Calibration (optional)

For **arm_publisher** with deg/rad: run the tool once, save as `calibration.json`, set `units` and `calibration_file` in the feetech bridge config.

For **leader_follower**: calibrate each arm separately. Pass the output path as the last argument so you can name the file directly (config expects `calibration_leader.json` and `calibration_follower.json`). Positions are in **normalized**.

```sh
cargo run -p cu-feetech --bin feetech-calibrate -- /dev/ttyACM0 1 2 3 4 5 6 calibration_leader.json
# move leader arm, press Enter

cargo run -p cu-feetech --bin feetech-calibrate -- /dev/ttyACM1 1 2 3 4 5 6 calibration_follower.json
# move follower arm, press Enter
```

## Run

```sh
cargo run -p cu-feetech-demo -- arm_publisher
cargo run -p cu-feetech-demo -- leader_follower
```
