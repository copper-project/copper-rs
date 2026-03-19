# cu-flight-controller

A bare-metal quadcopter flight controller implemented end-to-end using Copper components.

## Overview

This example demonstrates a complete flight controller running on the MicoAir H743 board (STM32H743). It showcases Copper's ability to run deterministic, real-time control loops on embedded hardware with zero dynamic allocation during runtime.

## Hardware

- **MCU**: STM32H743VIT @ 400MHz
- **IMU**: BMI088 (accelerometer + gyroscope)
- **RC Input**: CRSF protocol (ExpressLRS compatible)
- **ESC Output**: BDShot (bidirectional DShot with telemetry)
- **VTX/OSD**: MSP DisplayPort protocol
- **Storage**: MicroSD card for logging
- **Battery**: ADC voltage monitoring

See [doc/PINOUT.md](doc/PINOUT.md) for the complete pinout reference.

## Architecture

The flight controller uses a cascaded control architecture:

```
RC Input (CRSF) -> RC Mapper -> Attitude Controller -> Rate Controller -> Mixer -> ESCs (BDShot)
                                      ^                      ^
                                      |                      |
                          IMU -> Calibrator -> AHRS ---------+
```

### Task Graph

| Task | Description |
|------|-------------|
| `bmi088` | BMI088 IMU driver (accelerometer + gyroscope) |
| `imu_cal` | Gyroscope bias calibration on arm |
| `ahrs` | Attitude and Heading Reference System |
| `mapper` | RC channel mapping and arm/mode logic |
| `attitude` | Outer loop PID (angle to rate setpoint) |
| `rate` | Inner loop PID (rate to motor commands) |
| `mixer0-3` | QuadX motor mixing |
| `battery_adc` | Battery voltage monitoring |
| `vtx_osd` | MSP DisplayPort OSD rendering |
| `led_blink` | Status LED heartbeat |

### Flight Modes

- **Angle**: Self-leveling mode with attitude hold
- **Acro**: Rate mode for aerobatic flight
- **Position Hold**: (placeholder for GPS integration)

### Features

- **Airmode**: Maintains control authority at zero throttle for aerobatic maneuvers
- **Expo curves**: Configurable stick expo for smoother control
- **Gyro calibration**: Automatic bias calibration on arm
- **Zero-copy logging**: Binary logs to SD card via unified logger

## Building

### Firmware (for flashing to hardware)

```bash
# Build and flash with text logging (debug)
just fw

# Build and flash release (optimized, no text logs)
just fwr

# Build and flash debug profile
just fwd
```

### Log Reader (host tool)

```bash
# Extract CopperLists from log file
just logreader log=logs/embedded.copper

# Check log file integrity
just fsck log=logs/embedded.copper

# Extract text logs (requires log index)
just textlogs log=logs/embedded.copper
```

### Python GNSS Extraction

You can use the PyO3 bindings to iterate CopperLists directly from Python and
extract GNSS fields without going through JSON.

This is an offline log-analysis workflow, not a runtime Python task. Python only
touches data that Copper has already recorded, so this does not affect the
realtime behavior of the flight controller itself.

```bash
# Build the Python extension module
just py-build

# Print GNSS latitude/longitude from the flight-controller sim log
just py-gnss log=logs/flight_controller_sim.copper
```

The script is at `python/print_gnss_from_log.py` and can also be run directly:

```bash
python3 python/print_gnss_from_log.py logs/flight_controller_sim.copper
```

Implementation notes:

- `src/python_module.rs` exposes an app-specific `#[pymodule]`
- it uses `gen_cumsgs!("copperconfig.ron")` so the CopperList type matches this app
- the Python script imports that module and iterates typed CopperLists plus runtime lifecycle records

This pattern is the recommended Python story in Copper: post-process logs in Python
after the run, keep Python off the control path during the run.

### RC Tester (simulation)

```bash
# Test RC input via joystick
just rc
```

### Simulator (Bevy + Copper)

```bash
# Run the normal full-window simulator
just

# Run the split BevyMon simulator
just bevy

# Run the split BevyMon simulator in the browser
just web

# Build a deployable browser bundle into dist/flight-controller with hashed asset filenames
just web-dist
```

The split BevyMon path reuses the same `cu_bevymon::spawn_split_layout(...)` shell as
`cu_rp_balancebot` and `cu_bevymon_demo`, but the left panel still runs the real flight-controller
sim world, OSD, and help overlays.

### RC Input In Simulation

The simulator reads RC input from a host joystick device (`evdev`). By default it only auto-connects to
radio-style joystick profiles, to avoid false positives from keyboards/mice/gamepads that also expose joystick interfaces.

Environment variables:

```bash
# Prefer a specific device name substring (case-insensitive)
CU_SIM_JOYSTICK="radiomaster" just sim

# Allow generic/non-radio joystick devices as RC input
CU_SIM_ALLOW_GENERIC_JOYSTICK=1 just sim
```

Notes:
- If no compatible RC joystick is found, the sim falls back to keyboard controls.
- When connected, the bottom-right help panel shows the selected device name and technical axis bindings (`ABS_X`, `ABS_RY`, etc.).
- USB and Bluetooth radios both work if they appear as a joystick device on the host.

### Compatible TX/RX Notes

- **Simulation path**: no receiver is used directly; input is read from the host joystick interface.
- **Auto-detected TX joystick profiles**:
  - ExpressLRS-style USB joystick names (`expresslrs`, `radiomaster`)
  - OpenTX / EdgeTX USB joystick names (`opentx`, `edgetx`)
- **Firmware path (real hardware)**: RC input is CRSF on UART, so use a CRSF-compatible RX link
  (for example ExpressLRS/Crossfire-class receivers and compatible transmitters).

## Configuration

The task graph is defined in `copperconfig.ron`. Key configurable parameters:

### Rate Controller
```ron
config: {
    "kp": 0.04,
    "ki": 0.0,
    "kd": 0.0005,
    "airmode": true,
    "airmode_start_throttle_percent": 20.0,
}
```

### Attitude Controller
```ron
config: {
    "angle_limit_deg": 60.0,
    "acro_rate_dps": 180.0,
    "acro_expo": 0.3,
    "kp": 1.0,
}
```

### RC Mapper
```ron
config: {
    "arm_channel": 4,
    "arm_min": 1700,
    "arm_max": 1811,
    "mode_channel": 5,
}
```

## Motor Layout

QuadX configuration (props out):

```
    Front
  3       1
    \   /
      X
    /   \
  2       0
    Rear
```

| Motor | Position | Rotation |
|-------|----------|----------|
| 0 | Rear Right | CCW |
| 1 | Front Right | CW |
| 2 | Rear Left | CW |
| 3 | Front Left | CCW |

## Development

### Attaching to Running Target

```bash
just attach
```

### Viewing Logs

The firmware logs to the SD card in Copper's binary format. Use the log reader tools to extract and analyze:

```bash
# Extract structured data
just logreader

# Extract text logs (when compiled with textlogs feature)
just textlogs
```

## Dependencies

Key Copper components used:
- `cu29` - Core runtime
- `cu-ahrs` - Attitude estimation
- `cu-bdshot` - Bidirectional DShot ESC protocol
- `cu-crsf` - CRSF RC protocol (ExpressLRS)
- `cu-msp-bridge` - MSP protocol for VTX/OSD
- `cu-pid` - PID controller
- `cu-micoairh743` - MicoAir H743 HAL bundle
- `cu-logmon` - Log monitoring
