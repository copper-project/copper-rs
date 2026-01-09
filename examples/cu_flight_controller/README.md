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

### RC Tester (simulation)

```bash
# Test RC input via joystick
just rc
```

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
