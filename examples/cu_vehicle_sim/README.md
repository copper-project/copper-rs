# Toyota Vehicle CAN Network Simulation

A production-grade vehicle CAN network simulation built on top of **copper-rs**, using
real signal definitions from the [opendbc](https://github.com/commaai/opendbc) project's
Toyota TSS2 ADAS radar DBC file.

## What This Does

This example simulates a **Toyota Safety Sense 2 (TSS2) ADAS radar ECU** transmitting
all 34 CAN messages defined in `toyota_tss2_adas.dbc`:

| Message Group    | CAN IDs       | Count | Description                                   |
|-----------------|---------------|-------|-----------------------------------------------|
| `TRACK_A_0..15` | 0x180 – 0x18F | 16    | Primary track data (distance, speed, lateral)  |
| `TRACK_B_0..15` | 0x190 – 0x19F | 16    | Secondary track data (accel, confidence score) |
| `NEW_MSG_1`     | 0x240         | 1     | Misc ADAS signal 1                            |
| `NEW_MSG_2`     | 0x241         | 1     | Misc ADAS signal 2                            |

Each message includes:
- **Proper Toyota checksums** — computed identically to the real ECU (ported from opendbc's `toyotacan.py`)
- **Auto-incrementing counters** — 0–255 wrapping, one counter per message ID
- **Correctly packed signals** — big-endian (Motorola) bit packing matching opendbc's `set_value()`
- **Physically realistic values** — 16 radar tracks with simulated distance, speed, acceleration

## Architecture

```
┌───────────────────────┐          ┌──────────────────┐
│  ToyotaRadarEcu       │──CAN────▶│   CanBusSpy      │
│  (CuSrcTask)          │  Frame   │   (CuSinkTask)   │
│                       │          │                   │
│  • Round-robin 34 msgs│          │  • DBC decode     │
│  • Signal packing     │          │  • Checksum verify│
│  • Toyota checksums   │          │  • Counter check  │
│  • Physics simulation │          │  • Pretty-print   │
└───────────────────────┘          └──────────────────┘
```

The simulation runs as a standard copper-rs task graph:
- **`ToyotaRadarEcu`** is a `CuSrcTask` that produces one `CanFrame` per `process()` call
- **`CanBusSpy`** is a `CuSinkTask` that receives each frame, decodes it, and logs results
- Wiring is defined in `copperconfig.ron` — standard copper RON configuration

## File Structure

```
examples/cu_vehicle_sim/
├── Cargo.toml                  # Dependencies (cu29, cu29-helpers, cu-automotive-payloads)
├── build.rs                    # LOG_INDEX_DIR for copper runtime
├── copperconfig.ron            # Task graph: radar_ecu → bus_spy
├── dbc/
│   └── toyota_tss2_adas.dbc    # Real Toyota DBC from opendbc
├── scripts/
│   ├── parse_dbc.py            # DBC → Rust code generator
│   └── setup_vcan.sh           # Linux vcan0 interface setup
├── src/
│   ├── main.rs                 # Entry point + copper_runtime! macro
│   ├── dbc_generated.rs        # Auto-generated: 34 messages, 180+ signals
│   ├── signal_pack.rs          # CAN signal packing engine (opendbc-compatible)
│   ├── toyota_checksum.rs      # Toyota checksum algorithm
│   ├── ecu_radar.rs            # Radar ECU simulation task
│   └── bus_spy.rs              # CAN bus decoder/validator
└── README.md                   # This file
```

## Building

```bash
# From the workspace root
cargo build -p cu-vehicle-sim

# Run tests (7 tests for signal packing and checksums)
cargo test -p cu-vehicle-sim
```

## Running

```bash
cargo run -p cu-vehicle-sim
```

You'll see output like:

```
╔══════════════════════════════════════════════════════════════╗
║  Toyota TSS2 ADAS Vehicle CAN Network Simulation           ║
║  DBC: toyota_tss2_adas.dbc (34 messages, 16 radar tracks)  ║
║  Press Ctrl+C to stop                                      ║
╚══════════════════════════════════════════════════════════════╝
[CAN] 0x180 TRACK_A_0        [OK] LAT_DIST=-3.00m LONG_DIST=20.00m NEW_TRACK=1.00 REL_SPEED=-2.00 VALID=1.00
[CAN] 0x181 TRACK_A_1        [OK] LAT_DIST=-2.00m LONG_DIST=45.00m NEW_TRACK=0.00 REL_SPEED=-1.20 VALID=1.00
...
```

### Configuration

Edit `copperconfig.ron` to tune the simulation:

```ron
(id: "radar_ecu", type: "ecu_radar::ToyotaRadarEcu", config: {
    "active_tracks": 6,        // Number of active radar tracks (0–16)
    "base_speed_kph": 100.0,   // Ego vehicle speed in km/h
})

(id: "bus_spy", type: "bus_spy::CanBusSpy", config: {
    "verbose": true,           // Print every frame (false = summary only)
    "summary_interval": 340,   // Print summary every N frames
})
```

## How It Works

### 1. DBC Code Generation

The Python script `scripts/parse_dbc.py` parses the DBC file and generates
`src/dbc_generated.rs` — a Rust file with static arrays of message and signal
definitions. This means:

- **Zero runtime parsing** — all DBC data is compiled into the binary
- **Type-safe signal access** — each signal has factor, offset, min/max, unit
- **Deterministic memory** — no heap allocation for DBC data

To regenerate after DBC changes:
```bash
cd examples/cu_vehicle_sim
python3 scripts/parse_dbc.py dbc/toyota_tss2_adas.dbc > src/dbc_generated.rs
```

### 2. Signal Packing Engine

`signal_pack.rs` implements CAN signal packing/unpacking that matches opendbc's
`set_value()` and `get_raw_value()` exactly:

- **Physical → Raw**: `raw = round((physical - offset) / factor)`
- **Signed signals**: Two's complement encoding/decoding
- **Big-endian (Motorola)**: Correct bit walk for Toyota's byte order
- **Round-trip verified**: Pack → unpack produces the same physical value (within 1 LSB)

### 3. Toyota Checksum

`toyota_checksum.rs` is a direct port of the Toyota checksum from opendbc:
```
checksum = (dlc + sum_of_address_bytes + sum_of_data_bytes_except_last) & 0xFF
```
The checksum occupies the last byte of every TRACK_A and TRACK_B message.

### 4. Radar ECU Simulation

The `ToyotaRadarEcu` task simulates 16 radar tracks with:
- **Longitudinal distance** — 0 to 300m, evolving over time
- **Lateral distance** — ±50m, simulating lane positions
- **Relative speed** — cars approaching/receding at realistic speeds
- **Acceleration** — sinusoidal variation simulating traffic
- **Track score** — confidence inversely proportional to distance
- **New track detection** — triggered when a track resets

Messages are produced in **round-robin order** (TRACK_A_0, A_1, ..., A_15, B_0, ..., B_15, NEW_MSG_1, NEW_MSG_2), one per copper clock tick, matching how a real radar ECU schedules its CAN transmissions.

### 5. Bus Spy Validation

The `CanBusSpy` receives every frame and:
1. Looks up the CAN ID in the compiled DBC database
2. Validates the Toyota checksum
3. Checks counter continuity (detects dropped frames)
4. Unpacks all signals to physical values
5. Prints human-readable output or periodic summaries

## Optional: Virtual CAN Interface

For integration with Linux SocketCAN tools (candump, cansend, etc.):

```bash
sudo ./scripts/setup_vcan.sh        # Create vcan0
sudo ./scripts/setup_vcan.sh --down # Tear down vcan0
```

This creates a virtual CAN interface that can be used with standard Linux
CAN utilities for external monitoring.

## Extending This Example

### Adding a New ECU

1. Create a new file (e.g., `src/ecu_braking.rs`) implementing `CuSrcTask`
2. Use `pack_signal()` and `apply_toyota_checksum()` to build CAN frames
3. Add the task to `copperconfig.ron`
4. Wire it to the bus spy (or to other ECU consumers)

### Using a Different DBC File

1. Place your DBC file in `dbc/`
2. Run: `python3 scripts/parse_dbc.py dbc/your_file.dbc > src/dbc_generated.rs`
3. Update the ECU task to produce the new message set
4. The bus spy automatically adapts (it reads from `DBC_MESSAGES`)

### Adding Real CAN Data

Replace the simulated values in `ecu_radar.rs` with data from:
- A CAN log file (`.blf`, `.asc`, `.csv`)
- A live SocketCAN interface
- A vehicle simulator (CARLA, LGSVL, etc.)

## Tests

```bash
cargo test -p cu-vehicle-sim
```

| Test | Description |
|------|-------------|
| `pack_unpack_be_unsigned` | Round-trip 8-bit big-endian unsigned signal |
| `pack_unpack_be_signed` | Round-trip 11-bit big-endian signed signal (LAT_DIST) |
| `pack_unpack_le_unsigned` | Round-trip 7-bit big-endian unsigned signal |
| `round_trip_all_track_a_signals` | Full TRACK_A_0 message pack → unpack with DBC definitions |
| `checksum_basic` | Toyota checksum on empty frame |
| `checksum_with_data` | Toyota checksum on frame with data |
| `checksum_idempotent` | Applying checksum twice gives same result |

## Dependencies

- **cu29** — Copper runtime (task graph, scheduling, logging)
- **cu29-helpers** — Setup utilities (logger, slab allocation)
- **cu-automotive-payloads** — CAN frame types (`CanFrame`, `CanId`, `CanFlags`)
- **serde** — Configuration deserialization
