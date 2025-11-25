# cu-crsf

Serial bridge for TBS Crossfire / ExpressLRS receivers. It parses CRSF packets coming from a UART into Copper messages and can optionally forward RC or link statistics back out over the same link.

## Channels
- `rc_rx` (`RcChannelsPayload`): latest RC channel values from the receiver.
- `lq_rx` (`LinkStatisticsPayload`): downlink link-quality metrics.
- `rc_tx` / `lq_tx`: optional uplink of RC or link statistics toward the transmitter.

## Configuration
**std builds**

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `serial_path` | string | — | Path to the serial device (required). |
| `baudrate` | u32 | `420000` | UART baud rate. |
| `timeout_ms` | u32 | `100` | Read timeout in milliseconds. |

**no-std builds**

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `serial_port_index` | u32 | `0` | Slot index pulled from `cu_embedded_registry`. Register the port before starting Copper. |

## Usage
std target example:
```ron
bridges: [
  (
    id: "crsf",
    type: "cu_crsf::CrsfBridge<cu_crsf::std_serial::StdSerial, std::io::Error>",
    config: { "serial_path": "/dev/ttyUSB0", "baudrate": 420000 },
    channels: [ Rx (id: "rc_rx"), Rx (id: "lq_rx") ],
  ),
],
cnx: [
  (src: "crsf/rc_rx", dst: "controller", msg: "cu_crsf::messages::RcChannelsPayload"),
]
```

Embedded target example (no-std):
```rust
use cu_crsf::CrsfBridge;
use embedded_io::{Read, Write};

// Register your UART once; the bridge will pull it by slot at startup.
let uart = /* your UART implementing Read+Write */;
<CrsfBridge<_, _> as cu_crsf::SerialFactory<_>>::register_serial(0, uart)?;
```

See `examples/cu_elrs_bdshot_demo` for a complete wiring that feeds CRSF RC channels into a BDShot bridge on the RP2350 reference board.
