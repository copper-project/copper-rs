# cu-crsf

Serial bridge for TBS Crossfire / ExpressLRS receivers. It parses CRSF packets coming from a UART into Copper messages and can optionally forward RC or link statistics back out over the same link.

## Channels
- `rc_rx` (`RcChannelsPayload`): latest RC channel values from the receiver.
- `lq_rx` (`LinkStatisticsPayload`): downlink link-quality metrics.
- `rc_tx` / `lq_tx`: optional uplink of RC or link statistics toward the transmitter.

## Resources and configuration
The bridge expects a `serial` resource (anything implementing `embedded_io` `Read`/`Write` + `Send + Sync`). Point the bridge's `resources` map at a bundle entry.

**std builds**

Use the built-in `StdSerialBundle`, which opens a serial port and stores it as `Mutex<StdSerial>` under `<bundle>.serial`.

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `serial_path` | string | — | Path to the serial device (required). |
| `baudrate` | u32 | `420000` | UART baud rate. |
| `timeout_ms` | u32 | `100` | Read timeout in milliseconds. |

```ron
resources: [
  (
    id: "radio",
    provider: "cu_crsf::StdSerialBundle",
    config: { "serial_path": "/dev/ttyUSB0", "baudrate": 420000 },
  ),
],
bridges: [
  (
    id: "crsf",
    type: "cu_crsf::CrsfBridgeStd",
    resources: { serial: "radio.serial" },
    channels: [ Rx (id: "rc_rx"), Rx (id: "lq_rx") ],
  ),
],
```

**no-std builds**

Provide your own bundle that moves a UART into the `ResourceManager` (typically a `spin::Mutex<SerialPort>`). The `resources` module in `examples/cu_elrs_bdshot_demo` shows a complete pattern:

```ron
resources: [
  ( id: "fc", provider: "my_app::resources::RadioBundle" ),
],
bridges: [
  (
    id: "crsf",
    type: "cu_crsf::CrsfBridge<SerialResource, SerialPortError>",
    resources: { serial: "fc.serial" },
    channels: [ Rx (id: "rc_rx"), Tx (id: "lq_tx") ],
  ),
],
```

See `examples/cu_elrs_bdshot_demo` for a full wiring that feeds CRSF RC channels into a BDShot bridge on the RP2350 reference board.
