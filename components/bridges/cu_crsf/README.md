# cu-crsf

Serial bridge for TBS Crossfire / ExpressLRS receivers. It parses CRSF packets coming from a UART into Copper messages and can optionally forward RC or link statistics back out over the same link.

## Channels
- `rc_rx` (`RcChannelsPayload`): latest RC channel values from the receiver.
- `lq_rx` (`LinkStatisticsPayload`): downlink link-quality metrics.
- `rc_tx` / `lq_tx`: optional uplink of RC or link statistics toward the transmitter.

## Resources and configuration
The bridge expects a `serial` resource (anything implementing `embedded_io` `Read`/`Write` + `Send + Sync`). Point the bridge's `resources` map at a bundle entry.

**std builds**

Use `cu_linux_resources::LinuxResources` as the default provider. `cu_crsf::StdSerialBundle` is kept only for backward compatibility with older configs. Both open a serial port and store it as an owned `serial` resource under `<bundle>.serial`.

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `serial_path` / `device` | string | — | Path to the serial device (required). |
| `baudrate` | u32 | `115200` (`LinuxResources`) / `420000` (`StdSerialBundle`) | UART baud rate. |
| `timeout_ms` | u64 | `50` (`LinuxResources`) / `100` (`StdSerialBundle`) | Read timeout in milliseconds. |

```ron
resources: [
  (
    id: "radio",
    provider: "cu_linux_resources::LinuxResources",
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

Provide your own bundle that moves a UART into the `ResourceManager` (as an owned resource). If your UART type is not `Sync`, wrap it once at bundle registration time with `cu_linux_resources::Exclusive<T>`.

The `resources` module in `examples/cu_elrs_bdshot_demo` shows a complete pattern:

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
