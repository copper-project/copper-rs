# cu-crsf

Serial bridge for TBS Crossfire / ExpressLRS receivers. It parses CRSF packets coming from a UART into Copper messages and can optionally forward RC or link statistics back out over the same link.

## Channels
- `rc_rx` (`RcChannelsPayload`): latest RC channel values from the receiver.
- `lq_rx` (`LinkStatisticsPayload`): downlink link-quality metrics.
- `rc_tx` / `lq_tx`: optional uplink of RC or link statistics toward the transmitter.

## Resources and configuration
The bridge expects a `serial` resource (anything implementing `embedded_io` `Read`/`Write` + `Send + Sync`). Point the bridge's `resources` map at a bundle entry.

**std builds**

Use `cu_linux_resources::LinuxResources` as the serial provider; it stores an owned
resource in fixed slots like `<bundle>.serial_acm0` and `<bundle>.serial_usb0`.

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `tty_acm0_path` / `tty_usb0_path` | string | `/dev/ttyACM0` / `/dev/ttyUSB0` | Path override for a serial slot. |
| `serial_baudrate` | u32 | `115200` | UART baud rate for Linux serial slots. |
| `serial_timeout_ms` | u64 | `50` | Read timeout in milliseconds. |

```ron
resources: [
  (
    id: "radio",
    provider: "cu_linux_resources::LinuxResources",
    config: { "tty_usb0_path": "/dev/ttyUSB0", "serial_baudrate": 420000 },
  ),
],
bridges: [
  (
    id: "crsf",
    type: "cu_crsf::CrsfBridgeStd",
    resources: { serial: "radio.serial_usb0" },
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
