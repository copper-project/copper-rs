# cu-crsf

## Overview

Serial bridge for TBS Crossfire / ExpressLRS receivers. It parses CRSF packets coming from a UART into Copper messages and can optionally forward RC or link statistics back out over the same link.

## Interface

### Channels

- `rc_rx` (`RcChannelsPayload`): latest RC channel values from the receiver.
- `lq_rx` (`LinkStatisticsPayload`): downlink link-quality metrics.
- `rc_tx` / `lq_tx`: optional uplink of RC or link statistics toward the transmitter.

## Configuration

### Resources and configuration

The bridge expects a `serial` resource (anything implementing `embedded_io` `Read`/`Write` + `Send + Sync`). Point the bridge's `resources` map at a bundle entry.

**std builds**

Use `cu_linux_resources::LinuxResources` as the serial provider; it stores owned
resources in fixed slots like `<bundle>.serial_acm0` and `<bundle>.serial_usb0`.

For example, to back `serial_usb0`, configure serial slot `serial3_*`:

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `serial3_dev` | string | `/dev/ttyUSB0` | Device node for the `serial_usb0` resource slot. |
| `serial3_baudrate` | u32 | `115200` | UART baudrate for this slot. |
| `serial3_parity` | string | `none` | Parity for this slot (`none`, `odd`, `even`). |
| `serial3_stopbits` | u8 | `1` | Stop bits for this slot (`1` or `2`). |
| `serial3_timeout_ms` | u64 | `50` | Read timeout for this slot in milliseconds. |

```ron
resources: [
  (
    id: "radio",
    provider: "cu_linux_resources::LinuxResources",
    config: { "serial3_dev": "/dev/ttyUSB0", "serial3_baudrate": 420000 },
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
  ( id: "radio", provider: "my_app::resources::RadioBundle" ),
],
bridges: [
  (
    id: "crsf",
    type: "cu_crsf::CrsfBridge<SerialResource, SerialPortError>",
    resources: { serial: "radio.serial" },
    channels: [ Rx (id: "rc_rx"), Tx (id: "lq_tx") ],
  ),
],
```

See `examples/cu_elrs_bdshot_demo` for a full wiring that feeds CRSF RC channels into a BDShot bridge on the RP2350 reference board.

## Usage

Declare bridge types in app code, then reference them under `bridges` in `copperconfig.ron`.

## Compatibility

Depends on enabled transport features and target platform support for the selected backend.

## Links

- Crate path: `components/bridges/cu_crsf`
- docs.rs: <https://docs.rs/cu-crsf>
