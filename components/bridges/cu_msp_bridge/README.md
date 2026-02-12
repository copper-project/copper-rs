# cu-msp-bridge

## Overview

Bridge that combines the MSP source/sink tasks into one serial transport. It exposes a single TX channel (`requests`) that accepts batches of [`cu_msp_lib::structs::MspRequest`] messages and a single RX channel (`responses`) that yields [`MspResponse`] batches decoded from the line.

## Interface

Defines typed `Tx`/`Rx` channels and maps them to external transport routes.

## Configuration

### Resources and configuration

The bridge expects a `serial` resource. For std targets, use
`cu_linux_resources::LinuxResources` as the provider. It opens serial devices
and exposes fixed serial slots such as `<bundle>.serial_usb0`.

For `serial_usb0`, configure serial slot `serial3_*` in the Linux bundle:

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
    id: "linux",
    provider: "cu_linux_resources::LinuxResources",
    config: { "serial3_dev": "/dev/ttyUSB0", "serial3_baudrate": 115200 },
  ),
],
bridges: [
  (
    id: "msp_bridge",
    type: "cu_msp_bridge::CuMspBridgeStd",
    resources: { serial: "linux.serial_usb0" },
    channels: [ Tx (id: "requests"), Rx (id: "responses") ],
  ),
],
```

For embedded targets, provide your own bundle that moves the UART into the `ResourceManager` as an owned resource. If your UART type is not `Sync`, wrap it once at bundle registration time with `cu_linux_resources::Exclusive<T>`. See `examples/cu_elrs_bdshot_demo` and `examples/cu_msp_bridge_loopback` for end-to-end wiring and config mutation.

## Usage

Declare bridge types in app code, then reference them under `bridges` in `copperconfig.ron`.

## Compatibility

Depends on enabled transport features and target platform support for the selected backend.

## Links

- Crate path: `components/bridges/cu_msp_bridge`
- docs.rs: <https://docs.rs/cu-msp-bridge>
