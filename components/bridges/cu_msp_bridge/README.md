# cu-msp-bridge

Bridge that combines the MSP source/sink tasks into one serial transport.  It exposes a single TX channel (`requests`) that accepts batches of [`cu_msp_lib::structs::MspRequest`] messages and a single RX channel (`responses`) that yields [`MspResponse`] batches decoded from the line.

## Resources and configuration

The bridge expects a `serial` resource. For std targets, use
`cu_linux_resources::LinuxResources` as the provider. It opens the serial port
and exposes fixed serial slots such as `<bundle>.serial_usb0`.

| Key                | Type   | Default        | Description                                                     |
|--------------------|--------|----------------|-----------------------------------------------------------------|
| `tty_usb0_path`    | string | `/dev/ttyUSB0` | Path for the `serial_usb0` slot (can be a PTY).                |
| `serial_baudrate`  | u32    | `115200`       | Serial baud rate for all Linux serial slots.                   |
| `serial_timeout_ms`| u64    | `50`           | Read timeout passed to the serial driver in milliseconds.      |

```ron
resources: [
  (
    id: "fc",
    provider: "cu_linux_resources::LinuxResources",
    config: { "tty_usb0_path": "/dev/ttyUSB0", "serial_baudrate": 115200 },
  ),
],
bridges: [
  (
    id: "msp_bridge",
    type: "cu_msp_bridge::CuMspBridgeStd",
    resources: { serial: "fc.serial_usb0" },
    channels: [ Tx (id: "requests"), Rx (id: "responses") ],
  ),
],
```

For embedded targets, provide your own bundle that moves the UART into the `ResourceManager` as an owned resource. If your UART type is not `Sync`, wrap it once at bundle registration time with `cu_linux_resources::Exclusive<T>`. See `examples/cu_elrs_bdshot_demo` and `examples/cu_msp_bridge_loopback` for end-to-end wiring and config mutation.
