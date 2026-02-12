# cu-crsf

Serial bridge for TBS Crossfire / ExpressLRS receivers. It parses CRSF packets coming from a UART into Copper messages and can optionally forward RC or link statistics back out over the same link.

## Channels
- `rc_rx` (`RcChannelsPayload`): latest RC channel values from the receiver.
- `lq_rx` (`LinkStatisticsPayload`): downlink link-quality metrics.
- `rc_tx` / `lq_tx`: optional uplink of RC or link statistics toward the transmitter.

## Resources and configuration
The bridge expects a `serial` resource (anything implementing `embedded_io` `Read`/`Write` + `Send + Sync`). Point the bridge's `resources` map at a bundle entry.

**std builds**

Use `cu_linux_resources::LinuxResources` as the serial provider; it stores owned
resources in fixed slots like `<bundle>.serial0` through `<bundle>.serial5`.
For serial config keys (`serialN_dev`, `serialN_baudrate`, parity/stopbits/timeout), see
[`cu_linux_resources` README: Config Keys / Serial](../../res/cu_linux_resources/README.md#serial).

```ron
resources: [
  (
    id: "linux",
    provider: "cu_linux_resources::LinuxResources",
    config: { "serial3_dev": "/dev/ttyUSB0", "serial3_baudrate": 420000 },
  ),
],
bridges: [
  (
    id: "crsf",
    type: "cu_crsf::CrsfBridgeStd",
    resources: { serial: "linux.serial3" },
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
