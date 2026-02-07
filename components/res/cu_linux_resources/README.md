# cu-linux-resources

Shared resource bundle for host/Linux peripherals used by Copper components.

The bundle can register owned resources for:
- serial (`serial`)
- I2C (`i2c`)
- GPIO output (`gpio_out`)
- GPIO input (`gpio_in`)

## Provider

Use `cu_linux_resources::LinuxResources` as the resource provider in `copperconfig.ron`.

```ron
resources: [
  (
    id: "linux",
    provider: "cu_linux_resources::LinuxResources",
    config: {
      "serial_path": "/dev/ttyUSB0",
      "baudrate": 420000,
      "timeout_ms": 100,
    },
  ),
],
```

## Config keys

### Serial
- `serial_path` (string) or `device` (string)
- `baudrate` (u32, default `115200`)
- `timeout_ms` (u64, default `50`)

### I2C (Linux only)
- `i2c_bus` (string) or `bus` (string), e.g. `/dev/i2c-1`

### GPIO (Linux only)
- `gpio_out_pin` (u8) or `pin` (u8)
- `gpio_in_pin` (u8)

Each key is optional; only configured resources are created.
