# cu-linux-resources

Linux/host resource bundle for serial, I2C, and GPIO peripherals.

## Provider

Use `cu_linux_resources::LinuxResources` in `copperconfig.ron`.

```ron
resources: [
  (
    id: "linux",
    provider: "cu_linux_resources::LinuxResources",
    config: {
      "serial3_dev": "/dev/ttyUSB0",
      "serial3_baudrate": 420000,
      "serial3_parity": "none",
      "serial3_stopbits": 1,
      "serial3_timeout_ms": 100,
      "i2c1_dev": "/dev/i2c-1",
      "gpio_out0_pin": 23,
      "gpio_in0_pin": 24,
    },
  ),
],
```

## Fixed Resource Slots

`LinuxResources` exposes embedded-style fixed slots:

- Serial: `serial0`, `serial1`, `serial2`, `serial3`, `serial4`, `serial5`
- I2C: `i2c0`, `i2c1`, `i2c2`
- GPIO output: `gpio_out0`, `gpio_out1`, `gpio_out2`
- GPIO input: `gpio_in0`, `gpio_in1`, `gpio_in2`

Bind tasks/bridges to these slots via `<bundle>.<slot>` (for example `linux.serial3`).

## Config Keys

### Serial

Per-slot keys are `serial0..serial5`. Each slot maps to a fixed resource id:

- `serial0_*` -> `serial0`
- `serial1_*` -> `serial1`
- `serial2_*` -> `serial2`
- `serial3_*` -> `serial3`
- `serial4_*` -> `serial4`
- `serial5_*` -> `serial5`

**Note:** Serial and I2C slots are only initialized when explicitly configured. Unconfigured slots are skipped.

Supported keys per serial slot `serialN`:

- `serialN_dev` (`string`)
- `serialN_baudrate` (`u32`, default `115200`)
- `serialN_parity` (`string`: `none`, `odd`, `even`; default `none`)
- `serialN_stopbits` (`u8`: `1` or `2`; default `1`)
- `serialN_timeout_ms` (`u64`, default `50`)

### I2C

- `i2c0_dev` (no default - must be configured)
- `i2c1_dev` (no default - must be configured)
- `i2c2_dev` (no default - must be configured)

### GPIO

- Output pins: `gpio_out0_pin`, `gpio_out1_pin`, `gpio_out2_pin`
- Input pins: `gpio_in0_pin`, `gpio_in1_pin`, `gpio_in2_pin`

GPIO slots are created only for configured pins.

## Exclusive Wrapper

`cu_linux_resources::Exclusive<T>` is the bundle-layer wrapper for resources that are logically single-owner but must satisfy `Sync` for `ResourceManager::add_owned`.

Keeping that adaptation in the resource layer avoids consumer-side wrappers in bridges/tasks.
