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

- Serial: `serial_acm0`, `serial_acm1`, `serial_acm2`, `serial_usb0`, `serial_usb1`, `serial_usb2`
- I2C: `i2c0`, `i2c1`, `i2c2`
- GPIO output: `gpio_out0`, `gpio_out1`, `gpio_out2`
- GPIO input: `gpio_in0`, `gpio_in1`, `gpio_in2`

Bind tasks/bridges to these slots via `<bundle>.<slot>` (for example `linux.serial_usb0`).

## Config Keys

### Serial

Per-slot keys are anonymous (`serial0..serial5`). Each slot maps to a fixed resource id and default device:

- `serial0_*` -> `serial_acm0` (default dev `/dev/ttyACM0`)
- `serial1_*` -> `serial_acm1` (default dev `/dev/ttyACM1`)
- `serial2_*` -> `serial_acm2` (default dev `/dev/ttyACM2`)
- `serial3_*` -> `serial_usb0` (default dev `/dev/ttyUSB0`)
- `serial4_*` -> `serial_usb1` (default dev `/dev/ttyUSB1`)
- `serial5_*` -> `serial_usb2` (default dev `/dev/ttyUSB2`)

Supported keys per serial slot `serialN`:

- `serialN_dev` (`string`)
- `serialN_baudrate` (`u32`, default `115200`)
- `serialN_parity` (`string`: `none`, `odd`, `even`; default `none`)
- `serialN_stopbits` (`u8`: `1` or `2`; default `1`)
- `serialN_timeout_ms` (`u64`, default `50`)

### I2C

- `i2c0_dev` (default `/dev/i2c-0`)
- `i2c1_dev` (default `/dev/i2c-1`)
- `i2c2_dev` (default `/dev/i2c-2`)

### GPIO

- Output pins: `gpio_out0_pin`, `gpio_out1_pin`, `gpio_out2_pin`
- Input pins: `gpio_in0_pin`, `gpio_in1_pin`, `gpio_in2_pin`

GPIO slots are created only for configured pins.

## Exclusive Wrapper

`cu_linux_resources::Exclusive<T>` is the bundle-layer wrapper for resources that are logically single-owner but must satisfy `Sync` for `ResourceManager::add_owned`.

Keeping that adaptation in the resource layer avoids consumer-side wrappers in bridges/tasks.
