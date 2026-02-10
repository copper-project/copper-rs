# cu-linux-resources

Shared Linux/host resource bundle for serial, I2C, and GPIO peripherals.

## Provider

Use `cu_linux_resources::LinuxResources` in `copperconfig.ron`.

```ron
resources: [
  (
    id: "linux",
    provider: "cu_linux_resources::LinuxResources",
    config: {
      "tty_usb0_path": "/dev/ttyUSB0",
      "serial_baudrate": 420000,
      "serial_timeout_ms": 100,
      "i2c1_path": "/dev/i2c-1",
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
- `tty_acm0_path`, `tty_acm1_path`, `tty_acm2_path`
- `tty_usb0_path`, `tty_usb1_path`, `tty_usb2_path`
- `serial_baudrate` (`u32`, default `115200`)
- `serial_timeout_ms` (`u64`, default `50`)

If a serial path key is omitted, the default Linux device path is used (`/dev/ttyACM*`, `/dev/ttyUSB*`).

### I2C
- `i2c0_path` (default `/dev/i2c-0`)
- `i2c1_path` (default `/dev/i2c-1`)
- `i2c2_path` (default `/dev/i2c-2`)

### GPIO
- Output pins: `gpio_out0_pin`, `gpio_out1_pin`, `gpio_out2_pin`
- Input pins: `gpio_in0_pin`, `gpio_in1_pin`, `gpio_in2_pin`

GPIO slots are created only for configured pins.

## Exclusive Wrapper

`cu_linux_resources::Exclusive<T>` is the bundle-layer wrapper for resources that are logically single-owner but must satisfy `Sync` for `ResourceManager::add_owned`.

Keeping that adaptation in the resource layer avoids consumer-side wrappers in bridges/tasks.
