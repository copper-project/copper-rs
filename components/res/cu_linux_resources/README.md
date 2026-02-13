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
      "gpio0_pin": 23,
      "gpio0_direction": "output",
      "gpio0_bias": "pull_up",
      "gpio0_initial_level": "high",
      "gpio1_pin": 24,
      "gpio1_direction": "input",
      "gpio1_bias": "pull_down",
    },
  ),
],
```

## Fixed Resource Slots

`LinuxResources` exposes embedded-style fixed slots:

- Serial: `serial0`, `serial1`, `serial2`, `serial3`, `serial4`, `serial5`
- I2C: `i2c0`, `i2c1`, `i2c2`
- GPIO: `gpio0`, `gpio1`, `gpio2`, `gpio3`, `gpio4`, `gpio5`

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

Each slot `gpioN` (`N=0..5`) is enabled only when `gpioN_pin` is set.

Per-slot keys:

- `gpioN_pin` (`u8`, required when slot is configured)
- `gpioN_direction` (`string`, required when slot is configured): `input` or `output`
- `gpioN_bias` (`string`, optional, default `off`): `off`, `pull_up`, `pull_down`
- `gpioN_initial_level` (`string`, optional, output-only): `low`, `high`

Direction determines the concrete resource type:

- `direction: "input"` registers `LinuxInputPin`
- `direction: "output"` registers `LinuxOutputPin` (output mode with optional pull bias)

If `gpioN_initial_level` is omitted for an output slot, initialization preserves the current
GPIO output level.

Validation is strict for GPIO config:

- Invalid `direction`, `bias`, or `initial_level` values fail bundle build.
- Setting `gpioN_initial_level` on an input pin fails bundle build.
- Setting `gpioN_direction`/`bias`/`initial_level` without `gpioN_pin` fails bundle build.

## Exclusive Wrapper

`cu_linux_resources::Exclusive<T>` is the bundle-layer wrapper for resources that are logically single-owner but must satisfy `Sync` for `ResourceManager::add_owned`.

Keeping that adaptation in the resource layer avoids consumer-side wrappers in bridges/tasks.
