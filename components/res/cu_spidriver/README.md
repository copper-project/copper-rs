# cu-spidriver

SpiDriver (a Usb to SPI device) resource to be used with `embedded_hal@1.0` (`embedded_hal::blocking::spi`).


## Provider

Use `cu_linux_resources::SpiDriverResources` in `copperconfig.ron`.

```ron
resources: [
  (
    id: "spidriver0",
    provider: "cu_spidriver::SpiDriverResources",
    config: {
      "dev": "/dev/ttyUSB0",
    },
  ),
],
```

## Resource

`SpiDriverResources` exposes the following resources:
- SpiBus: `spi`
- Chip Select: `chip_select`
- Pin A: `pin_a`
- Pin B: `pin_b`

Bind tasks/bridges to these resources via `<bundle>.spi` (for example `spidriver0.spi`).
