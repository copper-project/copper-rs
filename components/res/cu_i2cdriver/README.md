# cu-i2cdriver

I2cDriver (a Usb to I2C device) resource to be used with `embedded_hal@1.0` (`embedded_hal::blocking::i2c`).


Depending on your OS, the port name may appear differently.

Linux: `/dev/ttyUSB0` or `/dev/ttyS0`; see the output of sudo dmesg for the
exact device name.

To avoid buffering delays, run `setserial /dev/ttyUSB0 low_latency`


MacOS: `/dev/ttyACM0`; see the output of sudo dmesg for the exact device
name.
Windows: `COM5`; see Device Manager for the exact name.

To avoid buffering delays, in Device Manager as Administrator, set the Latency to 1ms in "Advanced" under "Port Settings" - <https://www.instructables.com/Lampduino-an-8x8-RGB-Floor-Lamp/#step20>

## Provider

Use `cu_linux_resources::I2cDriverResources` in `copperconfig.ron`.

```ron
resources: [
  (
    id: "spidriver0",
    provider: "cu_spidriver::I2cDriverResources",
    config: {
      "dev": "/dev/ttyUSB0",
    },
  ),
],
```

## Resource

`I2cDriverResources` exposes single resources called `i2c`:

Bind tasks/bridges to this resource via `<bundle>.i2c` (for example `i2cdriver0.i2c`).
