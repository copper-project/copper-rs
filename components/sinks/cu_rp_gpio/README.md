# cu-rp-gpio

## Overview

`cu-rp-gpio` is a Copper `sinks` crate.

## Interface

Consumes upstream payloads and performs external side effects (actuation, IO, or export).

## Configuration

Set sink behavior through `config` and bind external handles through `resources` when needed.

## Usage

Wire upstream producers to this sink in `copperconfig.ron` and set required config keys.

## Compatibility

Depends on output transport/hardware availability and selected Cargo features.

## Links

- Crate path: `components/sinks/cu_rp_gpio`
- docs.rs: <https://docs.rs/cu-rp-gpio>

## Additional Notes

### Raspberry GPIO driver for Copper

This enables the communication with the GPIO pins of a raspberry Pi as a Sink task on Copper.

`RPGpio` can take an owned GPIO pin resource via `resources: { pin: \"<bundle>.gpio_out0\" }`
from `cu_linux_resources::LinuxResources`.

See the crate [cu29](https://crates.io/crates/cu29) for more information about the Copper project.
