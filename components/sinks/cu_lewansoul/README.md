# cu-lewansoul

## Overview

`cu-lewansoul` is a Copper `sinks` crate.

## Interface

Consumes upstream payloads and performs external side effects (actuation, IO, or export).

## Configuration

Set sink behavior through `config` and bind external handles through `resources` when needed.

## Usage

Wire upstream producers to this sink in `copperconfig.ron` and set required config keys.

## Compatibility

Depends on output transport/hardware availability and selected Cargo features.

## Links

- Crate path: `components/sinks/cu_lewansoul`
- docs.rs: <https://docs.rs/cu-lewansoul>

## Additional Notes

### Lewansoul Serial Bus driver for Copper

This enables the communication with the Lewansoul bus servos as a Sink task on Copper.

`Lewansoul` can consume an owned serial resource via
`resources: { serial: \"<bundle>.serial_usb0\" }` from
`cu_linux_resources::LinuxResources`.

See the crate [cu29](https://crates.io/crates/cu29) for more information about the Copper project.
