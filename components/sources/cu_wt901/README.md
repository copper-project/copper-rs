# cu-wt901

## Overview

`cu-wt901` is a Copper `sources` crate.

## Interface

Publishes source payloads into the Copper graph on each runtime tick.

## Configuration

Use `config` keys and `resources` bindings for hardware/driver handles.

## Usage

Register the source type and wire its output to downstream tasks/sinks in `copperconfig.ron`.

## Compatibility

Depends on sensor interface availability, resource providers, and enabled features.

## Links

- Crate path: `components/sources/cu_wt901`
- docs.rs: <https://docs.rs/cu-wt901>

## Additional Notes

### WitMotion WT901 driver for Copper

This enables the communication with a WitMotion WT901 over I2C a Source task on Copper.

On Linux, WT901 consumes an owned I2C resource (`i2c`) from
`cu_linux_resources::LinuxResources`, for example:
`resources: { i2c: "<bundle>.i2c1" }`.

See the crate cu29 for more information about the Copper project.
