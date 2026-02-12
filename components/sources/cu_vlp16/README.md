# cu-vlp16

## Overview

`cu-vlp16` is a Copper `sources` crate.

## Interface

Publishes source payloads into the Copper graph on each runtime tick.

## Configuration

Use `config` keys and `resources` bindings for hardware/driver handles.

## Usage

Register the source type and wire its output to downstream tasks/sinks in `copperconfig.ron`.

## Compatibility

Depends on sensor interface availability, resource providers, and enabled features.

## Links

- Crate path: `components/sources/cu_vlp16`
- docs.rs: <https://docs.rs/cu-vlp16>

## Additional Notes

### VLP16 driver for Copper

This enables the communication with a Velodyne VLP16 a Source task on Copper.

See the crate cu29 for more information about the Copper project.
