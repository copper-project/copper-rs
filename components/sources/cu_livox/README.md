# cu-livox

## Overview

Driver for the Livox Tele15 for Copper (check the crate cu29)

## Interface

Publishes source payloads into the Copper graph on each runtime tick.

## Configuration

Use `config` keys and `resources` bindings for hardware/driver handles.

## Usage

Register the source type and wire its output to downstream tasks/sinks in `copperconfig.ron`.

## Compatibility

Depends on sensor interface availability, resource providers, and enabled features.

## Links

- Crate path: `components/sources/cu_livox`
- docs.rs: <https://docs.rs/cu-livox>
