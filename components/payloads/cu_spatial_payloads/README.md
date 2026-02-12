# Copper Spatial payloads

## Overview

A set for components for spatial representation like poses, transforms etc...
It is made to be used with the Copper framework (cu29-copper)

## Interface

Defines payload/message types exchanged between Copper sources, tasks, sinks, and bridges.

## Configuration

No direct runtime config keys; select payload types in task/channel type definitions.

## Usage

Import payload types in component code and use them as I/O message contracts.

## Compatibility

Designed for cross-crate reuse; feature compatibility is defined in `Cargo.toml`.

## Links

- Crate path: `components/payloads/cu_spatial_payloads`
- docs.rs: <https://docs.rs/cu-spatial-payloads>
