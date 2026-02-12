# cu-msp-lib

## Overview

`cu-msp-lib` is a Copper `libs` crate.

## Interface

Provides Rust APIs consumed by Copper components and applications.

## Configuration

Typically no direct runtime `copperconfig.ron` keys; configure through Rust API usage.

## Usage

Add this crate as a dependency and call its APIs from your component/application code.

## Compatibility

Check crate features for `std`/`no_std` and optional dependency support.

## Links

- Crate path: `components/libs/cu_msp_lib`
- docs.rs: <https://docs.rs/cu-msp-lib>

## Additional Notes

### Multiwii Serial Protocol

This is a spin on: https://docs.rs/multiwii_serial_protocol_v2/latest/multiwii_serial_protocol_v2/

The main difference are optimizations and cleanups.
