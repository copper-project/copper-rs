# Copper Embedded Registry

## Overview

A shared registry for Copper components to register and retrieve serial ports, SPI buses, chip-select pins, and delay providers in no-std environments.

This library provides global slot-based registries for embedded devices so Copper applications can configure hardware in `main` and Copper components can pick them up later. It works in both std and no-std environments.

## Interface

Provides Rust APIs consumed by Copper components and applications.

## Configuration

Typically no direct runtime `copperconfig.ron` keys; configure through Rust API usage.

## Usage

Bridges and sources can take SPI/CS/delay or serial instances from the registry instead of constructing them directly, letting the board-support crate decide the concrete HAL types.

## Compatibility

### Features

- **no-std compatible**: Works in embedded environments
- **Type-safe**: Uses Rust's type system to ensure correct device types
- **Thread-safe**: Uses spin locks for concurrent access
- **Bounded**: Supports up to 8 slots per device class
- **Zero-cost when unused**: Minimal overhead

## Links

- Crate path: `components/libs/cu_embedded_registry`
- docs.rs: <https://docs.rs/cu-embedded-registry>

## Additional Notes

### Registering devices in your board-support crate

```rust
use cu_embedded_registry as reg;
use embedded_io::{Read, Write};
use embedded_hal as eh1;

// Register serial at slot 0
let serial: MySerial = /* ... */;
reg::register(0, serial)?;

// Register SPI + CS + delay for a sensor at slot 1
let spi: MySpi = /* ... */;
let cs: MyCsPin = /* ... */;
let delay: MyDelay = /* ... */;
reg::register_spi(1, spi)?;
reg::register_cs(1, cs)?;
reg::register_delay(1, delay)?;
```

### Consuming devices inside a Copper component

```rust
use cu_embedded_registry as reg;

let spi: MySpi = reg::take_spi(1).expect("SPI slot 1 not registered");
let cs: MyCsPin = reg::take_cs(1).expect("CS slot 1 not registered");
let delay: MyDelay = reg::take_delay(1).expect("Delay slot 1 not registered");
```

Slots are consumed when taken; re-register if the handle needs to be shared again.
