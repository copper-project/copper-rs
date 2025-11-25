# Copper Serial Registry

A shared serial port registry for Copper components to register and retrieve serial ports in no-std environments.

## Overview

This library provides a global registry for serial ports that can be shared between different Copper components like bridges. It's designed to work in both std and no-std environments.

## Usage

### Registering a Serial Port

```rust
use cu_serial_registry;
use embedded_io::{Read, Write};

// Register a serial port at slot 0
let serial_port = /* your serial implementation */;
cu_serial_registry::register(0, serial_port)?;
```

### Retrieving a Serial Port

```rust
// Take the serial port from slot 0
let serial: Option<MySerialType> = cu_serial_registry::take(0);
```

## Features

- **no-std compatible**: Works in embedded environments
- **Type-safe**: Uses Rust's type system to ensure correct serial port types
- **Thread-safe**: Uses spin locks for concurrent access
- **Bounded**: Supports up to `MAX_SERIAL_SLOTS` (8) serial ports
- **Zero-cost when unused**: Minimal overhead

## Integration with Copper Bridges

This registry is used by Copper bridges like CRSF and MSP to allow sharing serial ports between components in no-std environments where traditional dependency injection isn't available.