# cu29-traits

Common traits and shared types used across Copper runtime and components.

This crate is designed to be usable both inside and outside Copper. It provides
core result/error types, logging stream traits, and common robotics-facing
interfaces that help decouple components.

## Features

- `std` (default): enables `std::error::Error` integration.
- `defmt`: enables embedded `defmt` support.

## no_std

`cu29-traits` supports `no_std` by disabling default features.
