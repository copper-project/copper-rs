# cu-rp2350-skeleton

## Overview

`cu-rp2350-skeleton` is a runnable Copper example.

## Prerequisites

- Rust stable toolchain
- Any hardware or services referenced by this example

## Run

### Building

- Firmware (thumbv8m): `cargo build --target thumbv8m.main-none-eabihf --no-default-features --features firmware --bin cu-blinky`
- Host helper (README text app): `cargo run --features host --bin main`

## Expected Output

Expect successful startup logs and normal task-loop execution without panics.

## Links

- Example path: `examples/cu_rp2350_skeleton`
- Build/deploy guide: <https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application>

## Additional Notes

### Cu-rp2350-skeleton: a minimal / template for using Copper on an RP2350 microcontroller

This repository provides a minimal template for using the Copper library on an RP2350 microcontroller. It includes basic setup and configuration to get you started quickly.

If you need to have your code std and no-std compatible a the same time (to be able to run your tasks on local simulations for example) also check out the cu-min-nostd.
In this example we will focus on a pure no-std environment.

### Features

- Basic Copper library integration
- Minimal setup for RP2350 microcontroller
