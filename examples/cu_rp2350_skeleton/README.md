## Cu-rp2350-skeleton: a minimal / template for using Copper on an RP2350 microcontroller

This repository provides a minimal template for using the Copper library on an RP2350 microcontroller. It includes basic setup and configuration to get you started quickly.

If you need to have your code std and no-std compatible a the same time (to be able to run your tasks on local simulations for example) also check out the cu-min-nostd.
In this example we will focus on a pure no-std environment.

### Building
- Firmware (thumbv8m): `cargo build --target thumbv8m.main-none-eabihf --no-default-features --features firmware --bin cu-blinky`
- Host helper (README text app): `cargo run --features host --bin main`

### Features
- Basic Copper library integration
- Minimal setup for RP2350 microcontroller
