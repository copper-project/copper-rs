## Justfile commands

- `just mk-cu29-sdcard dev=/dev/sdX` — partition/format an SD card for Cu29 logging (destroys data).
- `just cross-armv7-deploy` — build all release binaries for `armv7-unknown-linux-gnueabihf` and scp them plus `copperconfig.ron` to `copper7:copper/`.
- `just cross-riscv64-deploy` — build all release binaries for `riscv64gc-unknown-linux-gnu` and scp them plus `copperconfig.ron` to `copperv:copper/`.
