# STM32 startup heap trace

This Cortex-M4 firmware runs on QEMU’s Netduino Plus 2 (STM32F405). A source
sends a u32 to a sink. Both builds use the same graph, clock, no-op logger and
96 KiB linked-list heap. The host records allocation and free call stacks.

From this directory, install the Rust target `thumbv7em-none-eabihf` and use
`qemu-system-arm` with LLVM’s `llvm-objcopy` and Python-enabled GDB with ARM support. Run:

```sh
cargo build --release
cp target/thumbv7em-none-eabihf/release/cu-stm32-heap-trace target/dynamic.elf
cargo build --release --features cu29/static-config
cp target/thumbv7em-none-eabihf/release/cu-stm32-heap-trace target/static.elf
for mode in dynamic static; do
    llvm-objcopy -O binary "target/$mode.elf" "target/$mode.bin"
    wc -c "target/$mode.bin"
    gdb -nx -batch "target/$mode.elf" -x trace.py
    gdb -nx -batch "target/$mode.elf" -x capacity.py
done
```

## QEMU results

| Bytes | Dynamic | Static |
| --- | ---: | ---: |
| Release flash image | 797,388 | 198,180 |
| Heap used before builder | 36 | 36 |
| Startup high-water mark | 5,152 | 1,088 |
| Heap used after first iteration | 1,840 | 1,088 |
| Minimum arena through first iteration | 5,768 | 1,112 |
| Free after first iteration | 96,464 | 97,216 |
| Largest successful 4-byte-aligned allocation | 94,416 | 97,192 |
| Free bytes outside that allocation | 2,048 | 24 |

Enabling `cu29/static-config` saves 599,208 bytes of flash (75.1%) on this two-task firmware. The raw image includes the vector table, code, read-only data, alignment padding and initial values copied to RAM. ELF debug information and zero-initialized RAM consume no flash in this image.

Each capacity trial boots a fresh emulator. Starting at the measured high-water mark, GDB tries every word-aligned
arena size until the first successful iteration. In the
96 KiB run, the firmware probes the backing heap directly after the first
iteration. GDB's trace bookkeeping runs on the host.

The dynamic peak occurs during monitor topology construction while the parsed
graph and effective RON are live. In the static build, the peak equals settled
usage. The largest allocation still leaves 24 bytes free. See the generated
`target/dynamic-heap-trace.txt` and `target/static-heap-trace.txt` for stacks.
The customer's 96/32 KiB report requires their graph, features and allocator.
