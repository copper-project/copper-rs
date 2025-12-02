## Cu-caterpillar: full example for Copper

This is an example for the Copper project running on a Raspberry Pi flipping in sequence 8 GPIO pins.
It allows to gauge the latency and performance of the Copper runtime with minimal user code.

See the crate cu29 for more information about the Copper project.

### Justfile commands

- `just caterpillar-copperlist` — dump the Copper list from a log at `/tmp/caterpillar.copper`.
- `just caterpillar-rendercfg` — open the rendered `copperconfig.ron` via the built `copper-rendercfg` binary.
- `just caterpillar-logreader` — extract logs from `logs/caterpillar.copper` into `../../target/debug/copper_log_index`.
- `just caterpillar-resim` — rerun the logged mission from `logs/caterpillar.copper`.
- `just caterpillar-cross-armv7` — cross-compile for armv7 and scp the binary to `copper7:copper`.
