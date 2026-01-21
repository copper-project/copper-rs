## Cu-caterpillar: full example for Copper

This is an example for the Copper project running on a Raspberry Pi flipping in sequence 8 GPIO pins.
It allows to gauge the latency and performance of the Copper runtime with minimal user code.

See the crate cu29 for more information about the Copper project.

### Justfile commands

- `just cl` — dump the Copper list from a log at `logs/caterpillar.copper`.
- `just logreader` — extract logs from `logs/caterpillar.copper` into `../../target/debug/cu29_log_index`.
- `just resim` — rerun the logged mission from `logs/caterpillar.copper`.
- `just dag-logstats` — generate logstats and open an annotated DAG SVG for the current `copperconfig.ron`.
