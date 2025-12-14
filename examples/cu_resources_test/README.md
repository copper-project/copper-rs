# cu-resources-test

Small, host-only Copper app that stress-tests resource bindings:

- Multiple missions (`A` / `B`) with different bundles/config.
- Owned vs shared resources.
- Bridge resources.
- Mission-scoped bundles and tasks.

Logging is disabled in `copperconfig.ron` to keep the demo lightweight and avoid allocator/space limits while exercising resources. The runtime still allocates a unified log slab (default 10 MiB via `SLAB_SIZE: None` in `main.rs`).

## Top-to-bottom resource flow

1) **Config declares bundles**  
`copperconfig.ron` lists four bundles:
   - `global` (`GlobalBundle`): one shared `log` store used everywhere.
   - `board_a` and `board_b` (`BoardBundle`): each mission gets its own bus + counter + tag, with per-mission config (`label`, `offset`).
   - `extras_b` (`ExtraBundle`): only mission B, providing a `note` string.

   Tasks/bridges bind to concrete paths like `board_a.bus` or `global.log`. The macro keeps only the bindings relevant to the active mission.

2) **Bundles build concrete resources**  
Implementations live in `src/resources.rs`:
   - `BoardBundle`:
     - `bus` → shared `Arc<SharedBus>` (labeled, holds last value).
     - `counter` → owned `OwnedCounter` (mission-specific offset).
     - `tag` → shared `Arc<String>` (same label, reused by tasks/bridge).
   - `GlobalBundle`:
     - `log` → shared `Arc<GlobalLog>` (mutexed Vec for demo).
   - `ExtraBundle`:
     - `note` → owned `String`.

   Each bundle receives the slice of `ResourceDecl` for its id and inserts the right keys into the `ResourceManager`.

3) **Tasks/bridges pull resources**  
Resource binding structs implement `ResourceBindings` manually:
   - `SensorTask` grabs an owned counter, shared bus/tag, and shared global log. It increments the counter, updates the bus, logs, and emits `BusReading`.
   - `InspectorTask` borrows bus + note + global log; it logs any incoming `BusReading` (or polls the bus if empty).
   - `StatsBridge` borrows bus/tag/global and drives its `stats_tx` channel with the current bus value, also logging usage.

   Owned resources move into the task; shared resources stay managed and are borrowed per tick.

4) **Missions exercise variations**  
Mission A uses `board_a` (offset +10) and bridge `stats_a`. Mission B uses `board_b` (offset -3) plus an extra note from `extras_b` and bridge `stats_b`. The same code runs in both missions; the config drives which concrete resources are wired in.

## Running

```bash
cargo run -p cu-resources-test -- --mission A   # mission A graph
cargo run -p cu-resources-test -- --mission B   # mission B graph
```

Logs end up in `logs/cu_resources_test.copper`; the global log is also appended in memory (see `GlobalLog`).
