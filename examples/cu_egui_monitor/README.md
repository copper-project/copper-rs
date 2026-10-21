# egui graph monitor

From this example directory, run `just` for the small live monitor or `just fc`
for the complete flight-controller graph:

```sh
cd examples/cu_egui_monitor
just
# Or: just fc
```

From the workspace root, `cargo run --release -p cu-egui-monitor` also runs the
small live demo.

The window displays a running 100 Hz `Tick → Scale → Output` Copper application,
CopperList/error counts, latest per-message processing times, and the SVG
configuration graph. **Stop runtime** stops Copper while leaving the graph open.
Closing the window stops and joins the runtime and rendering workers.
A component without a timestamped message displays `—` for its processing time.
The timing/counter samples are independent atomic reads, not a synchronized snapshot.

Scroll up/down to zoom around the pointer and click-drag to pan, like a map.
Pinch zoom, **Fit**, **100%**, the zoom slider, and both scrollbars are also
available. The initial view is at 100% zoom.
Enable **Tile boundaries** to label tiles and color ready tiles green and pending
tiles amber. Use the flight-controller preview below to exercise a larger graph.
The SVG is static: browser hover tooltips are not interactive in this viewer.

## Preview the flight controller

`just fc` uses the same executable to preview the larger graph without starting
its tasks or requiring their hardware/dependencies. Its equivalent command from
the workspace root is:

```sh
cargo run --release -p cu-egui-monitor -- \
  --config examples/cu_flight_controller/flight_controller.ron \
  --features sim --mission default
```

This reads the complete multi-Copper deployment and draws its subsystem
interconnects. Single-Copper configs also work. `--features` resolves conditional
RON includes; it does not build or run the previewed application. Preview mode
has no live runtime or telemetry panel.

## Structure

- `monitor.rs` implements `CuMonitor`. Construction allocates shared state once;
  callbacks update preallocated atomics without allocating, taking locks, copying
  payloads, or calling the GUI. Errors request runtime shutdown.
- `main.rs` owns the runtime thread and the egui main thread. The application
  supplies its already parsed `CuConfig` to `cu29_graph_view::render_config_svg`.
  Monitor metadata provides component identities and timings; no runtime API
  changes or config reloading inside the monitor are needed.
- `graph.rs` rasterizes SVG with Rust `resvg` on a worker. The UI draws cached GPU
  textures while the worker prepares missing tiles. Scrolling reuses tiles;
  zooming scales a cached level while a sharper level loads.

The cache holds at most 64 tiles of 512×512 pixels (64 MiB of RGBA data), plus a
small overview. The result queue holds at most four tiles, and the UI uploads at
most two per frame. Pending work is replaced when the viewport changes rather
than accumulating old scroll positions. Zoom resolutions use half-octave steps.

All SVG generation, font loading, layout, rasterization, and texture uploads run
outside Copper's execution loop. An unchanged graph is never regenerated for
telemetry updates. This example is host-only and adds no GUI dependency to the
runtime or embedded targets.

## Headless checks

```sh
cargo test -p cu-egui-monitor -p cu29-graph-view
cargo clippy -p cu-egui-monitor -p cu29-graph-view --all-targets -- -D warnings
```

Tests cover live monitor lifecycle/timing updates, the flight-controller preview,
tile reuse across scroll positions, zoom coverage, tile raster coordinates, and
worker shutdown. They do not require a display server.
