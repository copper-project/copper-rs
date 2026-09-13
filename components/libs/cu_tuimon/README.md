# cu_tuimon

`cu_tuimon` is the shared Ratatui UI library for Copper monitors.

The `DAG` and `HOP` tabs offer two views of the same `MonitorTopology` and
`MonitorModel`. DAG lays out the full graph on a scrollable canvas. HOP shows
one node, its incoming and outgoing connections, and live component status and timing
inside the supplied content rectangle. Both work in terminal and Bevy monitors.

The `dag` and `neighbors` Cargo features independently enable their renderer, state,
and tab. Both are enabled by default. `cu-consolemon` and `cu-bevymon` forward these
features; for example, use `default-features = false, features = ["neighbors"]` on a
monitor dependency to select only the HOP view. The initial screen is DAG when
enabled, otherwise HOP, otherwise Latency. Tab numbers follow the enabled tabs.

Select `HOP` in the monitor’s top bar (content area: at least 100 × 16).
Node names use the same symbols as DAG throughout HOP: `◈` source, `⚙` task,
`⭳` sink, and `⇆` bridge.

- Left/Right or Tab selects a list; Up/Down selects a row. Nodes activate immediately.
- Enter or a row click follows a neighbor. Backspace returns to the previous node.
- Press `/` to search Nodes. While searching, typing (including numbers and `q`)
  edits the filter and Backspace deletes characters. Enter applies the filter and
  ends search; Escape clears it. Selecting another list also ends search.
  Outside search, number keys switch tabs.
- The mouse wheel selects rows in the hovered list, including the space around centered
  neighbors. Each connection shows the neighboring node name above its short message
  type. Named ports appear beside their node; ports on the focused node appear with
  an arrow on the message line. Bridge channels retain their names. Green identifies
  outgoing connections and mauve incoming connections. Dotted rail extensions mark
  connections above or below the visible list.
- Ctrl-C quits the console monitor from any list; `q` quits outside search.

`MonitorUi` owns navigation and event dispatch. Backends translate Enter, Backspace,
Tab and Escape into `MonitorUiKey`; pointer wheels use `MonitorUiEvent::ScrollAt` with
cell coordinates. Coordinate-free `Scroll` operates on the selected list.
Topology and port labels are indexed once when the view is constructed. Runtime
component IDs are resolved separately from topology order. Rendering reads current
status and timing; errors stay visible until `MonitorModel::clear_component_error`.

Run `just tuimon-check` for shared-library regressions, feature combinations, and
monitor frontend checks.

It does not implement `CuMonitor` itself. Instead, it provides:

- `MonitorModel`: shared monitor state updated from Copper runtime data
- `MonitorUi`: reusable Ratatui rendering for system info, DAG, neighbors, latency, bandwidth, pool, and log views
- `MonitorUiEvent` and `MonitorUiKey`: backend-neutral monitor input events shared by multiple frontends
- `MonitorLogCapture`: shared live Copper log and optional `stderr` capture for monitor frontends
- shared scrolling and screen-selection behavior used by multiple frontends

Current intended consumers:

- `cu_consolemon`: terminal / crossterm frontend
- `cu_bevymon`: Bevy / `bevy_ratatui` frontend

Both frontends share the 😼 Copper mascot at the top right of every monitor tab.
The separate [UDP telemetry dashboard](../../../examples/cu_logstream_demo/) also
shows it in its full and compact headers. These are the three Copper Ratatui
frontends; vendored widget examples are upstream demos rather than Copper UIs.

Input ownership:

- `cu_tuimon` owns monitor behavior and bindings
- `cu_tuimon` owns the shared `LOG` pane, including live Copper log formatting
- frontend adapters translate raw backend events into `MonitorUiEvent`
- `cu_tuimon` never needs to know whether it is running in a terminal or in Bevy

This crate lives under `components/libs` because it is a reusable UI backend, not a standalone monitor component.
