# cu_tuimon

`cu_tuimon` is the shared Ratatui UI library for Copper monitors.

It does not implement `CuMonitor` itself. Instead, it provides:

- `MonitorModel`: shared monitor state updated from Copper runtime data
- `MonitorUi`: reusable Ratatui rendering for system info, DAG, latency, bandwidth, pool, and log views
- `MonitorUiEvent` and `MonitorUiKey`: backend-neutral monitor input events shared by multiple frontends
- `MonitorLogCapture`: shared live Copper log and optional `stderr` capture for monitor frontends
- shared scrolling and screen-selection behavior used by multiple frontends

Current intended consumers:

- `cu_consolemon`: terminal / crossterm frontend
- `cu_bevymon`: Bevy / `bevy_ratatui` frontend

Input ownership:

- `cu_tuimon` owns monitor behavior and bindings
- `cu_tuimon` owns the shared `LOG` pane, including live Copper log formatting
- frontend adapters translate raw backend events into `MonitorUiEvent`
- `cu_tuimon` never needs to know whether it is running in a terminal or in Bevy

This crate lives under `components/libs` because it is a reusable UI backend, not a standalone monitor component.
