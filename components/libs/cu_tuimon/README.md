# cu_tuimon

`cu_tuimon` is the shared Ratatui UI library for Copper monitors.

It does not implement `CuMonitor` itself. Instead, it provides:

- `MonitorModel`: shared monitor state updated from Copper runtime data
- `MonitorUi`: reusable Ratatui rendering for system info, DAG, latency, bandwidth, and pool views
- shared scrolling and screen-selection behavior used by multiple frontends

Current intended consumers:

- `cu_consolemon`: terminal / crossterm frontend
- `cu_bevymon`: Bevy / `bevy_ratatui` frontend

This crate lives under `components/libs` because it is a reusable UI backend, not a standalone monitor component.
