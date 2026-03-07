# cu_bevymon

`cu_bevymon` is a Copper monitor frontend that renders the shared TUI monitor UI inside Bevy using `bevy_ratatui`.

It provides:

- `CuBevyMon`: a `CuMonitor` implementation backed by the shared monitor model
- `CuBevyMonPlugin`: a Bevy plugin that draws the monitor into a Bevy texture
- `CuBevyMonTexture` and `CuBevyMonPanel`: helpers for placing the monitor inside a Bevy UI layout

Design intent:

- reuse the same Ratatui rendering as `cu_consolemon`
- integrate into existing Bevy sims like `cu_rp_balancebot` and `cu_flight_controller`
- keep the rendering/backend split clean enough for later browser / wasm work
