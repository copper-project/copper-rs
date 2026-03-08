# cu_bevymon_demo

`cu_bevymon_demo` is a minimal Bevy example that shows:

- a simple Bevy 3D scene on the left
- a `cu_bevymon` monitor panel on the right
- a real Copper app generated from `copperconfig.ron`
- click-to-focus input routing between the sim and the monitor
- the shared `cu_bevymon::spawn_split_layout(...)` split shell used by the other examples too
- structured Copper logs flowing into the shared `LOG` tab

Controls:

- click the left panel to give keyboard and mouse input back to the sim
- click the right panel to hand input to the monitor
- when the sim has focus: `WASD` moves the cube, `Q/E` move it vertically, arrow keys orbit the camera, `H` also yaws left, and the mouse wheel zooms
- when the monitor has focus: use the normal `cu_tuimon` clicks, tabs, scrolling, and keybindings

The left panel is still a small fake Bevy scene, but the right panel is now backed by an actual Copper runtime with real tasks, real copperlists, and app-emitted `debug!` / `info!` / `warn!` / `error!` statements.

Build targets:

- Native desktop:
  `just`
  or
  `just bevy`
- Browser/wasm compile check:
  `cargo check -p cu-bevymon-demo --target wasm32-unknown-unknown`
- Browser demo with Trunk:
  `just web`

Browser notes:

- The wasm build uses a shared core `NoopLogger` instead of the file-backed unified logger.
- Structured Copper log macros are still initialized so the monitor model can run without host logger setup.
- The page shell lives in `examples/cu_bevymon_demo/index.html` and binds Bevy to `#bevy`.
