# cu_bevymon_demo

`cu_bevymon_demo` is a minimal Bevy example that shows:

- a simple Bevy 3D scene on the left
- a `cu_bevymon` monitor panel on the right
- a real Copper app generated from `copperconfig.ron`
- click-to-focus input routing between the sim and the monitor
- shared `cu_bevymon` helpers for panel focus borders and camera viewport sizing
- structured Copper logs flowing into the shared `LOG` tab

Controls:

- click the left panel to give keyboard and mouse input back to the sim
- click the right panel to hand input to the monitor
- when the sim has focus: `WASD` moves the cube, `Q/E` move it vertically, arrow keys orbit the camera, `H` also yaws left, and the mouse wheel zooms
- when the monitor has focus: use the normal `cu_tuimon` clicks, tabs, scrolling, and keybindings

The left panel is still a small fake Bevy scene, but the right panel is now backed by an actual Copper runtime with real tasks, real copperlists, and app-emitted `debug!` / `info!` / `warn!` / `error!` statements.
