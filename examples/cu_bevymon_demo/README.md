# cu_bevymon_demo

`cu_bevymon_demo` is a minimal Bevy example that shows:

- a simple Bevy 3D scene on the left
- a `cu_bevymon` monitor panel on the right
- a fake Copper DAG feeding live monitor data
- click-to-focus input routing between the sim and the monitor
- shared `cu_bevymon` helpers for panel focus borders and camera viewport sizing

Controls:

- click the left panel to give keyboard and mouse input back to the sim
- click the right panel to hand input to the monitor
- when the sim has focus: `WASD` moves the cube, `Q/E` move it vertically, arrow keys orbit the camera, `H` also yaws left, and the mouse wheel zooms
- when the monitor has focus: use the normal `cu_tuimon` clicks, tabs, scrolling, and keybindings

The example exists to validate the reusable side-by-side layout model before wiring `cu_bevymon` into the larger existing Bevy simulations.
