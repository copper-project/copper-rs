# cu_bevymon

`cu_bevymon` is a Copper monitor frontend that renders the shared TUI monitor UI inside Bevy using `bevy_ratatui`.

By default, the Bevy windowed monitor uses bundled JetBrains Mono Nerd Font Mono TTF files so the Bevy rendering is much closer to the console monitor than the stock `mono_8x13` backend.

It provides:

- `CuBevyMon`: a `CuMonitor` implementation backed by the shared monitor model
- `CuBevyMonPlugin`: a Bevy plugin that draws the monitor into a Bevy texture
- `CuBevyMonFontOptions`: font sizing for the bundled windowed TUI renderer
- `CuBevyMonTexture` and `CuBevyMonPanel`: the Bevy-backed monitor render surface
- `CuBevyMonSurfaceNode`, `CuBevyMonFocus`, and `CuBevyMonFocusBorder`: helpers for click-to-focus split layouts
- `spawn_split_layout(...)`: the shared left/right split shell used by the current examples
- `CuBevyMonViewportSurface`: a camera helper for syncing a 3D viewport to a Bevy UI panel

Design intent:

- reuse the same Ratatui rendering as `cu_consolemon`
- reuse the same shared `LOG` pane, including Copper live logs and optional `stderr` capture
- integrate into existing Bevy sims like `cu_rp_balancebot` and `cu_flight_controller`
- keep the rendering/backend split clean enough for later browser / wasm work

Recommended app structure:

- keep Copper config backend-agnostic by pointing it at an app-local monitor alias
- pick the actual monitor implementation in Rust with `cfg(feature = "...")`
- let the Bevy host app decide layout and focus ownership

The existing flight-controller pattern is the recommended one:

```ron
// copperconfig.ron
(
    id: "monitor",
    type: "tasks::monitor::AppMonitor",
)
```

```rust
// src/tasks/monitor.rs
#[cfg(all(feature = "sim", not(feature = "firmware")))]
pub type AppMonitor = cu_bevymon::CuBevyMon;

#[cfg(all(feature = "terminal_sim", not(feature = "firmware")))]
pub type AppMonitor = cu_consolemon::CuConsoleMon;

#[cfg(any(feature = "firmware", not(feature = "sim")))]
pub type AppMonitor = cu_logmon::CuLogMon;
```

That keeps RON stable while the build target decides whether the app gets a Bevy panel, a terminal TUI, or a lighter real-target monitor.

Standard split-view pattern:

1. Keep your sim scene camera app-owned.
2. Recommended: render that camera to an offscreen `RenderTarget::Image`.
3. Add `CuBevyMonPlugin::new(model)` to the Bevy app.
4. Create a dedicated `Camera2d` for the split-shell UI.
5. Call `spawn_split_layout(...)` with the scene camera entity and the monitor texture handle.
6. Add app-specific overlays like loading cards, help panels, or OSD widgets as children of the returned `sim_panel`.
7. Gate sim-control systems on `Res<CuBevyMonFocus>` while `cu_bevymon` routes monitor events to `cu_tuimon`.

Shared shell sketch:

```rust
let layout = spawn_split_layout(
    &mut commands,
    monitor_texture.0.clone(),
    CuBevyMonSplitLayoutConfig::new(scene_camera).with_ui_camera(ui_camera),
);

commands.entity(layout.sim_panel).with_children(|panel| {
    panel.spawn((
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(5.0),
            right: Val::Px(5.0),
            ..default()
        },
        Pickable::IGNORE,
    ));
});
```

The `cu_bevymon_demo`, `cu_rp_balancebot`, and `cu_flight_controller` examples all use that shared split shell now. The main app-specific choice is how the left-side scene camera is produced.

Render strategy guidance:

- Use the offscreen `RenderTarget::Image` + `ViewportNode` path when the sim already has its own UI overlays, picking, or camera behavior. This is the standardized path in the current examples.
- `CuBevyMonViewportSurface` still exists for simpler panel-clipped window cameras, but it is now the lower-level escape hatch, not the primary documented pattern.

Input model:

- window focus stays with Bevy
- logical panel focus is tracked by `CuBevyMonFocus`
- `cu_bevymon` translates Bevy events into backend-neutral `cu_tuimon::MonitorUiEvent`
- monitor bindings live in `cu_tuimon`, not in the Bevy adapter

Font behavior:

- the default bundled font set is JetBrains Mono Nerd Font Mono Light / SemiBold / LightItalic
- the default bundled font size is 16 px
- panel resize changes how many rows and columns fit; it does not change the glyph size
- use `CuBevyMonPlugin::with_font_size(...)` or `CuBevyMonPlugin::with_font_options(...)` to tune the Bevy-side font size
- use `CuBevyMonPlugin::with_font_options(...)` for full control
- the bundled font bytes live under `assets/fonts/`
- this improves glyph coverage for the powerline tabs, symbols, arrows, and box drawing used by `cu_tuimon`
- OpenType feature selection like Kitty's `+zero` flag is not currently exposed through this backend
