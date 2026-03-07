# cu_bevymon

`cu_bevymon` is a Copper monitor frontend that renders the shared TUI monitor UI inside Bevy using `bevy_ratatui`.

By default, the Bevy windowed monitor uses bundled JetBrains Mono Nerd Font Mono TTF files so the Bevy rendering is much closer to the console monitor than the stock `mono_8x13` backend.

It provides:

- `CuBevyMon`: a `CuMonitor` implementation backed by the shared monitor model
- `CuBevyMonPlugin`: a Bevy plugin that draws the monitor into a Bevy texture
- `CuBevyMonFontOptions`: font sizing for the bundled windowed TUI renderer
- `CuBevyMonTexture` and `CuBevyMonPanel`: the Bevy-backed monitor render surface
- `CuBevyMonSurfaceNode`, `CuBevyMonFocus`, and `CuBevyMonFocusBorder`: helpers for click-to-focus split layouts
- `CuBevyMonViewportSurface`: a camera helper for syncing a 3D viewport to a Bevy UI panel

Design intent:

- reuse the same Ratatui rendering as `cu_consolemon`
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

Side-by-side Bevy pattern:

1. Add `CuBevyMonPlugin::new(model)` to the Bevy app.
2. Mark the outer monitor wrapper with `CuBevyMonSurfaceNode(CuBevyMonSurface::Monitor)` and `CuBevyMonFocusBorder::new(...)`.
3. Put `CuBevyMonPanel` on the inner node that actually displays the monitor texture.
4. Mark the sim panel with `CuBevyMonSurfaceNode(CuBevyMonSurface::Sim)` and `CuBevyMonFocusBorder::new(...)`.
5. Mark the 3D camera with `CuBevyMonViewportSurface(CuBevyMonSurface::Sim)`.
6. Gate sim-control systems on `Res<CuBevyMonFocus>` while `cu_bevymon` routes monitor events to `cu_tuimon`.

Minimal layout sketch:

```rust
commands.spawn((
    Node { ..default() },
    CuBevyMonSurfaceNode(CuBevyMonSurface::Sim),
    CuBevyMonFocusBorder::new(CuBevyMonSurface::Sim),
));

commands.spawn((
    Node { ..default() },
    CuBevyMonSurfaceNode(CuBevyMonSurface::Monitor),
    CuBevyMonFocusBorder::new(CuBevyMonSurface::Monitor),
)).with_children(|parent| {
    parent.spawn((
        ImageNode::new(texture.0.clone()),
        Node { ..default() },
        CuBevyMonPanel,
    ));
});

commands.spawn((
    Camera3d::default(),
    CuBevyMonViewportSurface(CuBevyMonSurface::Sim),
));
```

Input model:

- window focus stays with Bevy
- logical panel focus is tracked by `CuBevyMonFocus`
- `cu_bevymon` translates Bevy events into backend-neutral `cu_tuimon::MonitorUiEvent`
- monitor bindings live in `cu_tuimon`, not in the Bevy adapter

Font behavior:

- the default bundled font set is JetBrains Mono Nerd Font Mono Light / SemiBold / LightItalic
- use `CuBevyMonPlugin::with_font_size(...)` or `CuBevyMonPlugin::with_font_options(...)` to tune the Bevy-side font size
- the bundled font bytes live under `assets/fonts/`
- this improves glyph coverage for the powerline tabs, symbols, arrows, and box drawing used by `cu_tuimon`
- OpenType feature selection like Kitty's `+zero` flag is not currently exposed through this backend
