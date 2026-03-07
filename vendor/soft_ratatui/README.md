# soft_ratatui

[![Crates.io](https://img.shields.io/crates/v/soft_ratatui.svg)](https://crates.io/crates/soft_ratatui)
[![Documentation](https://docs.rs/soft_ratatui/badge.svg)](https://docs.rs/soft_ratatui/latest/soft_ratatui/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/bevyengine/bevy/blob/master/LICENSE)
[![Downloads](https://img.shields.io/crates/d/soft_ratatui.svg)](https://crates.io/crates/soft_ratatui)

**Software rendering backend for [`ratatui`](https://github.com/ratatui/ratatui). No GPU required. TUI everywhere.**

![](new.avif)


Fast, portable, no-bloat.

- Optimized for speed, generally faster than running ratatui inside a terminal with crossterm. 120+ fps on normal workloads.
- Choose your own rendering backend: embedded-graphics, embedded-ttf, cosmic-text, bdf-parser
- Custom portable pixel rasterizer, outputs RGB/RGBA pixmaps, color-to-alpha support

## Feature Flags

soft_ratatui is highly modular. Enable only the backends and features you need to reduce binary size and dependencies.

| Feature | Enables | Description |
|---------|---------|-------------|
| `unicodefonts` | [`embedded_graphics_unicodefonts`] | Embedded-graphics fonts with Unicode support. Automatically enables `embedded-graphics`. Enabled by default. |
| `embedded-graphics` | [`EmbeddedGraphics`] | Uses embedded-graphics font atlases for TUI rendering. |
| `bdf-parser` | [`Bdf`] | Bitmap Distribution Format font support. |
| `embedded-ttf` | [`EmbeddedTTF`] | TrueType font rendering via RustType. Automatically enables `embedded-graphics`. |
| `cosmic-text` | [`CosmicText`] | Advanced text shaping, layout, and Unicode support using CosmicText engine. |

## Minimal example

```Rust
use soft_ratatui::embedded_graphics_unicodefonts::{
    mono_8x13_atlas, mono_8x13_bold_atlas, mono_8x13_italic_atlas,
};
use ratatui::Terminal;
use ratatui::widgets::{Block, Borders, Paragraph, Wrap};
use soft_ratatui::{EmbeddedGraphics, SoftBackend};

fn main() {
    let font_regular = mono_8x13_atlas();
    let font_italic = mono_8x13_italic_atlas();
    let font_bold = mono_8x13_bold_atlas();
    let backend = SoftBackend::<EmbeddedGraphics>::new(
        100,
        50,
        font_regular,
        Some(font_bold),
        Some(font_italic),
    );
    let mut terminal = Terminal::new(backend).unwrap();
    terminal.clear();

    terminal.draw(|frame| {
        let area = frame.area();
        let textik = format!("Hello soft! The window area is {}", area);
        frame.render_widget(
            Paragraph::new(textik)
                .block(Block::new().title("Ratatui").borders(Borders::ALL))
                .wrap(Wrap { trim: false }),
            area,
        );
    });
}

```

## Integrations

- [`egui`](https://github.com/emilk/egui) integration provided by [`egui_ratatui`](https://github.com/gold-silver-copper/egui_ratatui). Have a TUI inside your GUI!
- [`bevy_ratatui`](https://github.com/cxreiff/bevy_ratatui) integration allows you to turn an existing terminal app built with bevy_ratatui into a native or web app. The best way to build a terminal app!!
- [`bevy`](https://github.com/bevyengine/bevy) game engine examples provided in the repo, so you can create your own game UI or world textures with ratatui
- WASM compatible, deploy your ratatui application on the web!

## See also

- [`mousefood`](https://github.com/j-g00da/mousefood) - a no-std embedded-graphics backend for Ratatui!
- [`ratzilla`](https://github.com/orhun/ratzilla) - Build terminal-themed web applications with Rust and WebAssembly.
- [`ratatui-wgpu`](https://github.com/Jesterhearts/ratatui-wgpu) - A wgpu based rendering backend for ratatui.
- [`bevy_ratatui_camera`](https://github.com/cxreiff/bevy_ratatui_camera) - A bevy plugin for rendering your bevy app to the terminal using ratatui.

## Cool BDF fonts

- [`spleen`](https://github.com/fcambus/spleen) (works best, many sizes)
- [`cozette`](https://github.com/the-moonwitch/Cozette) (very pretty)

## License

Dual-licensed under **MIT** or **Apache 2.0**.
Pick whichever suits you.


## Status

Comments and suggestions are appreciated.
