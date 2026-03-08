use std::ops::{Deref, DerefMut};

use bevy::prelude::{Resource, Result, Vec2};
use bevy_ratatui::{RatatuiPlugins, context::TerminalContext};
use ratatui::Terminal;
use soft_ratatui::rusttype::Font;
use soft_ratatui::{EmbeddedTTF, SoftBackend};

const DEFAULT_TERMINAL_COLS: u16 = 100;
const DEFAULT_TERMINAL_ROWS: u16 = 50;
const DEFAULT_FONT_SIZE_PX: u32 = 16;

const FONT_REGULAR: &[u8] = include_bytes!("../assets/fonts/CopperMono-Light.ttf");
const FONT_BOLD: &[u8] = include_bytes!("../assets/fonts/CopperMono-SemiBold.ttf");
const FONT_ITALIC: &[u8] = include_bytes!("../assets/fonts/CopperMono-LightItalic.ttf");

#[derive(Resource, Clone, Debug)]
pub struct CuBevyMonFontOptions {
    pub size_px: u32,
}

impl CuBevyMonFontOptions {
    pub fn new(size_px: u32) -> Self {
        Self { size_px }
    }

    fn clamped_size_px(&self) -> u32 {
        self.size_px.max(8)
    }
}

impl Default for CuBevyMonFontOptions {
    fn default() -> Self {
        Self::new(DEFAULT_FONT_SIZE_PX)
    }
}

#[derive(Resource)]
pub struct CuBevyMonTerminal(pub Terminal<SoftBackend<EmbeddedTTF>>);

impl CuBevyMonTerminal {
    pub fn from_options(options: &CuBevyMonFontOptions) -> Result<Self> {
        let font_size = options.clamped_size_px();
        let backend = SoftBackend::<EmbeddedTTF>::new(
            DEFAULT_TERMINAL_COLS,
            DEFAULT_TERMINAL_ROWS,
            font_size,
            load_font(FONT_REGULAR, "CopperMono-Light.ttf")?,
            Some(load_font(FONT_BOLD, "CopperMono-SemiBold.ttf")?),
            Some(load_font(FONT_ITALIC, "CopperMono-LightItalic.ttf")?),
        );
        Ok(Self(Terminal::new(backend)?))
    }
}

impl Deref for CuBevyMonTerminal {
    type Target = Terminal<SoftBackend<EmbeddedTTF>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for CuBevyMonTerminal {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl TerminalContext<SoftBackend<EmbeddedTTF>> for CuBevyMonTerminal {
    fn init() -> Result<Self> {
        Self::from_options(&CuBevyMonFontOptions::default())
    }

    fn restore() -> Result<()> {
        Ok(())
    }

    fn configure_plugin_group(
        _group: &RatatuiPlugins,
        builder: bevy::app::PluginGroupBuilder,
    ) -> bevy::app::PluginGroupBuilder {
        builder
    }
}

pub fn sync_terminal_to_panel(terminal: &mut CuBevyMonTerminal, panel_size: Vec2) {
    if panel_size.x <= 1.0 || panel_size.y <= 1.0 {
        return;
    }

    let char_width = terminal.backend().char_width.max(1) as f32;
    let char_height = terminal.backend().char_height.max(1) as f32;
    let cols = (panel_size.x / char_width).floor().max(1.0) as u16;
    let rows = (panel_size.y / char_height).floor().max(1.0) as u16;

    let current_area = terminal.backend().buffer().area;
    if current_area.width == cols && current_area.height == rows {
        return;
    }

    terminal.backend_mut().resize(cols, rows);
}

fn load_font(bytes: &'static [u8], label: &str) -> Result<Font<'static>> {
    Font::try_from_bytes(bytes).ok_or_else(|| {
        std::io::Error::other(format!("failed to load bundled cu_bevymon font {label}")).into()
    })
}
