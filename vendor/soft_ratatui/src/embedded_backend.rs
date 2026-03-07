//! This module provides the `SoftBackend` implementation for the [`Backend`] trait.
//! It is used in the integration tests to verify the correctness of the library.

use rustc_hash::FxHashSet;

use crate::colors::*;
use crate::pixmap::RgbPixmap;
use crate::soft_backend::RasterBackend;

use embedded_graphics::Drawable;

use embedded_graphics::mono_font::{MonoFont, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::prelude::{Point, RgbColor};
use embedded_graphics::text::Text;

use crate::SoftBackend;
use ratatui_core::backend::Backend;
use ratatui_core::buffer::{Buffer, Cell};
use ratatui_core::layout::Rect;
use ratatui_core::style;

/// PREFERRED: uses the embedded-graphics library for rendering, best when paired with embedded-graphics-unicodefonts (enabled by default)
pub struct EmbeddedGraphics {
    pub font_regular: MonoFont<'static>,
    /// Bold font.
    pub font_bold: Option<MonoFont<'static>>,
    /// Italic font.
    pub font_italic: Option<MonoFont<'static>>,
}

impl RasterBackend for EmbeddedGraphics {
    fn draw_cell(
        &mut self,
        xik: u16,
        yik: u16,
        rat_cell: &Cell,
        always_redraw_list: &mut FxHashSet<(u16, u16)>,
        blinking_fast: bool,
        blinking_slow: bool,
        char_width: usize,
        char_height: usize,
        rgb_pixmap: &mut RgbPixmap,
    ) {
        let mut rat_fg = rat_to_rgb(&rat_cell.fg, true);
        let mut rat_bg = rat_to_rgb(&rat_cell.bg, false);

        let mut style_builder = MonoTextStyleBuilder::new()
            .font(&self.font_regular)
            .text_color(Rgb888::WHITE)
            .background_color(Rgb888::BLACK);

        for modifier in rat_cell.modifier.iter() {
            style_builder = match modifier {
                style::Modifier::BOLD => match &self.font_bold {
                    None => style_builder,
                    Some(font) => style_builder.font(font),
                },
                style::Modifier::DIM => {
                    (rat_fg, rat_bg) = (dim_rgb(rat_fg), dim_rgb(rat_bg));
                    style_builder
                }
                style::Modifier::ITALIC => match &self.font_italic {
                    None => style_builder,
                    Some(font) => style_builder.font(font),
                },
                style::Modifier::UNDERLINED => style_builder.underline(),
                style::Modifier::SLOW_BLINK => {
                    always_redraw_list.insert((xik, yik));
                    if blinking_slow {
                        rat_fg = rat_bg;
                    }
                    style_builder
                }
                style::Modifier::RAPID_BLINK => {
                    always_redraw_list.insert((xik, yik));
                    if blinking_fast {
                        rat_fg = rat_bg;
                    }
                    style_builder
                }
                style::Modifier::REVERSED => {
                    (rat_bg, rat_fg) = (rat_fg, rat_bg);

                    style_builder
                }
                style::Modifier::HIDDEN => {
                    rat_fg = rat_bg;

                    style_builder
                }
                style::Modifier::CROSSED_OUT => style_builder.strikethrough(),
                _ => style_builder,
            }
        }

        style_builder = style_builder
            .text_color(Rgb888::new(rat_fg[0], rat_fg[1], rat_fg[2]))
            .background_color(Rgb888::new(rat_bg[0], rat_bg[1], rat_bg[2]));

        let begin_x = xik as usize * char_width;
        let begin_y = yik as usize * char_height;
        Text::with_baseline(
            rat_cell.symbol(),
            Point::new(begin_x as i32, begin_y as i32),
            style_builder.build(),
            embedded_graphics::text::Baseline::Top,
        )
        .draw(rgb_pixmap)
        .unwrap();
    }
}

impl SoftBackend<EmbeddedGraphics> {
    /// Creates a new software backend with the given monospaced font(s).
    ///
    /// (new width height font-regular font-bold font-italic) -> SoftBackend
    ///
    /// * width        : u16                  - Width of the terminal in cells
    /// * height       : u16                  - Height of the terminal in cells
    /// * font-regular : MonoFont<'static>    - Required base monospaced font
    /// * font-bold    : Option<MonoFont<'static>>   - Optional bold font
    /// * font-italic  : Option<MonoFont<'static>>   - Optional italic font
    ///
    /// # Examples
    /// ```rust
    /// use embedded_graphics_unicodefonts::{mono_8x13_atlas, mono_8x13_bold_atlas, mono_8x13_italic_atlas};
    /// use embedded_graphics::mono_font::{ascii::FONT_8X13, MonoFont};
    ///
    ///  let backend = SoftBackend::<EmbeddedGraphics>::new(100,50,font_regular,Some(font_bold),Some(font_italic));
    /// ```

    pub fn new(
        width: u16,
        height: u16,
        font_regular: MonoFont<'static>,
        font_bold: Option<MonoFont<'static>>,
        font_italic: Option<MonoFont<'static>>,
    ) -> Self {
        let char_width = font_regular.character_size.width as usize;
        let char_height = font_regular.character_size.height as usize;
        let rgb_pixmap = RgbPixmap::new(char_width * width as usize, char_height * height as usize);

        let mut return_struct = Self {
            buffer: Buffer::empty(Rect::new(0, 0, width, height)),
            cursor: false,
            cursor_pos: (0, 0),
            raster_backend: EmbeddedGraphics {
                font_regular: font_regular,
                font_bold,
                font_italic,
            },

            rgb_pixmap,

            char_width,
            char_height,

            blink_counter: 0,
            blinking_fast: false,
            blinking_slow: false,
            always_redraw_list: FxHashSet::default(),
        };
        _ = return_struct.clear();
        return_struct
    }
}
