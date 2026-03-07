//! This module provides the `SoftBackend` implementation for the [`Backend`] trait.
//! It is used in the integration tests to verify the correctness of the library.

use crate::colors::*;
use crate::pixmap::RgbPixmap;
use crate::soft_backend::RasterBackend;
use embedded_ttf::FontTextStyleBuilder;
use rustc_hash::FxHashSet;

use embedded_graphics::Drawable;

use crate::SoftBackend;

use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::prelude::{Dimensions, Point, RgbColor};
use embedded_graphics::text::Text;
use ratatui_core::backend::Backend;
use ratatui_core::buffer::{Buffer, Cell};
use ratatui_core::layout::Rect;
use ratatui_core::style;

/// Uses embedded-ttf + embedded-graphics for rendering, generally better than cosmic-text
pub struct EmbeddedTTF {
    pub font_regular: rusttype::Font<'static>,
    /// Bold font.
    pub font_bold: Option<rusttype::Font<'static>>,
    /// Italic font.
    pub font_italic: Option<rusttype::Font<'static>>,
    pub font_size: u32,
}
impl RasterBackend for EmbeddedTTF {
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
        let begin_x = xik as usize * char_width;
        let begin_y = yik as usize * char_height;

        let mut style_builder = FontTextStyleBuilder::new(self.font_regular.clone())
            .font_size(self.font_size)
            .text_color(Rgb888::WHITE);
        for modifier in rat_cell.modifier.iter() {
            style_builder = match modifier {
                style::Modifier::BOLD => match &self.font_bold {
                    None => style_builder,
                    Some(font) => FontTextStyleBuilder::new(font.clone())
                        .font_size(self.font_size)
                        .text_color(Rgb888::WHITE),
                },

                style::Modifier::ITALIC => match &self.font_italic {
                    None => style_builder,
                    Some(font) => FontTextStyleBuilder::new(font.clone())
                        .font_size(self.font_size)
                        .text_color(Rgb888::WHITE),
                },
                _ => style_builder,
            }
        }

        for modifier in rat_cell.modifier.iter() {
            style_builder = match modifier {
                style::Modifier::DIM => {
                    (rat_fg, rat_bg) = (dim_rgb(rat_fg), dim_rgb(rat_bg));
                    style_builder
                }

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

        for y in 0..char_height {
            let y_pos = begin_y + y;
            let mut x_pos = begin_x;
            for _ in 0..char_width {
                rgb_pixmap.put_pixel(x_pos, y_pos, rat_bg);
                x_pos += 1;
            }
        }
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
impl SoftBackend<EmbeddedTTF> {
    /// Creates a new software backend with the given TrueType font(s).
    ///
    /// (new width height font-size font-regular font-bold font-italic) -> SoftBackend
    ///
    /// * width        : u16                        - Width of the terminal in cells
    /// * height       : u16                        - Height of the terminal in cells
    /// * font-size    : u32                        - Font size in pixels
    /// * font-regular : rusttype::Font<'static>    - Required base TrueType font
    /// * font-bold    : Option<rusttype::Font<'static>>   - Optional bold font
    /// * font-italic  : Option<rusttype::Font<'static>>   - Optional italic font
    ///
    /// The character cell size (in pixels) is derived automatically from the given font and font size.
    /// The underlying pixel buffer is allocated to fit the full terminal area in pixels.
    ///
    /// # Examples
    /// ```rust
    /// use rusttype::Font;
    ///
    /// let font_regular = Font::try_from_bytes(include_bytes!("../assets/iosevka.ttf")).unwrap();
    /// let backend = SoftBackend::<EmbeddedTTF>::new(80, 60, 16, font_regular, None, None);
    /// ```

    pub fn new(
        width: u16,
        height: u16,
        font_size: u32,
        font_regular: rusttype::Font<'static>,
        font_bold: Option<rusttype::Font<'static>>,
        font_italic: Option<rusttype::Font<'static>>,
    ) -> Self {
        let style = FontTextStyleBuilder::new(font_regular.clone())
            .font_size(font_size)
            .text_color(Rgb888::WHITE)
            .build();
        let textik = Text::new("█", Point::new(0, 0), style);
        let char_width = textik.bounding_box().size.width as usize;
        let char_height = textik.bounding_box().size.height as usize;
        let rgb_pixmap = RgbPixmap::new(char_width * width as usize, char_height * height as usize);

        let mut return_struct = Self {
            buffer: Buffer::empty(Rect::new(0, 0, width, height)),
            cursor: false,
            cursor_pos: (0, 0),
            raster_backend: EmbeddedTTF {
                font_regular,
                font_bold,
                font_italic,
                font_size,
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
    /// Sets a new font size for the terminal image.
    /// This will recreate the pixmap and do a full redraw. Do not run every frame.
    pub fn set_font_size(&mut self, font_size: u32) {
        let style = FontTextStyleBuilder::new(self.raster_backend.font_regular.clone())
            .font_size(font_size)
            .text_color(Rgb888::WHITE)
            .build();
        let textik = Text::new("█", Point::new(0, 0), style);
        let char_width = textik.bounding_box().size.width as usize;
        let char_height = textik.bounding_box().size.height as usize;

        self.char_width = char_width;
        self.char_height = char_height;
        self.raster_backend.font_size = font_size;
        self.rgb_pixmap = RgbPixmap::new(
            self.char_width * self.buffer.area.width as usize,
            self.char_height * self.buffer.area.height as usize,
        );

        self.redraw();
    }
}
