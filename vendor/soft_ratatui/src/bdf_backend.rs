//! This module provides the `SoftBackend` implementation for the [`Backend`] trait.
//! It is used in the integration tests to verify the correctness of the library.

use crate::SoftBackend;
use crate::colors::*;
use crate::pixmap::RgbPixmap;
use crate::soft_backend::RasterBackend;
use copper_bdf_parser::*;
use ratatui_core::backend::Backend;
use ratatui_core::buffer::{Buffer, Cell};
use ratatui_core::layout::Rect;
use ratatui_core::style;
use rustc_hash::FxHashSet;

/// Uses bdf-parser for rendering from a .bdf (bitmap font), works pretty good
pub struct Bdf {
    font_regular: Font,
    font_italic: Option<Font>,
    font_bold: Option<Font>,
}
impl RasterBackend for Bdf {
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
        let mut font_to_use = &self.font_regular;
        let mut underline = false;
        let mut crossed_out = false;
        for modifier in rat_cell.modifier.iter() {
            match modifier {
                style::Modifier::BOLD => match &self.font_bold {
                    None => {}
                    Some(font) => font_to_use = font,
                },
                style::Modifier::DIM => {
                    (rat_fg, rat_bg) = (dim_rgb(rat_fg), dim_rgb(rat_bg));
                }
                style::Modifier::ITALIC => match &self.font_italic {
                    None => {}
                    Some(font) => font_to_use = font,
                },
                style::Modifier::UNDERLINED => underline = true,
                style::Modifier::SLOW_BLINK => {
                    always_redraw_list.insert((xik, yik));
                    if blinking_slow {
                        rat_fg = rat_bg;
                    }
                }
                style::Modifier::RAPID_BLINK => {
                    always_redraw_list.insert((xik, yik));
                    if blinking_fast {
                        rat_fg = rat_bg;
                    }
                }
                style::Modifier::REVERSED => {
                    (rat_bg, rat_fg) = (rat_fg, rat_bg);
                }
                style::Modifier::HIDDEN => {
                    rat_fg = rat_bg;
                }
                style::Modifier::CROSSED_OUT => crossed_out = true,
                _ => {}
            }
        }
        let begin_x = xik as usize * char_width;
        let begin_y = yik as usize * char_height;

        for y in 0..char_height {
            let y_pos = begin_y + y;
            let mut x_pos = begin_x;
            for _ in 0..char_width {
                rgb_pixmap.put_pixel(x_pos, y_pos, rat_bg);
                x_pos += 1;
            }
        }

        if underline {
            let y_pos = begin_y + char_height - 1;
            let mut x_pos = begin_x;
            for _ in 0..char_width {
                rgb_pixmap.put_pixel(x_pos, y_pos, rat_fg);
                x_pos += 1;
            }
        }
        if crossed_out {
            let y_pos = begin_y + char_height / 2;
            let mut x_pos = begin_x;
            for _ in 0..char_width {
                rgb_pixmap.put_pixel(x_pos, y_pos, rat_fg);
                x_pos += 1;
            }
        }

        let char = rat_cell.symbol().chars().next().unwrap();

        let ascent = font_to_use.metrics.ascent as i32;

        if let Some(glyph) = font_to_use.glyphs.get(char) {
            // glyph bitmap size (top-down rows in BDF BITMAP)
            let gw = glyph.bounding_box.size.x;
            let gh = glyph.bounding_box.size.y;
            let off_x = glyph.bounding_box.offset.x; // BBX x offset (signed)
            let off_y = glyph.bounding_box.offset.y; // BBX y offset (signed), *lower-left corner y relative to origin*

            let base_x = begin_x as i32; // top-left x of the cell in destination
            let base_y = begin_y as i32; // top-left y of the cell in destination

            // Iterate over the glyph bitmap (sx: left->right, sy: top->bottom)
            for sy in 0..gh {
                let sample_sy = sy as usize;

                for sx in 0..gw {
                    let sample_sx = sx as usize;

                    // only render pixels that are actually set in the glyph
                    if !matches!(glyph.pixel(sample_sx, sample_sy), Some(true)) {
                        continue;
                    }

                    // Map the glyph bitmap row (sy) to a y coordinate relative to the baseline:
                    // BDF: the lower-left corner of the bitmap sits at y = off_y (relative to origin/baseline).
                    // The glyph top row (sy = 0) has relative y:
                    //    y_rel = off_y + (gh - 1 - sy)
                    // We want destination (top-down) row index:
                    //    dst_y = base_y + (ascent - 1) - y_rel
                    //
                    // Simplified algebra gives:
                    //    dst_y = base_y + ascent - off_y - gh + sy
                    //
                    // (This places sy so that a glyph whose lower-left is on the baseline (off_y = 0)
                    //  will end with its bottom row on the baseline row: dst_y == base_y + ascent - 1.)
                    let dst_x_i32 = base_x + sx + off_x;
                    let dst_y_i32 = base_y + ascent - off_y - gh + sy;

                    // signed bounds check before casting to usize
                    if dst_x_i32 < base_x
                        || dst_y_i32 < base_y
                        || dst_x_i32 >= base_x + char_width as i32
                        || dst_y_i32 >= base_y + char_height as i32
                    {
                        continue;
                    }
                    let dst_x = dst_x_i32 as usize;
                    let dst_y = dst_y_i32 as usize;

                    rgb_pixmap.put_pixel(dst_x, dst_y, rat_fg);
                }
            }
        }
    }
}
impl SoftBackend<Bdf> {
    /// Creates a new Software Backend with the specified font configuration.
    ///
    /// # Arguments
    /// * `width` - Width of the terminal in character cells
    /// * `height` - Height of the terminal in character cells
    /// * `font_size` - Tuple of `(char_width, char_height)` specifying font dimensions in pixels
    /// * `font_regular` - BDF font data for regular text style
    /// * `font_bold` - Optional BDF font data for bold text style
    /// * `font_italic` - Optional BDF font data for italic text style
    ///
    /// # Examples
    /// ```rust
    /// let regular_font = include_str!("../assets/6x13.bdf"); // BDF font data
    /// let bold_font = include_str!("../assets/6x13_bold.bdf");    // Optional bold BDF font data
    /// let backend = SoftBackend::<Bdf>::new(80, 24, (8, 16), regular_font, Some(bold_font), None);
    /// ```
    pub fn new(
        width: u16,
        height: u16,
        font_size: (usize, usize),
        font_regular: &str,
        font_bold: Option<&str>,
        font_italic: Option<&str>,
    ) -> Self {
        let bdf_font_regular = Font::parse(font_regular).expect("COULD NOT PARSE BDF FONT DATA");
        let char_width = font_size.0;
        let char_height = font_size.1;

        let rgb_pixmap = RgbPixmap::new(char_width * width as usize, char_height * height as usize);

        let bdf_font_italic = font_italic.map(|x| Font::parse(x).expect("INVALID ITALIC FONT"));
        let bdf_font_bold = font_bold.map(|x| Font::parse(x).expect("INVALID BOLD FONT"));

        let mut return_struct = Self {
            buffer: Buffer::empty(Rect::new(0, 0, width, height)),
            cursor: false,
            cursor_pos: (0, 0),

            raster_backend: Bdf {
                font_regular: bdf_font_regular,
                font_italic: bdf_font_italic,
                font_bold: bdf_font_bold,
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
