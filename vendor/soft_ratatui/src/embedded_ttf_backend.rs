//! This module provides the `SoftBackend` implementation for the [`Backend`] trait.
//! It is used in the integration tests to verify the correctness of the library.

use crate::colors::*;
use crate::pixmap::RgbPixmap;
use crate::soft_backend::RasterBackend;
use rustc_hash::FxHashSet;

use crate::SoftBackend;

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
        let mut font_to_use = &self.font_regular;
        let mut underline = false;
        let mut crossed_out = false;

        for modifier in rat_cell.modifier.iter() {
            match modifier {
                style::Modifier::BOLD => match &self.font_bold {
                    None => {}
                    Some(font) => font_to_use = font,
                },
                style::Modifier::ITALIC => match &self.font_italic {
                    None => {}
                    Some(font) => font_to_use = font,
                },
                style::Modifier::DIM => {
                    (rat_fg, rat_bg) = (dim_rgb(rat_fg), dim_rgb(rat_bg));
                }
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

        for y in 0..char_height {
            let y_pos = begin_y + y;
            let mut x_pos = begin_x;
            for _ in 0..char_width {
                rgb_pixmap.put_pixel(x_pos, y_pos, rat_bg);
                x_pos += 1;
            }
        }

        let text_symbol = rat_cell.symbol();
        if !text_symbol.is_empty() {
            draw_ttf_symbol(
                font_to_use,
                self.font_size,
                text_symbol,
                begin_x,
                begin_y,
                char_width,
                char_height,
                rat_fg,
                rgb_pixmap,
            );
        }

        if underline {
            draw_horizontal_line(rgb_pixmap, begin_x, begin_y + char_height.saturating_sub(1), char_width, rat_fg);
        }
        if crossed_out {
            draw_horizontal_line(rgb_pixmap, begin_x, begin_y + char_height / 2, char_width, rat_fg);
        }
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
        let (char_width, char_height) = cell_dimensions(&font_regular, font_size);
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
        let (char_width, char_height) = cell_dimensions(&self.raster_backend.font_regular, font_size);

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

fn cell_dimensions(font: &rusttype::Font<'static>, font_size: u32) -> (usize, usize) {
    let scale = rusttype::Scale::uniform(font_size as f32);
    let char_width = font
        .glyph('M')
        .scaled(scale)
        .h_metrics()
        .advance_width
        .ceil()
        .max(1.0) as usize;
    (char_width, font_size.max(1) as usize)
}

fn draw_ttf_symbol(
    font: &rusttype::Font<'static>,
    font_size: u32,
    symbol: &str,
    begin_x: usize,
    begin_y: usize,
    cell_width: usize,
    cell_height: usize,
    fg_color: [u8; 3],
    rgb_pixmap: &mut RgbPixmap,
) {
    let scale = rusttype::Scale::uniform(font_size as f32);
    let v_metrics = font.v_metrics(scale);
    let offset = rusttype::point(begin_x as f32, begin_y as f32 + v_metrics.ascent);
    let is_terminal_art = symbol.chars().all(is_terminal_art_glyph);
    let use_solid_edges = symbol.chars().all(is_block_element_glyph);
    let cell_left = begin_x as i32;
    let cell_top = begin_y as i32;
    let cell_right = cell_left + cell_width as i32;
    let cell_bottom = cell_top + cell_height as i32;

    for glyph in font.layout(symbol, scale, offset) {
        let Some(bb) = glyph.pixel_bounding_box() else {
            continue;
        };

        // Preserve horizontal overhang so adjacent box-drawing cells can join without seams,
        // but keep glyphs inside their own text row to avoid cross-row corruption.
        glyph.draw(|off_x, off_y, coverage| {
            let x = bb.min.x + off_x as i32;
            let y = bb.min.y + off_y as i32;
            if x < 0 || y < 0 {
                return;
            }
            if y < cell_top || y >= cell_bottom {
                return;
            }
            if !is_terminal_art && (x < cell_left || x >= cell_right) {
                return;
            }

            let x = x as usize;
            let y = y as usize;
            if x >= rgb_pixmap.width() || y >= rgb_pixmap.height() {
                return;
            }

            let alpha = if use_solid_edges {
                u8::from(coverage > 0.0) * 255
            } else {
                (coverage * 255.0).round() as u8
            };
            if alpha == 0 {
                return;
            }

            let bg_color = rgb_pixmap.get_pixel(x, y);
            rgb_pixmap.put_pixel(x, y, blend_rgb(fg_color, bg_color, alpha));
        });
    }
}

fn draw_horizontal_line(
    rgb_pixmap: &mut RgbPixmap,
    begin_x: usize,
    y: usize,
    width: usize,
    color: [u8; 3],
) {
    for x in begin_x..begin_x + width {
        rgb_pixmap.put_pixel(x, y, color);
    }
}

fn blend_rgb(fg: [u8; 3], bg: [u8; 3], alpha: u8) -> [u8; 3] {
    if alpha == 255 {
        return fg;
    }

    let alpha = alpha as u16;
    let inv_alpha = 255 - alpha;
    [
        ((fg[0] as u16 * alpha + bg[0] as u16 * inv_alpha) / 255) as u8,
        ((fg[1] as u16 * alpha + bg[1] as u16 * inv_alpha) / 255) as u8,
        ((fg[2] as u16 * alpha + bg[2] as u16 * inv_alpha) / 255) as u8,
    ]
}

fn is_terminal_art_glyph(ch: char) -> bool {
    matches!(ch as u32, 0x2500..=0x259F)
}

fn is_block_element_glyph(ch: char) -> bool {
    matches!(ch as u32, 0x2580..=0x259F)
}
