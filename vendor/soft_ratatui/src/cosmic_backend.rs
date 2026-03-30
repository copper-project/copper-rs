//! Cosmic Text rasterization backend for [`SoftBackend`].

use crate::SoftBackend;
use crate::colors::*;
use crate::pixmap::RgbPixmap;
use crate::soft_backend::RasterBackend;
use crate::soft_backend::{BlinkConfig, CursorConfig};
use cosmic_text::fontdb::Database;
use cosmic_text::{
    Attrs, AttrsList, CacheKeyFlags, Family, LineEnding, Metrics, Shaping, Weight, Wrap,
};
use ratatui_core::backend::Backend;
use ratatui_core::buffer::{Buffer, Cell};
use ratatui_core::layout::Rect;
use ratatui_core::style::Modifier;

use cosmic_text::{Buffer as CosmicBuffer, FontSystem, SwashCache};
use rustc_hash::FxHashSet;

/// Raster backend built on `cosmic-text`.
///
/// This backend supports more advanced shaping than the bitmap-oriented
/// backends, but its anti-aliased output may be less desirable for
/// terminal-style rendering.
pub struct CosmicText {
    font_system: FontSystem,

    cosmic_buffer: CosmicBuffer,

    swash_cache: SwashCache,
}

fn add_strikeout(text: &String) -> String {
    let strike = '\u{0336}';
    text.chars().flat_map(|c| [c, strike]).collect()
}

fn add_underline(text: &String) -> String {
    let strike = '\u{0332}';
    text.chars().flat_map(|c| [c, strike]).collect()
}
impl RasterBackend for CosmicText {
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
        let mut rat_fg = rat_cell.fg;
        let rat_bg = rat_cell.bg;
        if rat_cell.modifier.contains(Modifier::HIDDEN) {
            rat_fg = rat_bg;
        }

        let (mut fg_color, mut bg_color) = if rat_cell.modifier.contains(Modifier::REVERSED) {
            (rat_to_rgb(&rat_bg, false), rat_to_rgb(&rat_fg, true))
        } else {
            (rat_to_rgb(&rat_fg, true), rat_to_rgb(&rat_bg, false))
        };

        if rat_cell.modifier.contains(Modifier::DIM) {
            (fg_color, bg_color) = (dim_rgb(fg_color), dim_rgb(bg_color));
        };

        let begin_x = xik as usize * char_width;
        let begin_y = yik as usize * char_height;

        for y in 0..char_height {
            let y_pos = begin_y + y;
            let mut x_pos = begin_x;
            for _ in 0..char_width {
                rgb_pixmap.put_pixel(x_pos, y_pos, bg_color);
                x_pos += 1;
            }
        }

        let mut text_symbol: String = rat_cell.symbol().to_string();

        if rat_cell.modifier.contains(Modifier::CROSSED_OUT) {
            text_symbol = add_strikeout(&text_symbol);
        }
        if rat_cell.modifier.contains(Modifier::UNDERLINED) {
            text_symbol = add_underline(&text_symbol);
        }

        if rat_cell.modifier.contains(Modifier::SLOW_BLINK) {
            always_redraw_list.insert((xik, yik));
            if blinking_slow {
                fg_color = bg_color.clone();
            }
        }
        if rat_cell.modifier.contains(Modifier::RAPID_BLINK) {
            always_redraw_list.insert((xik, yik));
            if blinking_fast {
                fg_color = bg_color.clone();
            }
        }

        let mut attrs = Attrs::new().family(Family::Monospace);
        // attrs = attrs.cache_key_flags(CacheKeyFlags::DISABLE_HINTING);
        if rat_cell.modifier.contains(Modifier::BOLD) {
            attrs = attrs.weight(Weight::BOLD);
        }
        if rat_cell.modifier.contains(Modifier::ITALIC) {
            attrs = attrs.cache_key_flags(CacheKeyFlags::FAKE_ITALIC);
        }
        let mets = self.cosmic_buffer.metrics().font_size;
        let line = self.cosmic_buffer.lines.get_mut(0).unwrap();
        line.set_text(&text_symbol, LineEnding::None, AttrsList::new(&attrs));

        line.layout(&mut self.font_system, mets, None, Wrap::None, None, 0);

        for run in self.cosmic_buffer.layout_runs() {
            for glyph in run.glyphs.iter() {
                let physical_glyph = glyph.physical((0., 0.), 1.0);

                //TODO : Handle Content::Color (emojis?)

                if let Some(image) = self
                    .swash_cache
                    .get_image(&mut self.font_system, physical_glyph.cache_key)
                {
                    let x = image.placement.left;

                    let y = -image.placement.top;
                    let mut i = 0;

                    for off_y in 0..image.placement.height {
                        for off_x in 0..image.placement.width {
                            let real_x = physical_glyph.x + x + off_x as i32;

                            let real_y = run.line_y as i32 + physical_glyph.y + y + off_y as i32;

                            if real_x >= 0 && real_y >= 0 {
                                let get_x = begin_x + real_x as usize;
                                let get_y = begin_y + real_y as usize;

                                //This is used to set the alpha tolerance of block characters to zero
                                let put_color = match override_char(&text_symbol) {
                                    (true, val) => {
                                        if image.data[i] > val {
                                            [fg_color[0], fg_color[1], fg_color[2]]
                                        } else {
                                            [bg_color[0], bg_color[1], bg_color[2]]
                                        }
                                    }
                                    (false, _) => blend_rgba(
                                        [fg_color[0], fg_color[1], fg_color[2], image.data[i]],
                                        [bg_color[0], bg_color[1], bg_color[2], 255],
                                    ),
                                };

                                rgb_pixmap.put_pixel(get_x, get_y, put_color);
                            }

                            i += 1;
                        }
                    }
                }
            }
        }
    }
}
impl SoftBackend<CosmicText> {
    /// Sets a new font size for the terminal image.
    /// This will recreate the pixmap and do a full redraw. Do not run every frame.
    pub fn set_font_size(&mut self, font_size: i32) {
        let metrics = Metrics::new(font_size as f32, font_size as f32);
        self.raster_backend
            .cosmic_buffer
            .set_metrics(&mut self.raster_backend.font_system, metrics);
        let mut buffer = CosmicBuffer::new(&mut self.raster_backend.font_system, metrics);
        let mut buffer = buffer.borrow_with(&mut self.raster_backend.font_system);
        //"█\n█",
        buffer.set_text(
            "█\n█",
            &Attrs::new().family(Family::Monospace),
            Shaping::Advanced,
        );
        buffer.shape_until_scroll(true);
        let boop = buffer.layout_runs().next().unwrap();
        let physical_glyph = boop.glyphs.iter().next().unwrap().physical((0., 0.), 1.0);

        let wa = self
            .raster_backend
            .swash_cache
            .get_image(
                &mut self.raster_backend.font_system,
                physical_glyph.cache_key,
            )
            .clone()
            .unwrap()
            .placement;

        let char_width = wa.width as usize;
        let char_height = wa.height as usize;
        self.raster_backend.cosmic_buffer.set_size(
            &mut self.raster_backend.font_system,
            Some(char_width as f32),
            Some(char_height as f32),
        );
        self.char_width = char_width;
        self.char_height = char_height;
        self.rgb_pixmap = RgbPixmap::new(
            self.char_width * self.buffer.area.width as usize,
            self.char_height * self.buffer.area.height as usize,
        );

        self.redraw();
    }

    /// Creates a new Software Backend with the given font data.
    ///
    /// (new-with-font width height font-size font-data) -> SoftBackend
    ///
    /// * width      : usize - Width of the terminal in cells
    /// * height     : usize - Height of the terminal in cells
    /// * font-size  : u32   - Font size in pixels
    /// * font-data  : &[u8] - Byte slice of the font (e.g., included with `include_bytes!`)
    ///
    /// # Examples
    /// ```rust
    /// use soft_ratatui::{CosmicText, SoftBackend};
    ///
    /// static FONT_DATA: &[u8] =
    ///     include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/iosevka.ttf"));
    /// let backend = SoftBackend::<CosmicText>::new(20, 20, 16, FONT_DATA);
    /// let _ = backend;
    /// ```

    pub fn new(width: u16, height: u16, font_size: i32, font_data: &[u8]) -> Self {
        let mut swash_cache = SwashCache::new();

        let mut db = Database::new();
        // "assets/iosevka.ttf"
        db.load_font_data(font_data.to_vec());
        //  db.set_monospace_family("Fira Mono");

        let mut font_system = FontSystem::new_with_locale_and_db("English".to_string(), db);
        let metrics = Metrics::new(font_size as f32, font_size as f32);

        let mut buffer = CosmicBuffer::new(&mut font_system, metrics);
        let mut buffer = buffer.borrow_with(&mut font_system);
        buffer.set_text(
            "██\n██",
            &Attrs::new().family(Family::Monospace),
            Shaping::Advanced,
        );
        //     buffer.shape_until_cursor(Cursor::new(2, 0), true);
        buffer.shape_until_scroll(true);

        let runczik = buffer.layout_runs().next().unwrap();

        let physical_glyph = runczik
            .glyphs
            .iter()
            .next()
            .unwrap()
            .physical((0., 0.), 1.0);

        let wa = swash_cache
            .get_image(&mut font_system, physical_glyph.cache_key)
            .clone()
            .unwrap()
            .placement;

        let mut cosmic_buffer = CosmicBuffer::new(&mut font_system, metrics);

        let char_width = wa.width as usize;

        let char_height = wa.height as usize; //- line_offset.ceil() as usize;
        cosmic_buffer.set_size(
            &mut font_system,
            Some(char_width as f32),
            Some(char_height as f32),
        );

        let rgb_pixmap = RgbPixmap::new(char_width * width as usize, char_height * height as usize);

        let mut return_struct = Self {
            buffer: Buffer::empty(Rect::new(0, 0, width, height)),
            cursor: false,
            cursor_pos: (0, 0),
            cursor_config: CursorConfig::default(),

            raster_backend: CosmicText {
                font_system,
                cosmic_buffer,
                swash_cache,
            },
            rgb_pixmap,

            char_width,
            char_height,

            frame_count: 0,
            blink_config: BlinkConfig::default(),
            always_redraw_list: FxHashSet::default(),
            rendered_cursor: None,
        };
        _ = return_struct.clear();
        return_struct
    }
}

fn override_char(c: &str) -> (bool, u8) {
    let code = c.chars().next().unwrap() as u32;
    //block shades
    if (0x2591..=0x2593).contains(&code) {
        (false, 0)
    }
    //block drawing
    else if (0x2580..=0x259F).contains(&code) {
        (true, 0)
    } else {
        (false, 127)
    }
}
