use crate::pixmap::RgbPixmap;
use ratatui_core::buffer::{Buffer, Cell};
use rustc_hash::FxHashSet;

use core::convert::Infallible;

use crate::colors::*;

use ratatui_core::backend::{Backend, ClearType, WindowSize};

use ratatui_core::layout::{Position, Rect, Size};

/// How the cursor is rendered on the pixmap.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CursorStyle {
    /// Invert the colors of the current character cell.
    Inverse,
    /// Draw a one-pixel underline at the bottom of the current character cell.
    Underline,
    /// Draw a one-pixel outline around the current character cell.
    Outline,
    /// Draw corner markers inspired by Japanese IME cursors.
    Japanese,
}

/// Cursor appearance and blink behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CursorConfig {
    /// Visual style for the cursor overlay.
    pub style: CursorStyle,
    /// Whether the cursor should hide using the slow blink cadence.
    pub blink: bool,
    /// RGB color for non-inverse cursor styles.
    pub color: [u8; 3],
}

impl Default for CursorConfig {
    fn default() -> Self {
        Self {
            style: CursorStyle::Inverse,
            blink: true,
            color: [255, 255, 255],
        }
    }
}

/// Timing parameters for a single blink pattern.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlinkTiming {
    /// How many times per second the element toggles.
    pub blinks_per_sec: u16,
    /// Percentage of each cycle spent hidden.
    pub duty_percent: u16,
    hidden: bool,
}

impl BlinkTiming {
    /// Returns `true` when the blink target should currently be hidden.
    pub fn is_hidden(&self) -> bool {
        self.hidden
    }

    fn update(&mut self, frame_count: u16, fps: u16) {
        if self.blinks_per_sec == 0 || fps == 0 {
            self.hidden = false;
            return;
        }

        let cycle_len = fps / self.blinks_per_sec;
        if cycle_len == 0 {
            self.hidden = false;
            return;
        }

        let pos = frame_count % cycle_len;
        let hidden_frames = ((self.duty_percent * cycle_len + 50) / 100).max(1);
        self.hidden = pos >= cycle_len.saturating_sub(hidden_frames);
    }
}

/// Blink configuration for text modifiers and cursor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlinkConfig {
    /// Display refresh rate used to convert frames to time.
    pub fps: u16,
    /// Timing for slow blink text and the cursor.
    pub slow: BlinkTiming,
    /// Timing for rapid blink text.
    pub fast: BlinkTiming,
    prev_state: (bool, bool),
}

impl BlinkConfig {
    /// Advances the blink state and returns `true` when visibility changed.
    pub fn tick(&mut self, frame_count: u16) -> bool {
        self.slow.update(frame_count, self.fps);
        self.fast.update(frame_count, self.fps);

        let state = (self.slow.hidden, self.fast.hidden);
        let toggled = state != self.prev_state;
        self.prev_state = state;
        toggled
    }
}

impl Default for BlinkConfig {
    fn default() -> Self {
        Self {
            fps: 60,
            slow: BlinkTiming {
                blinks_per_sec: 1,
                duty_percent: 15,
                hidden: false,
            },
            fast: BlinkTiming {
                blinks_per_sec: 3,
                duty_percent: 50,
                hidden: false,
            },
            prev_state: (false, false),
        }
    }
}

/// A software-rendering [`Backend`] for Ratatui.
///
/// `SoftBackend` rasterizes terminal cells into an internal [`RgbPixmap`], which
/// can then be consumed by GUI toolkits, game engines, or other non-terminal
/// renderers.
pub struct SoftBackend<R: RasterBackend> {
    pub buffer: Buffer,
    pub cursor: bool,
    pub cursor_pos: (u16, u16),
    pub cursor_config: CursorConfig,
    pub char_width: usize,
    pub char_height: usize,
    pub frame_count: u16,
    pub blink_config: BlinkConfig,
    pub rgb_pixmap: RgbPixmap,
    pub always_redraw_list: FxHashSet<(u16, u16)>,
    pub raster_backend: R,
    #[doc(hidden)]
    pub rendered_cursor: Option<(u16, u16)>,
}
/// Trait implemented by font rasterizers used by [`SoftBackend`].
pub trait RasterBackend {
    fn draw_cell(
        &mut self,
        x: u16,
        y: u16,
        rat_cell: &Cell,
        always_redraw_list: &mut FxHashSet<(u16, u16)>,
        blinking_fast: bool,
        blinking_slow: bool,
        char_width: usize,
        char_height: usize,
        rgb_pixmap: &mut RgbPixmap,
    );
}

impl<R: RasterBackend> Backend for SoftBackend<R> {
    type Error = Infallible;

    fn draw<'a, I>(&mut self, content: I) -> Result<(), Self::Error>
    where
        I: Iterator<Item = (u16, u16, &'a Cell)>,
    {
        self.update_blinking();
        let blinking_fast = self.fast_blink_hidden();
        let blinking_slow = self.slow_blink_hidden();
        for (x, y, c) in content {
            self.buffer[(x, y)] = c.clone();
            self.raster_backend.draw_cell(
                x,
                y,
                c,
                &mut self.always_redraw_list,
                blinking_fast,
                blinking_slow,
                self.char_width,
                self.char_height,
                &mut self.rgb_pixmap,
            );
        }
        for (x, y) in self.always_redraw_list.clone().iter() {
            let c = &self.buffer[(*x, *y)];
            self.raster_backend.draw_cell(
                *x,
                *y,
                c,
                &mut self.always_redraw_list,
                blinking_fast,
                blinking_slow,
                self.char_width,
                self.char_height,
                &mut self.rgb_pixmap,
            );
        }

        Ok(())
    }

    fn hide_cursor(&mut self) -> Result<(), Self::Error> {
        self.cursor = false;
        Ok(())
    }

    fn show_cursor(&mut self) -> Result<(), Self::Error> {
        self.cursor = true;
        Ok(())
    }

    fn get_cursor_position(&mut self) -> Result<Position, Self::Error> {
        Ok(self.cursor_pos.into())
    }

    fn set_cursor_position<P: Into<Position>>(&mut self, position: P) -> Result<(), Self::Error> {
        self.cursor_pos = position.into().into();
        Ok(())
    }

    fn clear(&mut self) -> Result<(), Self::Error> {
        self.buffer.reset();
        let clear_cell = Cell::EMPTY;
        let colorik = rat_to_rgb(&clear_cell.bg, false);
        self.rgb_pixmap.fill([colorik[0], colorik[1], colorik[2]]);
        self.rendered_cursor = None;
        self.sync_cursor_overlay();
        Ok(())
    }

    fn size(&self) -> Result<Size, Self::Error> {
        Ok(self.buffer.area.as_size())
    }

    fn window_size(&mut self) -> Result<WindowSize, Self::Error> {
        let window_pixels = Size {
            width: self.get_pixmap_width() as u16,
            height: self.get_pixmap_height() as u16,
        };
        Ok(WindowSize {
            columns_rows: self.buffer.area.as_size(),
            pixels: window_pixels,
        })
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.sync_cursor_overlay();
        Ok(())
    }

    fn clear_region(&mut self, clear_type: ClearType) -> Result<(), Self::Error> {
        let region = match clear_type {
            ClearType::All => return self.clear(),
            ClearType::AfterCursor => {
                let index = self.buffer.index_of(self.cursor_pos.0, self.cursor_pos.1) + 1;
                &mut self.buffer.content[index..]
            }
            ClearType::BeforeCursor => {
                let index = self.buffer.index_of(self.cursor_pos.0, self.cursor_pos.1);
                &mut self.buffer.content[..index]
            }
            ClearType::CurrentLine => {
                let line_start_index = self.buffer.index_of(0, self.cursor_pos.1);
                let line_end_index = self
                    .buffer
                    .index_of(self.buffer.area.width - 1, self.cursor_pos.1);
                &mut self.buffer.content[line_start_index..=line_end_index]
            }
            ClearType::UntilNewLine => {
                let index = self.buffer.index_of(self.cursor_pos.0, self.cursor_pos.1);
                let line_end_index = self
                    .buffer
                    .index_of(self.buffer.area.width - 1, self.cursor_pos.1);
                &mut self.buffer.content[index..=line_end_index]
            }
        };
        for cell in region {
            cell.reset();
        }
        Ok(())
    }
}

impl<R: RasterBackend> SoftBackend<R> {
    /// Returns the raw RGB data of the pixmap as a flat array.
    pub fn get_pixmap_data(&self) -> &[u8] {
        self.rgb_pixmap.data()
    }

    /// Returns the pixmap in RGBA format as a flat vector.
    pub fn get_pixmap_data_as_rgba(&self) -> Vec<u8> {
        self.rgb_pixmap.to_rgba()
    }

    /// Returns the width of the pixmap in pixels.
    pub fn get_pixmap_width(&self) -> usize {
        self.rgb_pixmap.width()
    }

    /// Returns the height of the pixmap in pixels.
    pub fn get_pixmap_height(&self) -> usize {
        self.rgb_pixmap.height()
    }

    /// Returns a reference to the current Ratatui cell buffer.
    pub const fn buffer(&self) -> &Buffer {
        &self.buffer
    }

    /// Resizes the terminal in character cells and reallocates the backing pixmap.
    pub fn resize(&mut self, width: u16, height: u16) {
        self.buffer.resize(Rect::new(0, 0, width, height));
        let rgb_pixmap = RgbPixmap::new(
            self.char_width as usize * width as usize,
            self.char_height as usize * height as usize,
        );
        self.rgb_pixmap = rgb_pixmap;
        self.rendered_cursor = None;
        self.redraw();
    }

    /// Redraws the entire pixmap from the current cell buffer.
    pub fn redraw(&mut self) {
        self.always_redraw_list = FxHashSet::default();
        let blinking_fast = self.fast_blink_hidden();
        let blinking_slow = self.slow_blink_hidden();
        for x in 0..self.buffer.area.width {
            for y in 0..self.buffer.area.height {
                let c = &self.buffer[(x, y)];
                self.raster_backend.draw_cell(
                    x,
                    y,
                    c,
                    &mut self.always_redraw_list,
                    blinking_fast,
                    blinking_slow,
                    self.char_width,
                    self.char_height,
                    &mut self.rgb_pixmap,
                );
            }
        }
        self.rendered_cursor = None;
        self.sync_cursor_overlay();
    }

    fn update_blinking(&mut self) {
        self.frame_count = self.frame_count.wrapping_add(1);
        let _ = self.blink_config.tick(self.frame_count);
    }

    fn sync_cursor_overlay(&mut self) {
        self.restore_cursor_cell();

        if let Some((x, y)) = self.cursor_to_render() {
            self.draw_cursor_overlay(x, y);
            self.rendered_cursor = Some((x, y));
        }
    }

    fn restore_cursor_cell(&mut self) {
        if let Some((x, y)) = self.rendered_cursor.take()
            && self.position_in_bounds((x, y))
        {
            self.redraw_cell(x, y);
        }
    }

    fn cursor_to_render(&self) -> Option<(u16, u16)> {
        if !self.cursor || !self.position_in_bounds(self.cursor_pos) {
            return None;
        }

        if self.cursor_config.blink && self.slow_blink_hidden() {
            return None;
        }

        Some(self.cursor_pos)
    }

    fn position_in_bounds(&self, (x, y): (u16, u16)) -> bool {
        x < self.buffer.area.width && y < self.buffer.area.height
    }

    fn redraw_cell(&mut self, x: u16, y: u16) {
        let c = &self.buffer[(x, y)];
        let blinking_fast = self.fast_blink_hidden();
        let blinking_slow = self.slow_blink_hidden();
        self.raster_backend.draw_cell(
            x,
            y,
            c,
            &mut self.always_redraw_list,
            blinking_fast,
            blinking_slow,
            self.char_width,
            self.char_height,
            &mut self.rgb_pixmap,
        );
    }

    fn fast_blink_hidden(&self) -> bool {
        self.blink_config.fast.is_hidden()
    }

    fn slow_blink_hidden(&self) -> bool {
        self.blink_config.slow.is_hidden()
    }

    fn draw_cursor_overlay(&mut self, x: u16, y: u16) {
        let base_x = x as usize * self.char_width;
        let base_y = y as usize * self.char_height;

        match self.cursor_config.style {
            CursorStyle::Inverse => {
                self.rgb_pixmap
                    .invert_rect(base_x, base_y, self.char_width, self.char_height);
            }
            CursorStyle::Underline => {
                if self.char_height > 0 {
                    self.rgb_pixmap.fill_rect(
                        base_x,
                        base_y + self.char_height - 1,
                        self.char_width,
                        1,
                        self.cursor_config.color,
                    );
                }
            }
            CursorStyle::Outline => {
                if self.char_width == 0 || self.char_height == 0 {
                    return;
                }

                self.rgb_pixmap.fill_rect(
                    base_x,
                    base_y,
                    self.char_width,
                    1,
                    self.cursor_config.color,
                );
                self.rgb_pixmap.fill_rect(
                    base_x,
                    base_y + self.char_height - 1,
                    self.char_width,
                    1,
                    self.cursor_config.color,
                );
                self.rgb_pixmap.fill_rect(
                    base_x,
                    base_y,
                    1,
                    self.char_height,
                    self.cursor_config.color,
                );
                self.rgb_pixmap.fill_rect(
                    base_x + self.char_width - 1,
                    base_y,
                    1,
                    self.char_height,
                    self.cursor_config.color,
                );
            }
            CursorStyle::Japanese => {
                if self.char_width == 0 || self.char_height == 0 {
                    return;
                }

                let corner_width = (self.char_width / 2).max(1);
                let corner_height = (self.char_height / 2).max(1);
                self.rgb_pixmap.fill_rect(
                    base_x,
                    base_y,
                    corner_width,
                    1,
                    self.cursor_config.color,
                );
                self.rgb_pixmap.fill_rect(
                    base_x,
                    base_y,
                    1,
                    corner_height,
                    self.cursor_config.color,
                );
                self.rgb_pixmap.fill_rect(
                    base_x + self.char_width - 1,
                    base_y + self.char_height.saturating_sub(corner_height),
                    1,
                    corner_height,
                    self.cursor_config.color,
                );
                self.rgb_pixmap.fill_rect(
                    base_x + self.char_width.saturating_sub(corner_width),
                    base_y + self.char_height - 1,
                    corner_width,
                    1,
                    self.cursor_config.color,
                );
            }
        }
    }
}
