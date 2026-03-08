use crate::pixmap::RgbPixmap;
use ratatui_core::buffer::{Buffer, Cell};
use rustc_hash::FxHashSet;

use core::convert::Infallible;

use crate::colors::*;

use ratatui_core::backend::{Backend, ClearType, WindowSize};

use ratatui_core::layout::{Position, Rect, Size};

/// SoftBackend is a Software rendering backend for Ratatui. It stores the generated image internally as rgb_pixmap.
pub struct SoftBackend<R: RasterBackend> {
    pub buffer: Buffer,
    pub cursor: bool,
    pub cursor_pos: (u16, u16),
    pub char_width: usize,
    pub char_height: usize,
    pub blink_counter: u16,
    pub blinking_fast: bool,
    pub blinking_slow: bool,
    pub rgb_pixmap: RgbPixmap,
    pub always_redraw_list: FxHashSet<(u16, u16)>,
    pub raster_backend: R,
}
/// Trait for raster backends (TTF, embedded-graphics, etc.)
pub trait RasterBackend {
    #[allow(clippy::too_many_arguments)]
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
        for (x, y, c) in content {
            self.buffer[(x, y)] = c.clone();
            self.raster_backend.draw_cell(
                x,
                y,
                c,
                &mut self.always_redraw_list,
                self.blinking_fast,
                self.blinking_slow,
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
                self.blinking_fast,
                self.blinking_slow,
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

    /// Returns a reference to the internal buffer of the `SoftBackend`.
    pub const fn buffer(&self) -> &Buffer {
        &self.buffer
    }

    /// Resizes the `SoftBackend` to the specified width and height.
    pub fn resize(&mut self, width: u16, height: u16) {
        self.buffer.resize(Rect::new(0, 0, width, height));
        let rgb_pixmap = RgbPixmap::new(self.char_width * width as usize, self.char_height * height as usize);
        self.rgb_pixmap = rgb_pixmap;
        self.redraw();
    }

    /// Redraws the pixmap
    pub fn redraw(&mut self) {
        self.always_redraw_list = FxHashSet::default();
        for x in 0..self.buffer.area.width {
            for y in 0..self.buffer.area.height {
                let c = &self.buffer[(x, y)];
                self.raster_backend.draw_cell(
                    x,
                    y,
                    c,
                    &mut self.always_redraw_list,
                    self.blinking_fast,
                    self.blinking_slow,
                    self.char_width,
                    self.char_height,
                    &mut self.rgb_pixmap,
                );
            }
        }
    }

    fn update_blinking(&mut self) {
        self.blink_counter = (self.blink_counter + 1) % 200;

        self.blinking_fast = matches!(self.blink_counter % 100, 0..=5);
        self.blinking_slow = matches!(self.blink_counter, 20..=25);
    }
}
