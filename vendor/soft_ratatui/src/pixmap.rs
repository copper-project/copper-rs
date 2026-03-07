use core::convert::Infallible;
#[cfg(any(feature = "embedded-graphics", feature = "embedded-ttf"))]
use embedded_graphics::{
    Pixel,
    draw_target::DrawTarget,
    geometry::{OriginDimensions, Size},
    pixelcolor::Rgb888,
    prelude::*,
};

/// A pixmap with RGB pixels stored in a flat vector.
#[derive(Debug, Clone)]
pub struct RgbPixmap {
    pub width: usize,
    pub height: usize,
    pub data: Vec<u8>,
}

impl RgbPixmap {
    /// Creates a new pixmap.
    pub fn new(width: usize, height: usize) -> Self {
        let data = [0, 0, 0].repeat(width * height);
        Self {
            width,
            height,
            data,
        }
    }
    /// Outputs the RGBpixmap as a RGBA flat vector, useful when target renderer does not take pure RGB data
    pub fn to_rgba(&self) -> Vec<u8> {
        let mut rgba_data = Vec::with_capacity(self.width * self.height * 4);
        for chunk in self.data.chunks_exact(3) {
            let r = chunk[0];
            let g = chunk[1];
            let b = chunk[2];
            rgba_data.extend_from_slice(&[r, g, b, 255]); // Alpha = 255 for no transparency
        }
        rgba_data
    }
    /// Outputs the RGBpixmap as a RGBA flat vector, with a color set to transparent, useful for stuff
    pub fn to_rgba_with_color_as_transparent(&self, color: &(u8, u8, u8)) -> Vec<u8> {
        let mut rgba_data = Vec::with_capacity(self.width * self.height * 4);
        for chunk in self.data.chunks_exact(3) {
            let r = chunk[0];
            let g = chunk[1];
            let b = chunk[2];
            if &(r, g, b) == color {
                rgba_data.extend_from_slice(&[r, g, b, 0]);
            } else {
                rgba_data.extend_from_slice(&[r, g, b, 255]);
            }
        }
        rgba_data
    }

    /// Sets the RGB value of a pixel at (x, y).
    pub fn put_pixel(&mut self, x: usize, y: usize, color: [u8; 3]) {
        if x < self.width && y < self.height {
            let index = 3 * (y * self.width + x);
            self.data[index..index + 3].copy_from_slice(&color);
        }
    }

    /// Returns the RGB value of a pixel at (x, y).
    pub fn get_pixel(&self, x: usize, y: usize) -> [u8; 3] {
        debug_assert!(
            x < self.width && y < self.height,
            "Pixel coordinates out of bounds"
        );
        let index = 3 * (y * self.width + x);
        self.data[index..index + 3]
            .try_into()
            .expect("ERROR RETRIEVING VALUE FROM X Y COORDINATES")
    }

    /// Fills the entire pixmap with the specified RGB color.
    pub fn fill(&mut self, color: [u8; 3]) {
        for chunk in self.data.chunks_mut(3) {
            chunk.copy_from_slice(&color);
        }
    }

    /// Returns the width of the pixmap in pixels
    pub fn width(&self) -> usize {
        self.width
    }
    /// Returns the height of the pixmap in pixels
    pub fn height(&self) -> usize {
        self.height
    }

    /// Retuns the raw rgb data of the pixmap as a flat array
    pub fn data(&self) -> &[u8] {
        &self.data
    }
}
#[cfg(any(feature = "embedded-graphics", feature = "embedded-ttf"))]
impl DrawTarget for RgbPixmap {
    type Color = Rgb888;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels {
            //   if coord.x >= 0&& coord.y >=0 {}
            self.put_pixel(
                coord.x as usize,
                coord.y as usize,
                [color.r(), color.g(), color.b()],
            );
        }
        Ok(())
    }
}
#[cfg(any(feature = "embedded-graphics", feature = "embedded-ttf"))]
impl OriginDimensions for RgbPixmap {
    fn size(&self) -> Size {
        Size::new(self.width as u32, self.height as u32)
    }
}
