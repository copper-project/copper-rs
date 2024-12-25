use image::{ImageBuffer, Pixel};

#[repr(align(4096))]
/// A simple image struct that can be used to store image data in a contiguous block of memory.
struct CuImage<const WIDTH: usize, const HEIGHT: usize, const STRIDE: usize, P: Pixel + Default> {
    pixels: [[P; STRIDE]; HEIGHT],
}

impl<const WIDTH: usize, const HEIGHT: usize, const STRIDE: usize, P> Default
    for CuImage<WIDTH, HEIGHT, STRIDE, P>
where
    P: Default + Copy,
{
    fn default() -> Self {
        Self {
            pixels: [[P::default(); STRIDE]; HEIGHT],
        }
    }
}

impl<const WIDTH: usize, const HEIGHT: usize, const STRIDE: usize, P>
    CuImage<WIDTH, HEIGHT, STRIDE, P>
where
    P: Pixel + Default + 'static,
    P::Subpixel: Copy,
{
    /// Builds an ImageBuffer from the image crate backed by the CuImage's pixel data.
    pub fn as_image_buffer(&self) -> ImageBuffer<P, &[P::Subpixel]> {
        assert_eq!(
            STRIDE, WIDTH,
            "STRIDE must equal WIDTH for ImageBuffer compatibility."
        );

        let raw_pixels: &[P::Subpixel] = unsafe {
            core::slice::from_raw_parts(
                self.pixels.as_ptr() as *const P::Subpixel,
                WIDTH * HEIGHT * P::CHANNEL_COUNT as usize,
            )
        };

        ImageBuffer::from_raw(WIDTH as u32, HEIGHT as u32, raw_pixels)
            .expect("Failed to create ImageBuffer with CuImage's pixel data.")
    }
}
