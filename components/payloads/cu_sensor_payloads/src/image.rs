use bincode::{Decode, Encode};
use cu29::prelude::CuBufferHandle;

#[cfg(feature = "image")]
use image::{ImageBuffer, Pixel};

#[derive(Default, Debug, Encode, Decode, Clone, Copy)]
pub struct CuImageBufferFormat {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub pixel_format: [u8; 4],
}

#[derive(Debug, Default, Clone, Decode, Encode)]
pub struct CuImage {
    pub seq: u64,
    pub format: CuImageBufferFormat,
    pub buffer_handle: CuBufferHandle,
}

impl CuImage {
    pub fn new(format: CuImageBufferFormat, buffer_handle: CuBufferHandle) -> Self {
        CuImage {
            seq: 0,
            format,
            buffer_handle,
        }
    }

    pub fn as_slice(&self) -> &[u8] {
        self.buffer_handle.as_slice()
    }
}

impl CuImage {
    /// Builds an ImageBuffer from the image crate backed by the CuImage's pixel data.
    #[cfg(feature = "image")]
    pub fn as_image_buffer<P: Pixel>(&self) -> ImageBuffer<P, &[P::Subpixel]> {
        let width = self.format.width;
        let height = self.format.height;
        let data = self.buffer_handle.as_slice();

        assert_eq!(
            width, self.format.stride,
            "STRIDE must equal WIDTH for ImageBuffer compatibility."
        );

        let raw_pixels: &[P::Subpixel] =
            unsafe { core::slice::from_raw_parts(data.as_ptr() as *const P::Subpixel, data.len()) };

        ImageBuffer::from_raw(width, height, raw_pixels)
            .expect("Failed to create ImageBuffer with CuImage's pixel data.")
    }
}
