use bincode::{Decode, Encode};
use cu29::prelude::CuBufferHandle;

#[allow(unused_imports)]
use cu29::{CuError, CuResult};

#[cfg(feature = "image")]
use image::{ImageBuffer, Pixel};
#[cfg(feature = "kornia")]
use kornia::image::Image;
#[cfg(feature = "kornia")]
use kornia::image::ImageSize;

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
    pub fn as_image_buffer<P: Pixel>(&self) -> CuResult<ImageBuffer<P, &[P::Subpixel]>> {
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
            .ok_or("Could not create the image:: buffer".into())
    }

    #[cfg(feature = "kornia")]
    pub fn as_kornia_image<T: Clone, const C: usize>(&self) -> CuResult<Image<T, C>> {
        let width = self.format.width as usize;
        let height = self.format.height as usize;
        let data = self.buffer_handle.as_slice();

        assert_eq!(
            width, self.format.stride as usize,
            "STRIDE must equal WIDTH for Kornia compatibility."
        );

        let size = width * height * C;

        let raw_pixels: &[T] = unsafe {
            core::slice::from_raw_parts(data.as_ptr() as *const T, data.len() / size_of::<T>())
        };
        let img_size: ImageSize = ImageSize { height, width };

        unsafe { Image::from_raw_parts(img_size, raw_pixels.as_ptr(), size) }
            .map_err(|e| CuError::new_with_cause("Could not create a Kornia Image", e))
    }
}
