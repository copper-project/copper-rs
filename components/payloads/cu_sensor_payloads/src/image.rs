use bincode::de::Decoder;
use bincode::error::DecodeError;
use bincode::{Decode, Encode};
use cu29::prelude::{ArrayLike, CuHandle};
use std::fmt::Debug;
use std::sync::{Arc, Mutex};

#[allow(unused_imports)]
use cu29::{CuError, CuResult};

#[cfg(feature = "image")]
use image::{ImageBuffer, Pixel};
#[cfg(feature = "kornia")]
use kornia::image::Image;

#[derive(Default, Debug, Encode, Decode, Clone, Copy)]
pub struct CuImageBufferFormat {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub pixel_format: [u8; 4],
}

impl CuImageBufferFormat {
    pub fn byte_size(&self) -> usize {
        self.stride as usize * self.height as usize
    }
}

#[derive(Debug, Default, Clone, Encode)]
pub struct CuImage<'a, A>
where
    A: ArrayLike<Element = u8>,
{
    pub seq: u64,
    pub format: CuImageBufferFormat,
    pub buffer_handle: CuHandle<'a, A>,
}

impl<'a> Decode for CuImage<'a, Vec<u8>> {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let seq = u64::decode(decoder)?;
        let format = CuImageBufferFormat::decode(decoder)?;
        let buffer = Vec::decode(decoder)?;
        let buffer_handle = CuHandle::Detached(Arc::new(Mutex::new(buffer)));

        Ok(Self {
            seq,
            format,
            buffer_handle,
        })
    }
}

impl<'a, A> CuImage<'a, A>
where
    A: ArrayLike<Element = u8>,
{
    pub fn new(format: CuImageBufferFormat, buffer_handle: CuHandle<'a, A>) -> Self {
        // assert!(
        //     format.byte_size() < buffer_handle.lock().unwrap().len(),
        //     "Buffer size must at least match the format."
        // );
        CuImage {
            seq: 0,
            format,
            buffer_handle: buffer_handle,
        }
    }

    fn buffer_handle(&self) -> &CuHandle<'a, A> {
        &self.buffer_handle
    }
}

impl<'a, A> CuImage<'a, A>
where
    A: ArrayLike<Element = u8>,
{
    /// Builds an ImageBuffer from the image crate backed by the CuImage's pixel data.
    #[cfg(feature = "image")]
    pub fn as_image_buffer<P: Pixel>(&self) -> CuResult<ImageBuffer<P, &[P::Subpixel]>> {
        let width = self.format.width;
        let height = self.format.height;
        assert_eq!(
            width, self.format.stride,
            "STRIDE must equal WIDTH for ImageBuffer compatibility."
        );

        let raw_pixels: &[P::Subpixel] = match self.buffer_handle {
            CuHandle::Detached(ref detached) => {
                let handle = detached.lock().unwrap();
                let data = handle.slice();
                unsafe {
                    core::slice::from_raw_parts(data.as_ptr() as *const P::Subpixel, data.len())
                }
            }
            CuHandle::Pooled(ref pooled) => {
                let handle = pooled.lock().unwrap();
                let data = handle.slice();
                unsafe {
                    core::slice::from_raw_parts(data.as_ptr() as *const P::Subpixel, data.len())
                }
            }
        };
        ImageBuffer::from_raw(width, height, raw_pixels)
            .ok_or("Could not create the image:: buffer".into())
    }

    #[cfg(feature = "kornia")]
    pub fn as_kornia_image<T: Clone, const C: usize>(&self) -> CuResult<Image<T, C>> {
        let width = self.format.width as usize;
        let height = self.format.height as usize;

        assert_eq!(
            width, self.format.stride as usize,
            "stride must equal width for Kornia compatibility."
        );

        let size = width * height * C;

        let raw_pixels: &[T] = match self.buffer_handle {
            CuHandle::Detached(ref detached) => {
                let handle = detached.lock().unwrap();
                let data = handle.slice();
                unsafe {
                    core::slice::from_raw_parts(
                        data.as_ptr() as *const T,
                        data.len() / size_of::<T>(),
                    )
                }
            }
            CuHandle::Pooled(ref pooled) => {
                let handle = pooled.lock().unwrap();
                let data = handle.slice();
                unsafe {
                    core::slice::from_raw_parts(
                        data.as_ptr() as *const T,
                        data.len() / size_of::<T>(),
                    )
                }
            }
        };

        unsafe { Image::from_raw_parts([height, width].into(), raw_pixels.as_ptr(), size) }
            .map_err(|e| CuError::new_with_cause("Could not create a Kornia Image", e))
    }
}
