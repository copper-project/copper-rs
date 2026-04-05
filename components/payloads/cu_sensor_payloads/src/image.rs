use bincode::de::Decoder;
use bincode::error::DecodeError;
use bincode::{Decode, Encode};
use core::fmt::Debug;
use cu29::prelude::*;

#[cfg(feature = "image")]
use image::{ImageBuffer, Pixel};
#[cfg(feature = "kornia")]
use kornia_image::Image;
#[cfg(feature = "kornia")]
use kornia_image::allocator::ImageAllocator;
use serde::{Deserialize, Serialize, Serializer};

#[derive(Default, Debug, Encode, Decode, Clone, Copy, Serialize, Deserialize, Reflect)]
pub struct CuImageBufferFormat {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub pixel_format: [u8; 4],
}

impl CuImageBufferFormat {
    pub fn byte_size(&self) -> usize {
        let plane_size = self.stride as usize * self.height as usize;
        match &self.pixel_format {
            b"NV12" | b"NV21" | b"I420" | b"YV12" => plane_size + plane_size / 2,
            _ => plane_size,
        }
    }
}

#[derive(Debug, Default, Clone, Encode, Reflect)]
#[reflect(from_reflect = false, no_field_bounds, type_path = false)]
pub struct CuImage<A>
where
    A: ArrayLike<Element = u8> + Send + Sync + 'static,
{
    pub seq: u64,
    pub format: CuImageBufferFormat,
    #[reflect(ignore)]
    pub buffer_handle: CuHandle<A>,
}

impl<A> TypePath for CuImage<A>
where
    A: ArrayLike<Element = u8> + Send + Sync + 'static,
{
    fn type_path() -> &'static str {
        "cu_sensor_payloads::CuImage"
    }

    fn short_type_path() -> &'static str {
        "CuImage"
    }

    fn type_ident() -> Option<&'static str> {
        Some("CuImage")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_sensor_payloads")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_sensor_payloads")
    }
}

impl<A> Decode<()> for CuImage<A>
where
    A: ArrayLike<Element = u8> + Send + Sync + 'static,
    CuHandle<A>: Decode<()>,
{
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let seq: u64 = Decode::decode(decoder)?;
        let format: CuImageBufferFormat = Decode::decode(decoder)?;
        let buffer_handle: CuHandle<A> = Decode::decode(decoder)?;

        Ok(Self {
            seq,
            format,
            buffer_handle,
        })
    }
}

impl<'de, A> Deserialize<'de> for CuImage<A>
where
    A: ArrayLike<Element = u8> + Send + Sync + 'static,
    CuHandle<A>: Deserialize<'de>,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct CuImageWire<H> {
            seq: u64,
            format: CuImageBufferFormat,
            handle: H,
        }

        let wire = CuImageWire::<CuHandle<A>>::deserialize(deserializer)?;
        Ok(Self {
            seq: wire.seq,
            format: wire.format,
            buffer_handle: wire.handle,
        })
    }
}

impl<A> Serialize for CuImage<A>
where
    A: ArrayLike<Element = u8> + Send + Sync + 'static,
    CuHandle<A>: Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut struct_ = serializer.serialize_struct("CuImage", 3)?;
        struct_.serialize_field("seq", &self.seq)?;
        struct_.serialize_field("format", &self.format)?;
        struct_.serialize_field("handle", &self.buffer_handle)?;
        struct_.end()
    }
}

impl<A> CuImage<A>
where
    A: ArrayLike<Element = u8> + Send + Sync + 'static,
{
    pub fn new(format: CuImageBufferFormat, buffer_handle: CuHandle<A>) -> Self {
        assert!(
            format.byte_size() <= buffer_handle.with_inner(|i| i.len()),
            "Buffer size must at least match the format."
        );
        CuImage {
            seq: 0,
            format,
            buffer_handle,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::CuImageBufferFormat;

    #[test]
    fn byte_size_for_packed_formats_is_stride_times_height() {
        let format = CuImageBufferFormat {
            width: 4,
            height: 3,
            stride: 16,
            pixel_format: *b"BGRA",
        };

        assert_eq!(format.byte_size(), 48);
    }

    #[test]
    fn byte_size_for_nv12_includes_uv_plane() {
        let format = CuImageBufferFormat {
            width: 1280,
            height: 720,
            stride: 1280,
            pixel_format: *b"NV12",
        };

        assert_eq!(format.byte_size(), 1_382_400);
    }

    #[test]
    fn byte_size_for_i420_includes_chroma_planes() {
        let format = CuImageBufferFormat {
            width: 640,
            height: 480,
            stride: 640,
            pixel_format: *b"I420",
        };

        assert_eq!(format.byte_size(), 460_800);
    }
}

impl<A> CuImage<A>
where
    A: ArrayLike<Element = u8> + Send + Sync + 'static,
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

        let raw_pixels: &[P::Subpixel] = self.buffer_handle.with_inner(|inner| {
            // SAFETY: The buffer is contiguous, aligned for P::Subpixel (typically u8), and large enough.
            unsafe {
                let data: &[u8] = inner;
                core::slice::from_raw_parts(data.as_ptr() as *const P::Subpixel, data.len())
            }
        });
        ImageBuffer::from_raw(width, height, raw_pixels)
            .ok_or("Could not create the image:: buffer".into())
    }

    #[cfg(feature = "kornia")]
    pub fn as_kornia_image<T: Clone, const C: usize, K: ImageAllocator>(
        &self,
        k: K,
    ) -> CuResult<Image<T, C, K>> {
        let width = self.format.width as usize;
        let height = self.format.height as usize;

        assert_eq!(
            width, self.format.stride as usize,
            "stride must equal width for Kornia compatibility."
        );

        let size = width * height * C;
        let raw_pixels: &[T] = self.buffer_handle.with_inner(|inner| {
            // SAFETY: The buffer is aligned for T, its length is a multiple of T, and it lives long enough.
            unsafe {
                let data: &[u8] = inner;
                core::slice::from_raw_parts(
                    data.as_ptr() as *const T,
                    data.len() / core::mem::size_of::<T>(),
                )
            }
        });

        // SAFETY: raw_pixels points to size elements laid out for the requested shape.
        unsafe { Image::from_raw_parts([height, width].into(), raw_pixels.as_ptr(), size, k) }
            .map_err(|e| CuError::new_with_cause("Could not create a Kornia Image", e))
    }
}
