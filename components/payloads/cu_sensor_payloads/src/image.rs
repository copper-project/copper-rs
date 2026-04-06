use bincode::de::Decoder;
use bincode::error::DecodeError;
use bincode::{Decode, Encode};
use core::fmt::Debug;
use core::ops::Range;
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CuImagePlaneLayout {
    pub offset_bytes: usize,
    pub row_bytes: u32,
    pub stride_bytes: u32,
    pub height: u32,
}

impl CuImagePlaneLayout {
    pub fn byte_len(&self) -> usize {
        self.stride_bytes as usize * self.height as usize
    }

    pub fn byte_range(&self) -> Range<usize> {
        self.offset_bytes..self.offset_bytes + self.byte_len()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CuImageMemoryLayout {
    Packed { bytes_per_pixel: u32 },
    SemiPlanar420,
    Planar420,
    SinglePlane,
}

impl CuImageBufferFormat {
    fn memory_layout(&self) -> CuImageMemoryLayout {
        match &self.pixel_format {
            b"GRAY" | b"Y800" => CuImageMemoryLayout::Packed { bytes_per_pixel: 1 },
            b"YUYV" | b"UYVY" => CuImageMemoryLayout::Packed { bytes_per_pixel: 2 },
            b"RGB3" | b"BGR3" | b"RGB " | b"BGR " => {
                CuImageMemoryLayout::Packed { bytes_per_pixel: 3 }
            }
            b"RGBA" | b"BGRA" => CuImageMemoryLayout::Packed { bytes_per_pixel: 4 },
            b"NV12" | b"NV21" => CuImageMemoryLayout::SemiPlanar420,
            b"I420" | b"YV12" => CuImageMemoryLayout::Planar420,
            _ => CuImageMemoryLayout::SinglePlane,
        }
    }

    pub fn plane_count(&self) -> usize {
        match self.memory_layout() {
            CuImageMemoryLayout::Planar420 => 3,
            CuImageMemoryLayout::SemiPlanar420 => 2,
            CuImageMemoryLayout::Packed { .. } | CuImageMemoryLayout::SinglePlane => 1,
        }
    }

    pub fn is_packed(&self) -> bool {
        matches!(self.memory_layout(), CuImageMemoryLayout::Packed { .. })
    }

    pub fn packed_row_bytes(&self) -> Option<u32> {
        match self.memory_layout() {
            CuImageMemoryLayout::Packed { bytes_per_pixel } => Some(self.width * bytes_per_pixel),
            CuImageMemoryLayout::SemiPlanar420
            | CuImageMemoryLayout::Planar420
            | CuImageMemoryLayout::SinglePlane => None,
        }
    }

    pub fn plane(&self, index: usize) -> Option<CuImagePlaneLayout> {
        let y_plane_bytes = self.stride as usize * self.height as usize;
        let chroma_height = self.height.div_ceil(2);

        match self.memory_layout() {
            CuImageMemoryLayout::Packed { bytes_per_pixel } if index == 0 => {
                Some(CuImagePlaneLayout {
                    offset_bytes: 0,
                    row_bytes: self.width * bytes_per_pixel,
                    stride_bytes: self.stride,
                    height: self.height,
                })
            }
            CuImageMemoryLayout::SinglePlane if index == 0 => Some(CuImagePlaneLayout {
                offset_bytes: 0,
                row_bytes: self.stride,
                stride_bytes: self.stride,
                height: self.height,
            }),
            CuImageMemoryLayout::SemiPlanar420 if index == 0 => Some(CuImagePlaneLayout {
                offset_bytes: 0,
                row_bytes: self.width,
                stride_bytes: self.stride,
                height: self.height,
            }),
            CuImageMemoryLayout::SemiPlanar420 if index == 1 => Some(CuImagePlaneLayout {
                offset_bytes: y_plane_bytes,
                row_bytes: self.width.div_ceil(2) * 2,
                stride_bytes: self.stride,
                height: chroma_height,
            }),
            CuImageMemoryLayout::Planar420 if index == 0 => Some(CuImagePlaneLayout {
                offset_bytes: 0,
                row_bytes: self.width,
                stride_bytes: self.stride,
                height: self.height,
            }),
            CuImageMemoryLayout::Planar420 if index == 1 => Some({
                let chroma_stride = self.stride.div_ceil(2);
                CuImagePlaneLayout {
                    offset_bytes: y_plane_bytes,
                    row_bytes: self.width.div_ceil(2),
                    stride_bytes: chroma_stride,
                    height: chroma_height,
                }
            }),
            CuImageMemoryLayout::Planar420 if index == 2 => Some({
                let chroma_stride = self.stride.div_ceil(2);
                let chroma_plane_bytes = chroma_stride as usize * chroma_height as usize;
                CuImagePlaneLayout {
                    offset_bytes: y_plane_bytes + chroma_plane_bytes,
                    row_bytes: self.width.div_ceil(2),
                    stride_bytes: chroma_stride,
                    height: chroma_height,
                }
            }),
            _ => None,
        }
    }

    pub fn is_valid(&self) -> bool {
        (0..self.plane_count()).all(|index| {
            self.plane(index)
                .map(|plane| plane.row_bytes <= plane.stride_bytes)
                .unwrap_or(false)
        })
    }

    pub fn required_bytes(&self) -> usize {
        self.plane(self.plane_count().saturating_sub(1))
            .map(|plane| plane.offset_bytes + plane.byte_len())
            .unwrap_or(0)
    }

    pub fn byte_size(&self) -> usize {
        self.required_bytes()
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
            format.is_valid(),
            "Image format layout is invalid for the declared stride."
        );
        assert!(
            format.required_bytes() <= buffer_handle.with_inner(|i| i.len()),
            "Buffer size must at least match the format."
        );
        CuImage {
            seq: 0,
            format,
            buffer_handle,
        }
    }
}

impl<A> CuImage<A>
where
    A: ArrayLike<Element = u8> + Send + Sync + 'static,
{
    pub fn with_plane_bytes<R>(
        &self,
        plane_index: usize,
        f: impl FnOnce(&[u8], CuImagePlaneLayout) -> R,
    ) -> CuResult<R> {
        let plane = self
            .format
            .plane(plane_index)
            .ok_or_else(|| CuError::from(format!("Invalid image plane index {plane_index}")))?;
        Ok(self.buffer_handle.with_inner(|inner| {
            let range = plane.byte_range();
            f(&inner[range], plane)
        }))
    }

    pub fn with_plane_bytes_mut<R>(
        &mut self,
        plane_index: usize,
        f: impl FnOnce(&mut [u8], CuImagePlaneLayout) -> R,
    ) -> CuResult<R> {
        let plane = self
            .format
            .plane(plane_index)
            .ok_or_else(|| CuError::from(format!("Invalid image plane index {plane_index}")))?;
        Ok(self.buffer_handle.with_inner_mut(|inner| {
            let range = plane.byte_range();
            f(&mut inner[range], plane)
        }))
    }

    /// Builds an ImageBuffer from the image crate backed by the CuImage's pixel data.
    #[cfg(feature = "image")]
    pub fn as_image_buffer<P: Pixel>(&self) -> CuResult<ImageBuffer<P, &[P::Subpixel]>> {
        let width = self.format.width;
        let height = self.format.height;
        let plane = self
            .format
            .plane(0)
            .ok_or_else(|| CuError::from("Image format has no addressable planes"))?;
        if self.format.plane_count() != 1 {
            return Err(CuError::from(
                "ImageBuffer compatibility requires a single-plane packed image.",
            ));
        }
        if plane.row_bytes != plane.stride_bytes {
            return Err(CuError::from(
                "ImageBuffer compatibility requires tightly packed rows without padding.",
            ));
        }

        self.with_plane_bytes(0, |data, _| {
            let raw_pixels: &[P::Subpixel] = unsafe {
                core::slice::from_raw_parts(
                    data.as_ptr() as *const P::Subpixel,
                    data.len() / core::mem::size_of::<P::Subpixel>(),
                )
            };
            ImageBuffer::from_raw(width, height, raw_pixels)
                .ok_or("Could not create the image:: buffer".into())
        })?
    }

    #[cfg(feature = "kornia")]
    pub fn as_kornia_image<T: Clone, const C: usize, K: ImageAllocator>(
        &self,
        k: K,
    ) -> CuResult<Image<T, C, K>> {
        let width = self.format.width as usize;
        let height = self.format.height as usize;
        let plane = self
            .format
            .plane(0)
            .ok_or_else(|| CuError::from("Image format has no addressable planes"))?;
        if self.format.plane_count() != 1 {
            return Err(CuError::from(
                "Kornia compatibility requires a single-plane packed image.",
            ));
        }
        if plane.row_bytes != plane.stride_bytes {
            return Err(CuError::from(
                "Kornia compatibility requires tightly packed rows without padding.",
            ));
        }

        let size = width * height * C;
        self.with_plane_bytes(0, |data, _| {
            let raw_pixels: &[T] = unsafe {
                core::slice::from_raw_parts(
                    data.as_ptr() as *const T,
                    data.len() / core::mem::size_of::<T>(),
                )
            };

            unsafe { Image::from_raw_parts([height, width].into(), raw_pixels.as_ptr(), size, k) }
                .map_err(|e| CuError::new_with_cause("Could not create a Kornia Image", e))
        })?
    }
}

#[cfg(test)]
mod tests {
    use super::{CuImageBufferFormat, CuImagePlaneLayout};

    fn assert_plane(
        plane: Option<CuImagePlaneLayout>,
        offset_bytes: usize,
        row_bytes: u32,
        stride_bytes: u32,
        height: u32,
    ) {
        assert_eq!(
            plane,
            Some(CuImagePlaneLayout {
                offset_bytes,
                row_bytes,
                stride_bytes,
                height,
            })
        );
    }

    #[test]
    fn packed_rgb3_layout_uses_single_plane() {
        let format = CuImageBufferFormat {
            width: 4,
            height: 2,
            stride: 12,
            pixel_format: *b"RGB3",
        };

        assert!(format.is_packed());
        assert_eq!(format.plane_count(), 1);
        assert_eq!(format.packed_row_bytes(), Some(12));
        assert_plane(format.plane(0), 0, 12, 12, 2);
        assert_eq!(format.required_bytes(), 24);
        assert!(format.is_valid());
    }

    #[test]
    fn nv12_layout_exposes_two_planes() {
        let format = CuImageBufferFormat {
            width: 640,
            height: 360,
            stride: 640,
            pixel_format: *b"NV12",
        };

        assert!(!format.is_packed());
        assert_eq!(format.plane_count(), 2);
        assert_plane(format.plane(0), 0, 640, 640, 360);
        assert_plane(format.plane(1), 230_400, 640, 640, 180);
        assert_eq!(format.required_bytes(), 345_600);
        assert!(format.is_valid());
    }

    #[test]
    fn i420_layout_exposes_three_planes() {
        let format = CuImageBufferFormat {
            width: 640,
            height: 360,
            stride: 640,
            pixel_format: *b"I420",
        };

        assert_eq!(format.plane_count(), 3);
        assert_plane(format.plane(0), 0, 640, 640, 360);
        assert_plane(format.plane(1), 230_400, 320, 320, 180);
        assert_plane(format.plane(2), 288_000, 320, 320, 180);
        assert_eq!(format.required_bytes(), 345_600);
        assert!(format.is_valid());
    }

    #[test]
    fn invalid_stride_is_detected_for_packed_formats() {
        let format = CuImageBufferFormat {
            width: 4,
            height: 2,
            stride: 4,
            pixel_format: *b"RGB3",
        };

        assert!(!format.is_valid());
        assert_eq!(format.packed_row_bytes(), Some(12));
    }

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
