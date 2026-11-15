//! Rerun components for sensor payloads.
use crate::{CuImage, CuImageBufferFormat, PointCloud, PointCloudSoa};
use alloc::vec::Vec;
use cu29::prelude::ArrayLike;
use rerun::components::{ImageBuffer, ImageFormat};
use rerun::datatypes::{Blob, ImageFormat as RerunImageFormat};
use rerun::{
    AsComponents, ChannelDatatype, ColorModel, Image, PixelFormat, Points3D, Position3D,
    SerializedComponentBatch,
};

fn image_format_from_cu(
    fmt: CuImageBufferFormat,
) -> (
    Option<PixelFormat>,
    Option<ColorModel>,
    Option<ChannelDatatype>,
) {
    match &fmt.pixel_format {
        b"NV12" => (Some(PixelFormat::NV12), None, None),
        b"YUYV" | b"YUY2" => (Some(PixelFormat::YUY2), None, None),
        b"RGB3" | b"RGB " => (None, Some(ColorModel::RGB), Some(ChannelDatatype::U8)),
        b"BGR3" | b"BGR " => (None, Some(ColorModel::BGR), Some(ChannelDatatype::U8)),
        b"GRAY" | b"Y800" => (None, Some(ColorModel::L), Some(ChannelDatatype::U8)),
        b"RGBA" => (None, Some(ColorModel::RGBA), Some(ChannelDatatype::U8)),
        b"BGRA" => (None, Some(ColorModel::BGRA), Some(ChannelDatatype::U8)),
        _ => (None, None, None),
    }
}

impl<A> AsComponents for CuImage<A>
where
    A: ArrayLike<Element = u8>,
{
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let (pixel_format, color_model, channel_datatype) = image_format_from_cu(self.format);
        if pixel_format.is_none() && color_model.is_none() {
            return Vec::new();
        }
        let Some(bytes) = packed_image_bytes(self) else {
            return Vec::new();
        };
        let format = ImageFormat(RerunImageFormat {
            width: self.format.width,
            height: self.format.height,
            pixel_format,
            color_model,
            channel_datatype,
        });
        let blob = Blob::from(bytes);
        let image_buffer = ImageBuffer::from(blob);

        Image::new(image_buffer, format).as_serialized_batches()
    }
}

// Rerun images are tightly packed; Copper strides are bytes and may include padding.
fn packed_image_bytes<A: ArrayLike<Element = u8>>(image: &CuImage<A>) -> Option<Vec<u8>> {
    let mut format = image.format;
    if format.pixel_format == *b"YUY2" {
        format.pixel_format = *b"YUYV";
    }
    if format.width == 0 || format.height == 0 || !format.is_valid() {
        return None;
    }
    if matches!(&format.pixel_format, b"NV12")
        && (!format.width.is_multiple_of(2) || !format.height.is_multiple_of(2))
    {
        return None;
    }
    if matches!(&format.pixel_format, b"YUYV") && !format.width.is_multiple_of(2) {
        return None;
    }
    image.buffer_handle.with_inner(|inner| {
        let mut bytes = Vec::new();
        for index in 0..format.plane_count() {
            let plane = format.plane(index)?;
            for row in 0..plane.height as usize {
                let start = plane
                    .offset_bytes
                    .checked_add(row.checked_mul(plane.stride_bytes as usize)?)?;
                let end = start.checked_add(plane.row_bytes as usize)?;
                bytes.extend_from_slice(inner.get(start..end)?);
            }
        }
        Some(bytes)
    })
}

impl AsComponents for PointCloud {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        Points3D::new([[self.x.value, self.y.value, self.z.value]]).as_serialized_batches()
    }
}

impl<const N: usize> AsComponents for PointCloudSoa<N> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        Points3D::new(pointcloud_positions(self)).as_serialized_batches()
    }
}

fn pointcloud_positions<const N: usize>(pointcloud: &PointCloudSoa<N>) -> Vec<Position3D> {
    let len = pointcloud.len.min(N);
    let mut points = Vec::with_capacity(len);

    for i in 0..len {
        points.push(Position3D::new(
            pointcloud.x[i].value,
            pointcloud.y[i].value,
            pointcloud.z[i].value,
        ));
    }

    points
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CuImageBufferFormat, Distance};
    use cu29::units::si::length::meter;
    use rerun::{ChannelDatatype, ColorModel};

    #[test]
    fn image_padding_is_removed_and_short_buffers_are_rejected() {
        let format = CuImageBufferFormat {
            width: 2,
            height: 2,
            stride: 8,
            pixel_format: *b"RGB3",
        };
        let image = CuImage::new(
            format,
            cu29::prelude::CuHandle::new_detached(vec![
                1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 99, 99,
            ]),
        );
        assert_eq!(
            packed_image_bytes(&image),
            Some(vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
        );
        let short = CuImage::new(format, cu29::prelude::CuHandle::new_detached(vec![0; 10]));
        assert!(short.as_serialized_batches().is_empty());
    }

    #[test]
    fn nv12_padding_is_removed_from_both_planes() {
        let format = CuImageBufferFormat {
            width: 2,
            height: 2,
            stride: 4,
            pixel_format: *b"NV12",
        };
        let image = CuImage::new(
            format,
            cu29::prelude::CuHandle::new_detached(vec![1, 2, 99, 99, 3, 4, 99, 99, 5, 6, 99, 99]),
        );
        assert_eq!(packed_image_bytes(&image), Some(vec![1, 2, 3, 4, 5, 6]));
    }

    #[test]
    fn unsupported_image_formats_emit_no_components() {
        let format = CuImageBufferFormat {
            width: 2,
            height: 2,
            stride: 4,
            pixel_format: *b"MJPG",
        };
        let image = CuImage::new(format, cu29::prelude::CuHandle::new_detached(vec![0; 8]));
        assert!(image.as_serialized_batches().is_empty());
    }

    #[test]
    fn image_format_rgb3_maps_to_color_model() {
        let format = CuImageBufferFormat {
            width: 2,
            height: 1,
            stride: 2,
            pixel_format: *b"RGB3",
        };
        let (pixel_format, color_model, channel_datatype) = image_format_from_cu(format);

        assert!(pixel_format.is_none());
        assert_eq!(color_model, Some(ColorModel::RGB));
        assert_eq!(channel_datatype, Some(ChannelDatatype::U8));
    }

    #[test]
    fn pointcloud_positions_respects_len() {
        let mut pointcloud = PointCloudSoa::<3> {
            len: 1,
            ..Default::default()
        };
        pointcloud.x[0] = Distance::new::<meter>(1.0);
        pointcloud.y[0] = Distance::new::<meter>(2.0);
        pointcloud.z[0] = Distance::new::<meter>(3.0);
        pointcloud.x[1] = Distance::new::<meter>(9.0);
        pointcloud.y[1] = Distance::new::<meter>(9.0);
        pointcloud.z[1] = Distance::new::<meter>(9.0);

        let points = pointcloud_positions(&pointcloud);
        assert_eq!(points.len(), 1);
        assert_eq!(points[0], Position3D::new(1.0, 2.0, 3.0));
    }
}
