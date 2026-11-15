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
        b"NV12" | b"NV21" => (Some(PixelFormat::NV12), None, None),
        b"I420" | b"YV12" => (Some(PixelFormat::Y_U_V12_LimitedRange), None, None),
        b"YUYV" | b"YUY2" | b"UYVY" => (Some(PixelFormat::YUY2), None, None),
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
            return rerun::Clear::flat().as_serialized_batches();
        }
        let Some(bytes) = packed_image_bytes(self) else {
            return rerun::Clear::flat().as_serialized_batches();
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
    if matches!(&format.pixel_format, b"NV12" | b"NV21" | b"I420" | b"YV12")
        && (!format.width.is_multiple_of(2) || !format.height.is_multiple_of(2))
    {
        return None;
    }
    if matches!(&format.pixel_format, b"YUYV" | b"UYVY") && !format.width.is_multiple_of(2) {
        return None;
    }
    image.buffer_handle.with_inner(|inner| {
        let mut bytes = Vec::new();
        for index in 0..format.plane_count() {
            // YV12 stores V before U; Rerun's planar format stores U before V.
            let source_index = if format.pixel_format == *b"YV12" && index > 0 {
                3 - index
            } else {
                index
            };
            let plane = format.plane(source_index)?;
            for row in 0..plane.height as usize {
                let start = plane
                    .offset_bytes
                    .checked_add(row.checked_mul(plane.stride_bytes as usize)?)?;
                let end = start.checked_add(plane.row_bytes as usize)?;
                bytes.extend_from_slice(inner.get(start..end)?);
            }
        }
        match &format.pixel_format {
            b"UYVY" => {
                for pair in bytes.as_chunks_mut::<4>().0 {
                    pair.swap(0, 1);
                    pair.swap(2, 3);
                }
            }
            b"NV21" => {
                let luma_len = (format.width as usize).checked_mul(format.height as usize)?;
                for pair in bytes.get_mut(luma_len..)?.as_chunks_mut::<2>().0 {
                    pair.swap(0, 1);
                }
            }
            _ => {}
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
        let short = CuImage {
            seq: 0,
            format,
            buffer_handle: cu29::prelude::CuHandle::new_detached(vec![0; 10]),
        };
        assert_eq!(
            short.as_serialized_batches()[0].descriptor,
            rerun::Clear::descriptor_is_recursive()
        );
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
    fn yuv_formats_remove_padding_and_preserve_channel_order() {
        let cases = [
            (
                *b"UYVY",
                6,
                vec![10, 1, 20, 2, 99, 99, 30, 3, 40, 4, 99, 99],
                vec![1, 10, 2, 20, 3, 30, 4, 40],
                PixelFormat::YUY2,
            ),
            (
                *b"NV21",
                4,
                vec![1, 2, 99, 99, 3, 4, 99, 99, 20, 10, 99, 99],
                vec![1, 2, 3, 4, 10, 20],
                PixelFormat::NV12,
            ),
            (
                *b"I420",
                4,
                vec![1, 2, 99, 99, 3, 4, 99, 99, 10, 99, 20, 99],
                vec![1, 2, 3, 4, 10, 20],
                PixelFormat::Y_U_V12_LimitedRange,
            ),
            (
                *b"YV12",
                4,
                vec![1, 2, 99, 99, 3, 4, 99, 99, 20, 99, 10, 99],
                vec![1, 2, 3, 4, 10, 20],
                PixelFormat::Y_U_V12_LimitedRange,
            ),
        ];
        for (pixel_format, stride, data, expected, rerun_format) in cases {
            let format = CuImageBufferFormat {
                width: 2,
                height: 2,
                stride,
                pixel_format,
            };
            let image = CuImage::new(format, cu29::prelude::CuHandle::new_detached(data));
            assert_eq!(
                packed_image_bytes(&image),
                Some(expected),
                "{pixel_format:?}"
            );
            assert_eq!(image_format_from_cu(format).0, Some(rerun_format));
            assert!(
                image
                    .as_serialized_batches()
                    .iter()
                    .any(|batch| batch.descriptor == Image::descriptor_buffer())
            );
            let short = CuImage {
                seq: 0,
                format,
                buffer_handle: cu29::prelude::CuHandle::new_detached(vec![0; 2]),
            };
            assert_eq!(
                short.as_serialized_batches()[0].descriptor,
                rerun::Clear::descriptor_is_recursive()
            );
        }
    }

    #[test]
    fn subsampled_images_reject_unsupported_odd_dimensions() {
        for pixel_format in [*b"UYVY", *b"NV21", *b"I420", *b"YV12"] {
            let image = CuImage::new(
                CuImageBufferFormat {
                    width: 3,
                    height: 2,
                    stride: 8,
                    pixel_format,
                },
                cu29::prelude::CuHandle::new_detached(vec![0; 32]),
            );
            assert!(packed_image_bytes(&image).is_none());
        }
    }

    #[test]
    fn unsupported_image_formats_clear_previous_image() {
        let format = CuImageBufferFormat {
            width: 2,
            height: 2,
            stride: 4,
            pixel_format: *b"MJPG",
        };
        let image = CuImage::new(format, cu29::prelude::CuHandle::new_detached(vec![0; 8]));
        assert_eq!(
            image.as_serialized_batches()[0].descriptor,
            rerun::Clear::descriptor_is_recursive()
        );
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
