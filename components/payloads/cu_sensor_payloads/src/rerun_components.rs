use alloc::vec::Vec;

use cu29::prelude::ArrayLike;
use rerun::components::{ImageBuffer, ImageFormat};
use rerun::datatypes::{Blob, ImageFormat as RerunImageFormat};
use rerun::{
    AsComponents, ChannelDatatype, ColorModel, Image, PixelFormat, Points3D, Position3D,
    SerializedComponentBatch,
};

use crate::{CuImage, CuImageBufferFormat, PointCloud, PointCloudSoa};

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
        b"RGB3" => (None, Some(ColorModel::RGB), Some(ChannelDatatype::U8)),
        b"BGR3" => (None, Some(ColorModel::BGR), Some(ChannelDatatype::U8)),
        _ => (None, None, None),
    }
}

impl<A> AsComponents for CuImage<A>
where
    A: ArrayLike<Element = u8>,
{
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let (pixel_format, color_model, channel_datatype) = image_format_from_cu(self.format);
        let format = ImageFormat(RerunImageFormat {
            width: self.format.width,
            height: self.format.height,
            pixel_format,
            color_model,
            channel_datatype,
        });
        let blob = self
            .buffer_handle
            .with_inner(|inner| Blob::from(&inner[..]));
        let image_buffer = ImageBuffer::from(blob);

        Image::new(image_buffer, format).as_serialized_batches()
    }
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
    use crate::Distance;
    use cu29::units::si::length::meter;

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
