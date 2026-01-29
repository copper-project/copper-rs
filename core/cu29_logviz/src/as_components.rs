use cu_sensor_payloads::{CuImage, Distance, PointCloud};
use cu_spatial_payloads::Transform3D;
use rerun::{AsComponents, Points3D, SerializedComponentBatch};

use crate::{build_rerun_image, build_rerun_transform};

pub struct LogvizImageVec<'a> {
    inner: &'a CuImage<Vec<u8>>,
}

impl<'a> LogvizImageVec<'a> {
    pub fn new(inner: &'a CuImage<Vec<u8>>) -> Self {
        Self { inner }
    }
}

impl<'a> AsComponents for LogvizImageVec<'a> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let image = build_rerun_image(self.inner);
        image.as_serialized_batches()
    }
}

pub struct LogvizPoint<'a> {
    inner: &'a PointCloud,
}

impl<'a> LogvizPoint<'a> {
    pub fn new(inner: &'a PointCloud) -> Self {
        Self { inner }
    }
}

impl<'a> AsComponents for LogvizPoint<'a> {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let Distance(x) = self.inner.x;
        let Distance(y) = self.inner.y;
        let Distance(z) = self.inner.z;
        Points3D::new([[x.value, y.value, z.value]]).as_serialized_batches()
    }
}

pub struct LogvizTransform<'a, T: Copy + core::fmt::Debug + 'static> {
    inner: &'a Transform3D<T>,
}

impl<'a, T: Copy + core::fmt::Debug + 'static> LogvizTransform<'a, T> {
    pub fn new(inner: &'a Transform3D<T>) -> Self {
        Self { inner }
    }
}

impl<'a, T: Copy + Into<f64> + core::fmt::Debug + Default + 'static> AsComponents
    for LogvizTransform<'a, T>
{
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let rr_transform = build_rerun_transform(self.inner);
        rr_transform.as_serialized_batches()
    }
}
