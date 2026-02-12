#![cfg(feature = "rerun")]

use cu_sensor_payloads::{CuImage, CuImageBufferFormat, Distance, PointCloud, PointCloudSoa};
use cu29::clock::CuTime;
use cu29::prelude::CuHandle;
use rerun::AsComponents;

#[test]
fn cu_image_emits_rerun_components() {
    let format = CuImageBufferFormat {
        width: 4,
        height: 2,
        stride: 12,
        pixel_format: *b"RGB3",
    };
    let image = CuImage::new(format, CuHandle::new_detached(vec![0u8; 24]));

    assert!(!image.as_serialized_batches().is_empty());
}

#[test]
fn pointcloud_emits_rerun_components() {
    let point = PointCloud::new(CuTime::from(0u64), 1.0, 2.0, 3.0, 0.0, None);

    assert!(!point.as_serialized_batches().is_empty());
}

#[test]
fn pointcloud_soa_emits_rerun_components() {
    let mut pointcloud = PointCloudSoa::<2> {
        len: 2,
        ..Default::default()
    };
    pointcloud.x[0] = Distance::from(1.0);
    pointcloud.y[0] = Distance::from(2.0);
    pointcloud.z[0] = Distance::from(3.0);
    pointcloud.x[1] = Distance::from(4.0);
    pointcloud.y[1] = Distance::from(5.0);
    pointcloud.z[1] = Distance::from(6.0);

    assert!(!pointcloud.as_serialized_batches().is_empty());
}
