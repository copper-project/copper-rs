use cu_sensor_payloads::{CuImage, CuImageBufferFormat, PointCloud};
use cu_spatial_payloads::Transform3D;
use cu29::prelude::CuResult;
use cu29_logviz::log_as_components;
use rerun::AsComponents;

#[test]
fn payload_image_emits_components() {
    let fmt = CuImageBufferFormat {
        width: 4,
        height: 2,
        stride: 12,
        pixel_format: *b"RGB3",
    };
    let image = CuImage::new(fmt, cu29::prelude::CuHandle::new_detached(vec![0u8; 24]));

    assert!(!image.as_serialized_batches().is_empty());
}

#[test]
fn payload_point_emits_components() {
    let point = PointCloud::new(cu29::clock::CuTime::from(0u64), 1.0, 2.0, 3.0, 0.0, None);

    assert!(!point.as_serialized_batches().is_empty());
}

#[test]
fn payload_transform_f32_emits_components() {
    let transform = Transform3D::<f32>::from_matrix([
        [1.0, 0.0, 0.0, 0.1],
        [0.0, 1.0, 0.0, 0.2],
        [0.0, 0.0, 1.0, 0.3],
        [0.0, 0.0, 0.0, 1.0],
    ]);

    assert!(!transform.as_serialized_batches().is_empty());
}

#[test]
fn payload_transform_f64_emits_components() {
    let transform = Transform3D::<f64>::from_matrix([
        [1.0, 0.0, 0.0, 1.1],
        [0.0, 1.0, 0.0, 1.2],
        [0.0, 0.0, 1.0, 1.3],
        [0.0, 0.0, 0.0, 1.0],
    ]);

    assert!(!transform.as_serialized_batches().is_empty());
}

#[test]
fn log_as_components_accepts_payload() -> CuResult<()> {
    let fmt = CuImageBufferFormat {
        width: 4,
        height: 2,
        stride: 12,
        pixel_format: *b"RGB3",
    };
    let image = CuImage::new(fmt, cu29::prelude::CuHandle::new_detached(vec![0u8; 24]));
    let rec = rerun::RecordingStream::disabled();

    log_as_components(&rec, "image", &image)
}
