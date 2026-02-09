use cu_sensor_payloads::{CuImage, CuImageBufferFormat, PointCloud};
use cu_spatial_payloads::Transform3D;
use cu29::prelude::CuResult;
use cu29_logviz::log_as_components;

#[test]
fn log_as_components_accepts_payload_image() -> CuResult<()> {
    let format = CuImageBufferFormat {
        width: 4,
        height: 2,
        stride: 12,
        pixel_format: *b"RGB3",
    };
    let image = CuImage::new(format, cu29::prelude::CuHandle::new_detached(vec![0u8; 24]));
    let rec = rerun::RecordingStream::disabled();

    log_as_components(&rec, "image", &image)
}

#[test]
fn log_as_components_accepts_payload_pointcloud() -> CuResult<()> {
    let point = PointCloud::new(cu29::clock::CuTime::from(0u64), 1.0, 2.0, 3.0, 0.0, None);
    let rec = rerun::RecordingStream::disabled();

    log_as_components(&rec, "point", &point)
}

#[test]
fn log_as_components_accepts_payload_transform() -> CuResult<()> {
    let transform = Transform3D::<f32>::from_matrix([
        [1.0, 0.0, 0.0, 0.1],
        [0.0, 1.0, 0.0, 0.2],
        [0.0, 0.0, 1.0, 0.3],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let rec = rerun::RecordingStream::disabled();

    log_as_components(&rec, "transform", &transform)
}
