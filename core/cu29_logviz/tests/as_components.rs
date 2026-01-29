use cu_sensor_payloads::{CuImage, CuImageBufferFormat, PointCloud};
use cu_spatial_payloads::Transform3D;
use cu29::prelude::CuResult;
use cu29_logviz::as_components::{LogvizImageVec, LogvizPoint, LogvizTransform};
use cu29_logviz::log_as_components;
use rerun::AsComponents;

#[test]
fn logviz_image_vec_emits_components() {
    let fmt = CuImageBufferFormat {
        width: 4,
        height: 2,
        stride: 12,
        pixel_format: *b"RGB3",
    };
    let img = CuImage::new(fmt, cu29::prelude::CuHandle::new_detached(vec![0u8; 24]));
    let wrapper = LogvizImageVec::new(&img);
    assert!(!wrapper.as_serialized_batches().is_empty());
}

#[test]
fn logviz_point_emits_components() {
    let pc = PointCloud::new(cu29::clock::CuTime::from(0u64), 1.0, 2.0, 3.0, 0.0, None);
    let wrapper = LogvizPoint::new(&pc);
    assert!(!wrapper.as_serialized_batches().is_empty());
}

#[test]
fn logviz_transform_f32_emits_components() {
    let t = Transform3D::<f32>::from_matrix([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [1.0, 2.0, 3.0, 1.0],
    ]);
    let wrapper = LogvizTransform::new(&t);
    assert!(!wrapper.as_serialized_batches().is_empty());
}

#[test]
fn logviz_transform_f64_emits_components() {
    let t = Transform3D::<f64>::from_matrix([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [1.0, 2.0, 3.0, 1.0],
    ]);
    let wrapper = LogvizTransform::new(&t);
    assert!(!wrapper.as_serialized_batches().is_empty());
}

#[test]
fn log_as_components_accepts_wrapper() -> CuResult<()> {
    let fmt = CuImageBufferFormat {
        width: 4,
        height: 2,
        stride: 12,
        pixel_format: *b"RGB3",
    };
    let img = CuImage::new(fmt, cu29::prelude::CuHandle::new_detached(vec![0u8; 24]));
    let wrapper = LogvizImageVec::new(&img);
    let rec = rerun::RecordingStream::disabled();
    log_as_components(&rec, "image", &wrapper)
}
