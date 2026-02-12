use cu_sensor_payloads::{CuImage, CuImageBufferFormat, Distance, PointCloud, PointCloudSoa};
use cu_spatial_payloads::Transform3D;
use cu29::prelude::CuResult;
use rerun::RecordingStream;
use serde::Serialize;

#[test]
fn auto_dispatch_logs_known_payload_types() -> CuResult<()> {
    let rec = RecordingStream::disabled();

    let fmt = CuImageBufferFormat {
        width: 4,
        height: 2,
        stride: 12,
        pixel_format: *b"RGB3",
    };
    let image = CuImage::new(fmt, cu29::prelude::CuHandle::new_detached(vec![0u8; 24]));
    cu29_logviz::log_payload_auto(&rec, "image", &image)?;

    let point = PointCloud::new(cu29::clock::CuTime::from(0u64), 1.0, 2.0, 3.0, 0.0, None);
    cu29_logviz::log_payload_auto(&rec, "point", &point)?;

    let transform = Transform3D::<f32>::from_matrix([
        [1.0, 0.0, 0.0, 0.1],
        [0.0, 1.0, 0.0, 0.2],
        [0.0, 0.0, 1.0, 0.3],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    cu29_logviz::log_payload_auto(&rec, "transform", &transform)?;

    Ok(())
}

#[derive(Serialize)]
struct CustomPayload {
    value: i32,
}

#[test]
fn auto_dispatch_falls_back_for_custom_payloads() -> CuResult<()> {
    let rec = RecordingStream::disabled();
    let payload = CustomPayload { value: 7 };
    cu29_logviz::log_payload_auto(&rec, "custom", &payload)
}

#[test]
fn pointcloud_soa_serialization_shape_is_detectable() {
    let mut pc = PointCloudSoa::<4> {
        len: 2,
        ..Default::default()
    };
    pc.x[0] = Distance::from(1.0);
    pc.y[0] = Distance::from(2.0);
    pc.z[0] = Distance::from(3.0);
    pc.x[1] = Distance::from(4.0);
    pc.y[1] = Distance::from(5.0);
    pc.z[1] = Distance::from(6.0);

    let value = serde_json::to_value(pc).expect("serialize PointCloudSoa");
    let text = value.to_string();
    assert!(text.contains("\"x\""));
    assert!(text.contains("\"y\""));
    assert!(text.contains("\"z\""));
    assert!(text.contains("\"len\":2"));
}
