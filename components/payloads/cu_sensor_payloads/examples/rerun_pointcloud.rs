//! Inspect a Copper point cloud stored behind a handle in Rerun.
use cu_sensor_payloads::{PointCloud, PointCloudSoaHandle};
use cu29::clock::CuTime;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "pointcloud.rrd".into());

    let mut cloud = PointCloudSoaHandle::<4096>::default();
    cloud.with_inner_mut(|points| {
        for row in 0..32 {
            for col in 0..96 {
                let a = col as f32 * core::f32::consts::TAU / 96.0;
                let radius = 1.0 + 0.2 * (row as f32 * 0.4).sin();
                points.push(PointCloud::new(
                    CuTime::from_nanos(0),
                    radius * a.cos(),
                    radius * a.sin(),
                    row as f32 * 0.07,
                    col as f32,
                    None,
                ));
            }
        }
    });
    use cu29::bevy_reflect::Typed;
    use cu29::debug_representation::{DebugRepresentation, DebugRepresentationContext};
    let representation = DebugRepresentation::for_type(PointCloudSoaHandle::<4096>::type_info())
        .expect("point cloud representation");
    let bytes = (representation.encode)(
        &cloud,
        &DebugRepresentationContext {
            entity_path: "cloud",
            culistid: 0,
            time_ns: 0,
        },
    )?;
    std::fs::write(path, bytes)?;
    Ok(())
}
