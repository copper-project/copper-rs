//! Record translated and rotated coordinate frames for inspection in Rerun.
use cu_spatial_payloads::Transform3D;
use cu29::bevy_reflect::PartialReflect;
use cu29::debug_representation::{DebugRepresentation, DebugRepresentationContext};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "poses.rrd".into());
    let pose = Transform3D::<f32>::from_matrix([
        [0.866, -0.5, 0.0, 2.0],
        [0.5, 0.866, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let value = pose;
    let representation =
        DebugRepresentation::for_type(value.get_represented_type_info().expect("type info"))
            .expect("Rerun representation");
    let bytes = (representation.encode)(
        &value,
        &DebugRepresentationContext {
            entity_path: "pose",
            culistid: 0,
            time_ns: 0,
        },
    )?;
    std::fs::write(path, bytes)?;
    Ok(())
}
