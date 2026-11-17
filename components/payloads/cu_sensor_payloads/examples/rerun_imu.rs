//! Record sensor-frame directions for inspection in Rerun.
use cu_sensor_payloads::{ImuPayload, ImuWithMagPayload, MagnetometerPayload};
use cu29::bevy_reflect::PartialReflect;
use cu29::debug_representation::{DebugRepresentation, DebugRepresentationContext};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).unwrap_or_else(|| "imu.rrd".into());
    let imu = ImuPayload::from_raw([3.0, 1.0, 9.3], [0.1, 0.3, 0.2], 25.0);
    let mag = MagnetometerPayload::from_raw([20.0, 5.0, -40.0]);
    let value = ImuWithMagPayload::new(imu, Some(mag));
    let representation =
        DebugRepresentation::for_type(value.get_represented_type_info().expect("type info"))
            .expect("Rerun representation");
    let bytes = (representation.encode)(
        &value,
        &DebugRepresentationContext {
            entity_path: "imu",
            culistid: 0,
            time_ns: 0,
        },
    )?;
    std::fs::write(path, bytes)?;
    Ok(())
}
