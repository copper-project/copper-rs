//! Record a GNSS fix through Copper's debug representation for Rerun's map view.
use cu_gnss_payloads::{GeodeticPosition, GnssFixSolution, GnssFixType, GnssNavEpoch};
use cu29::bevy_reflect::PartialReflect;
use cu29::debug_representation::{DebugRepresentation, DebugRepresentationContext};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).unwrap_or_else(|| "gnss.rrd".into());
    let value = GnssNavEpoch {
        fix: GnssFixSolution {
            position: GeodeticPosition::from_degrees(41.878, -87.630),
            fix_type: GnssFixType::Fix3D,
            gnss_fix_ok: true,
            invalid_llh: false,
            carrier_solution: 2,
            ..Default::default()
        },
        ..Default::default()
    };
    let representation =
        DebugRepresentation::for_type(value.get_represented_type_info().expect("type info"))
            .expect("Rerun representation");
    let bytes = (representation.encode)(
        &value,
        &DebugRepresentationContext {
            entity_path: "gnss",
            culistid: 0,
            time_ns: 0,
        },
    )?;
    std::fs::write(path, bytes)?;
    Ok(())
}
