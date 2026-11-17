//! Geographic positions with fix validity and quality.

use crate::{GnssFixSolution, GnssFixType, GnssNavEpoch};
use alloc::vec::Vec;
use cu_rerun::rerun::{AsComponents, GeoPoints, Radius, SerializedComponentBatch};

impl AsComponents for GnssFixSolution {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let latitude = self.position.latitude_degrees();
        let longitude = self.position.longitude_degrees();
        let valid = self.gnss_fix_ok
            && !self.invalid_llh
            && matches!(
                self.fix_type,
                GnssFixType::Fix2D | GnssFixType::Fix3D | GnssFixType::GnssDeadReckoningCombined
            )
            && (-90.0..=90.0).contains(&latitude)
            && (-180.0..=180.0).contains(&longitude);
        let points = if valid {
            vec![(latitude, longitude)]
        } else {
            Vec::new()
        };
        let color = match self.carrier_solution {
            2 => [166, 227, 161],
            1 => [203, 166, 247],
            _ if self.differential_solution => [137, 180, 250],
            _ => [243, 139, 168],
        };
        GeoPoints::from_lat_lon(points)
            .with_colors([color])
            .with_radii([Radius::new_ui_points(8.0)])
            .as_serialized_batches()
    }
}

impl AsComponents for GnssNavEpoch {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        self.fix.as_serialized_batches()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GeodeticPosition;
    use cu_rerun::rerun::{FromArrow, components::LatLon};
    use cu29::bevy_reflect::Reflect;
    use cu29::debug_representation::{DebugRepresentation, DebugRepresentationContext};

    fn coordinates(fix: &GnssFixSolution) -> Vec<LatLon> {
        let batch = fix
            .as_serialized_batches()
            .into_iter()
            .find(|batch| batch.descriptor == GeoPoints::descriptor_positions())
            .unwrap();
        LatLon::from_arrow(&batch.array).unwrap()
    }

    #[test]
    fn invalid_fixes_clear_points_and_valid_fixes_keep_coordinates() {
        let mut fix = GnssFixSolution::default();
        assert!(coordinates(&fix).is_empty());
        fix.position = GeodeticPosition::from_degrees(41.878, -87.630);
        fix.fix_type = GnssFixType::Fix3D;
        fix.gnss_fix_ok = true;
        fix.invalid_llh = false;
        assert_eq!(coordinates(&fix)[0].0.0, [41.878, -87.630]);
        fix.position = GeodeticPosition::from_degrees(f64::NAN, 0.0);
        assert!(coordinates(&fix).is_empty());
    }

    #[test]
    fn debugger_discovers_geodetic_fix_and_epoch() {
        let position = GeodeticPosition::from_degrees(41.878, -87.630);
        let fix = GnssFixSolution::default();
        let epoch = GnssNavEpoch::default();
        for value in [&position as &dyn Reflect, &fix, &epoch] {
            let representation =
                DebugRepresentation::for_type(value.get_represented_type_info().unwrap()).unwrap();
            let bytes = (representation.encode)(
                value,
                &DebugRepresentationContext {
                    entity_path: "position",
                    culistid: 42,
                    time_ns: 1_000,
                },
            )
            .unwrap();
            assert!(!bytes.is_empty());
        }
    }
}
