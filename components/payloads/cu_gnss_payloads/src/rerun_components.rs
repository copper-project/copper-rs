//! Geographic positions with fix validity and quality.

use crate::{GnssFixSolution, GnssFixType, GnssNavEpoch};
use alloc::vec::Vec;
use cu_rerun::rerun::{AsComponents, GeoPoints, SerializedComponentBatch};

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
        let (label, color) = match self.carrier_solution {
            2 => ("RTK fixed", [166, 227, 161]),
            1 => ("RTK float", [203, 166, 247]),
            _ if self.differential_solution => ("Differential", [137, 180, 250]),
            _ => ("GNSS", [243, 139, 168]),
        };
        GeoPoints::from_lat_lon(points)
            .with_colors([color])
            .with_labels([label])
            .as_serialized_batches()
    }
}

impl AsComponents for GnssNavEpoch {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        self.fix.as_serialized_batches()
    }
}
