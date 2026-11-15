//! Sensor-frame directions with explicit magnitudes and units.

use crate::{ImuPayload, ImuWithMagPayload, MagnetometerPayload};
use alloc::{format, vec::Vec};
use cu_rerun::rerun::{Arrows3D, AsComponents, Radius, SerializedComponentBatch};
use cu29::units::si::{
    acceleration::meter_per_second_squared, angular_velocity::radian_per_second,
    magnetic_flux_density::microtesla,
};

fn arrows(
    imu: Option<&ImuPayload>,
    mag: Option<&MagnetometerPayload>,
) -> Vec<SerializedComponentBatch> {
    let mut vectors = Vec::new();
    let mut origins = Vec::new();
    let mut labels = Vec::new();
    let mut colors = Vec::new();
    let mut add = |value: [f32; 3], label: &str, unit: &str, color: [u8; 3], offset: f32| {
        if value.iter().any(|v| !v.is_finite()) {
            return;
        }
        let magnitude = value.iter().map(|v| v * v).sum::<f32>().sqrt();
        vectors.push(value.map(|v| if magnitude > 0.0 { v / magnitude } else { 0.0 }));
        origins.push([offset, 0.0, 0.0]);
        labels.push(format!("{label}: {magnitude:.3} {unit}"));
        colors.push(color);
    };
    if let Some(imu) = imu {
        add(
            [
                imu.accel_x.get::<meter_per_second_squared>(),
                imu.accel_y.get::<meter_per_second_squared>(),
                imu.accel_z.get::<meter_per_second_squared>(),
            ],
            "Acceleration",
            "m/s²",
            [243, 139, 168],
            0.0,
        );
        add(
            [
                imu.gyro_x.get::<radian_per_second>(),
                imu.gyro_y.get::<radian_per_second>(),
                imu.gyro_z.get::<radian_per_second>(),
            ],
            "Angular velocity",
            "rad/s",
            [166, 227, 161],
            2.0,
        );
    }
    if let Some(mag) = mag {
        add(
            [
                mag.mag_x.get::<microtesla>(),
                mag.mag_y.get::<microtesla>(),
                mag.mag_z.get::<microtesla>(),
            ],
            "Magnetic field",
            "µT",
            [137, 180, 250],
            4.0,
        );
    }
    Arrows3D::from_vectors(vectors)
        .with_origins(origins)
        .with_labels(labels)
        .with_colors(colors)
        .with_radii([Radius::new_ui_points(2.0)])
        .as_serialized_batches()
}

impl AsComponents for ImuPayload {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        arrows(Some(self), None)
    }
}
impl AsComponents for MagnetometerPayload {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        arrows(None, Some(self))
    }
}
impl AsComponents for ImuWithMagPayload {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        arrows(Some(&self.imu), self.mag.as_ref())
    }
}
