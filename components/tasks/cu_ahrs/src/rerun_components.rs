//! AHRS orientation rendered as the aerospace body axes in the navigation frame.

use crate::{AhrsPose, CuAhrsDebugState};
use alloc::vec::Vec;
use cu_rerun::rerun::{Arrows3D, AsComponents, Radius, SerializedComponentBatch};
use cu29::units::si::{angle::radian, ratio::ratio};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

fn axes(orientation: UnitQuaternion<f32>) -> Vec<SerializedComponentBatch> {
    let directions = [Vector3::x(), Vector3::y(), Vector3::z()].map(|axis| {
        let direction = orientation.transform_vector(&axis);
        [direction.x, direction.y, direction.z]
    });
    Arrows3D::from_vectors(directions)
        .with_origins([[0.0, 0.0, 0.0]; 3])
        .with_colors([[243, 139, 168], [166, 227, 161], [137, 180, 250]])
        .with_labels(["Forward", "Right", "Down"])
        .with_radii([Radius::new_ui_points(2.0)])
        .as_serialized_batches()
}

impl AsComponents for AhrsPose {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        axes(UnitQuaternion::from_euler_angles(
            self.roll.get::<radian>(),
            self.pitch.get::<radian>(),
            self.yaw.get::<radian>(),
        ))
    }
}

impl AsComponents for CuAhrsDebugState {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        axes(UnitQuaternion::new_normalize(Quaternion::new(
            self.orientation_w.get::<ratio>(),
            self.orientation_i.get::<ratio>(),
            self.orientation_j.get::<ratio>(),
            self.orientation_k.get::<ratio>(),
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CuAhrs;
    use cu_rerun::rerun::{FromArrow, components::Vector3D as RerunVector};
    use cu29::units::si::f32::Angle;

    fn vectors(value: &impl AsComponents) -> Vec<[f32; 3]> {
        let batch = value
            .as_serialized_batches()
            .into_iter()
            .find(|batch| batch.descriptor == Arrows3D::descriptor_vectors())
            .unwrap();
        RerunVector::from_arrow(&batch.array)
            .unwrap()
            .iter()
            .map(|v| v.0.0)
            .collect()
    }

    #[test]
    fn euler_and_quaternion_representations_rotate_the_body_axes() {
        let pose = AhrsPose {
            yaw: Angle::new::<radian>(core::f32::consts::FRAC_PI_2),
            ..Default::default()
        };
        let mut state = CuAhrs::new_filter().debug_state();
        state.orientation_w =
            cu29::units::si::f32::Ratio::new::<ratio>(core::f32::consts::FRAC_1_SQRT_2);
        state.orientation_k = state.orientation_w;
        let expected = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        for actual in [vectors(&pose), vectors(&state)] {
            for (actual, expected) in actual.iter().zip(expected) {
                for (a, e) in actual.iter().zip(expected) {
                    assert!((a - e).abs() < 1e-5);
                }
            }
        }
        let recording = cu_rerun::rerun::RecordingStreamBuilder::new("ahrs-local")
            .buffered()
            .unwrap();
        recording.log("pose", &pose).unwrap();
        recording.log("state", &state).unwrap();
    }
}
