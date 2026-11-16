//! Tendon lengths -> constant-curvature sections -> ordinary Rerun components.
//!
//! Model: https://www.opencontinuumrobotics.com/101/2022/12/09/tdcr-cc-model.html
//! Each section has three equally spaced tendons and bends without twisting.

use crate::{SoftArm, TendonSection};
use cu_rerun::rerun::{Arrows3D, AsComponents, LineStrips3D, Points3D, SerializedComponentBatch};
use cu29::units::si::length::meter;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use std::f32::consts::TAU;

const SAMPLES: usize = 32;
const SECTION_COLORS: [[u8; 3]; 3] = [[203, 166, 247], [166, 227, 161], [243, 139, 168]];
const TENDON_COLORS: [[u8; 3]; 3] = [[243, 139, 168], [166, 227, 161], [137, 180, 250]];

/// Recover the bend from the tendon lengths, then evaluate the frame along the arc.
fn frame(section: &TendonSection, fraction: f32) -> Isometry3<f32> {
    let lengths = section.tendon_lengths.map(|length| length.get::<meter>());
    let length = lengths.iter().sum::<f32>() / 3.0;
    let radius = section.routing_radius.get::<meter>();
    let mut bend = Vector3::zeros();
    for (tendon, tendon_length) in lengths.iter().enumerate() {
        let angle = tendon as f32 * TAU / 3.0;
        let shortening = length - tendon_length;
        bend.x += shortening * angle.cos();
        bend.y += shortening * angle.sin();
    }
    bend *= 2.0 / (3.0 * radius);
    let theta = bend.norm();
    let (offset, rotation) = if theta < 1e-5 {
        (
            Vector3::new(0.0, 0.0, length * fraction),
            UnitQuaternion::identity(),
        )
    } else {
        let angle = theta * fraction;
        // 2 sin²(angle/2) avoids cancellation near a straight configuration.
        let sideways = length / theta * 2.0 * (0.5 * angle).sin().powi(2);
        let offset = Vector3::new(
            sideways * bend.x / theta,
            sideways * bend.y / theta,
            length / theta * angle.sin(),
        );
        let rotation =
            UnitQuaternion::from_scaled_axis(Vector3::new(-bend.y, bend.x, 0.0) * fraction);
        (offset, rotation)
    };
    Isometry3::from_parts(Translation3::from(offset), rotation)
}

fn point(frame: &Isometry3<f32>, local: [f32; 3]) -> [f32; 3] {
    let p = frame.transform_point(&Point3::from(local));
    [p.x, p.y, p.z]
}

impl AsComponents for SoftArm {
    fn as_serialized_batches(&self) -> Vec<SerializedComponentBatch> {
        let mut strips = Vec::new();
        let mut colors = Vec::new();
        let mut radii = Vec::new();
        let mut joints = vec![[0.0, 0.0, 0.0]];
        let mut base = Isometry3::identity();

        for (section_index, section) in self.sections.iter().enumerate() {
            let radius = section.routing_radius.get::<meter>();
            // Missing or invalid measurements must not produce NaN geometry.
            if !radius.is_finite()
                || radius <= 0.0
                || section
                    .tendon_lengths
                    .iter()
                    .any(|l| !l.value.is_finite() || l.value <= 0.0)
            {
                break;
            }
            let frames = (0..=SAMPLES)
                .map(|sample| base * frame(section, sample as f32 / SAMPLES as f32))
                .collect::<Vec<_>>();
            strips.push(
                frames
                    .iter()
                    .map(|pose| point(pose, [0.0; 3]))
                    .collect::<Vec<_>>(),
            );
            colors.push(SECTION_COLORS[section_index]);
            radii.push(0.009);

            // Each tendon follows an offset curve; the rings expose local orientation.
            for (tendon, color) in TENDON_COLORS.iter().enumerate() {
                let angle = tendon as f32 * TAU / 3.0;
                let local = [radius * angle.cos(), radius * angle.sin(), 0.0];
                strips.push(frames.iter().map(|pose| point(pose, local)).collect());
                colors.push(*color);
                radii.push(0.003);
            }
            for pose in frames.iter().step_by(4) {
                strips.push(
                    (0..=24)
                        .map(|vertex| {
                            let angle = vertex as f32 * TAU / 24.0;
                            point(pose, [radius * angle.cos(), radius * angle.sin(), 0.0])
                        })
                        .collect(),
                );
                colors.push(SECTION_COLORS[section_index]);
                radii.push(0.0015);
            }
            base *= frame(section, 1.0);
            joints.push(point(&base, [0.0; 3]));
        }

        let tip = point(&base, [0.0; 3]);
        let axes = [Vector3::x(), Vector3::y(), Vector3::z()].map(|axis| {
            let axis = base.rotation.transform_vector(&(axis * 0.12));
            [axis.x, axis.y, axis.z]
        });
        // These batches all belong to ONE entity, for both rec.log() and debug transport.
        let mut batches = LineStrips3D::new(strips)
            .with_colors(colors)
            .with_radii(radii)
            .as_serialized_batches();
        batches.extend(
            Points3D::new(joints)
                .with_radii([0.014])
                .with_colors([[205, 214, 244]])
                .as_serialized_batches(),
        );
        batches.extend(
            Arrows3D::from_vectors(axes)
                .with_origins([tip; 3])
                .with_colors(TENDON_COLORS)
                .with_radii([0.003])
                .as_serialized_batches(),
        );
        batches
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::units::si::f32::Length;
    use cu29_traits::debug_representation::{DebugRepresentation, DebugRepresentationContext};

    #[test]
    fn equal_lengths_are_straight_and_shortening_first_tendon_bends_toward_it() {
        let mut section = TendonSection::default();
        let straight = frame(&section, 1.0);
        assert!((straight.translation.vector - Vector3::new(0.0, 0.0, 0.4)).norm() < 1e-6);
        assert!(straight.rotation.angle() < 1e-6);
        let theta = 0.7;
        let r = section.routing_radius.get::<meter>();
        section.tendon_lengths = [
            0.4 - r * theta,
            0.4 + r * theta / 2.0,
            0.4 + r * theta / 2.0,
        ]
        .map(Length::new::<meter>);
        let tip = frame(&section, 1.0);
        let expected = Vector3::new(
            0.4 / theta * (1.0 - theta.cos()),
            0.0,
            0.4 / theta * theta.sin(),
        );
        assert!((tip.translation.vector - expected).norm() < 1e-6);
        assert!((tip.rotation.angle() - theta).abs() < 1e-6);
    }

    #[test]
    fn debug_attribute_encodes_the_same_components_as_local_logging() {
        use cu_rerun::rerun::external::re_log_encoding::Decoder;
        use cu_rerun::rerun::external::re_log_msg::LogMsg;
        use cu_rerun::rerun::external::re_sdk_types::View;
        use cu29::reflect::PartialReflect;

        let arm = SoftArm::at_phase(0.8);
        let attribute =
            DebugRepresentation::for_type(arm.get_represented_type_info().unwrap()).unwrap();
        assert_eq!(
            attribute.view_class,
            cu_rerun::views::Spatial3DView::identifier()
        );
        let context = DebugRepresentationContext {
            entity_path: "arm",
            culistid: 7,
            time_ns: 1_000_000_000,
        };
        let bytes = (attribute.encode)(&arm, &context).unwrap();
        let messages = Decoder::<LogMsg>::decode_lazy(bytes.as_slice())
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        let chunks = messages
            .iter()
            .filter_map(|message| match message {
                LogMsg::ArrowMsg(_, chunk) => Some(chunk),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert!(!chunks.is_empty());
        assert!(
            chunks
                .iter()
                .all(|chunk| chunk.batch.entity_path().to_string() == "/arm")
        );
        let (rec, storage) = cu_rerun::rerun::RecordingStreamBuilder::new("soft-arm-test")
            .send_properties(false)
            .memory()
            .unwrap();
        rec.log("arm", &arm).unwrap();
        let local = storage.take();
        // Compare the actual component arrays, excluding independently generated row IDs/times.
        let local_chunk = local
            .iter()
            .find_map(|message| match message {
                LogMsg::ArrowMsg(_, chunk) => Some(chunk),
                _ => None,
            })
            .unwrap();
        let remote_chunk = chunks[0];
        for batch in arm.as_serialized_batches() {
            let name = batch.descriptor.component.as_str();
            let local_column = local_chunk.batch.column_by_name(name).unwrap();
            let remote_column = remote_chunk.batch.column_by_name(name).unwrap();
            assert_eq!(local_column, remote_column);
        }
    }
}
