//! A Copper payload whose tendon measurements need custom geometry to be useful.

use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::f32::Length;
use cu29::units::si::length::meter;
use serde::{Deserialize, Serialize};

#[cfg(feature = "rerun")]
mod representation;

#[cfg(feature = "app")]
pub mod tasks;

/// Three tendons routed at 0°, 120°, and 240° around one flexible section.
/// Lengths are measured within this section, not from the base of the whole arm.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct TendonSection {
    pub tendon_lengths: [Length; 3],
    pub routing_radius: Length,
}

impl Default for TendonSection {
    fn default() -> Self {
        Self {
            tendon_lengths: [Length::new::<meter>(0.4); 3],
            routing_radius: Length::new::<meter>(0.055),
        }
    }
}

/// Three flexible sections. No poses, points, or rendering types are stored here.
///
/// The optional attribute makes the same `AsComponents` implementation discoverable
/// by a Copper replay server built with `cu29/rerun`.
#[derive(Debug, Clone, Default, Serialize, Deserialize, Encode, Decode, Reflect)]
#[cfg_attr(feature = "rerun", reflect(@cu_rerun::representation::<Self, cu_rerun::views::Spatial3DView>()))]
pub struct SoftArm {
    pub sections: [TendonSection; 3],
}

impl SoftArm {
    /// Synthetic tendon motion for one periodic gesture. Phase is in radians.
    pub fn at_phase(phase: f32) -> Self {
        let mut arm = Self::default();
        for (section_index, section) in arm.sections.iter_mut().enumerate() {
            let offset = section_index as f32 * 2.1;
            let bend_x = 0.8 * (phase + offset).sin();
            let bend_y = 0.6 * (2.0 * phase + offset).cos();
            let radius = section.routing_radius.get::<meter>();
            section.tendon_lengths = core::array::from_fn(|tendon| {
                let angle = tendon as f32 * core::f32::consts::TAU / 3.0;
                Length::new::<meter>(0.4 - radius * (bend_x * angle.cos() + bend_y * angle.sin()))
            });
        }
        arm
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn domain_type_is_a_copper_payload_without_visualization_fields() {
        fn accepts_payload<T: CuMsgPayload>() {}
        accepts_payload::<SoftArm>();
        let arm = SoftArm::at_phase(0.7);
        let bytes = bincode::encode_to_vec(&arm, bincode::config::standard()).unwrap();
        let (restored, _): (SoftArm, _) =
            bincode::decode_from_slice(&bytes, bincode::config::standard()).unwrap();
        assert_eq!(
            restored.sections[1].tendon_lengths,
            arm.sections[1].tendon_lengths
        );
    }
}
