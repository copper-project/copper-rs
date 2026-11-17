//! Optional floating-point pose representations without restricting generic reflection.

use crate::Transform3D;
use alloc::vec::Vec;
use core::any::TypeId;
use cu_rerun::rerun::{Arrows3D, AsComponents, Radius, SerializedComponentBatch};
use cu29::debug_representation::DebugRepresentation;

pub(crate) fn representation<T: 'static>() -> Option<DebugRepresentation> {
    if TypeId::of::<T>() == TypeId::of::<f32>() {
        Some(cu_rerun::representation::<
            Transform3D<f32>,
            cu_rerun::views::Spatial3DView,
        >())
    } else if TypeId::of::<T>() == TypeId::of::<f64>() {
        Some(cu_rerun::representation::<
            Transform3D<f64>,
            cu_rerun::views::Spatial3DView,
        >())
    } else {
        None
    }
}

pub(crate) fn axes() -> Vec<SerializedComponentBatch> {
    Arrows3D::from_vectors([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        .with_origins([[0.0, 0.0, 0.0]; 3])
        .with_labels(["X", "Y", "Z"])
        .with_colors([[243, 139, 168], [166, 227, 161], [137, 180, 250]])
        .with_radii([Radius::new_ui_points(2.0)])
        .as_serialized_batches()
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::bevy_reflect::{Reflect, Typed};
    use cu29::debug_representation::DebugRepresentationContext;

    #[test]
    fn floating_point_poses_encode_without_restricting_other_scalars() {
        let f32_pose = Transform3D::<f32>::default();
        let f64_pose = Transform3D::<f64>::default();
        for value in [&f32_pose as &dyn Reflect, &f64_pose] {
            let representation =
                DebugRepresentation::for_type(value.get_represented_type_info().unwrap()).unwrap();
            let bytes = (representation.encode)(
                value,
                &DebugRepresentationContext {
                    entity_path: "pose",
                    culistid: 42,
                    time_ns: 1_000,
                },
            )
            .unwrap();
            assert!(!bytes.is_empty());
        }
        assert!(DebugRepresentation::for_type(Transform3D::<i32>::type_info()).is_none());
    }
}
