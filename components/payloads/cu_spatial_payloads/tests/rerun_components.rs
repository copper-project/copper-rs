#![cfg(feature = "rerun")]

use cu_spatial_payloads::{GeodeticPosition, Transform3D};
use rerun::AsComponents;

#[test]
fn transform3d_f32_emits_rerun_components() {
    let transform = Transform3D::<f32>::from_matrix([
        [1.0, 0.0, 0.0, 0.1],
        [0.0, 1.0, 0.0, 0.2],
        [0.0, 0.0, 1.0, 0.3],
        [0.0, 0.0, 0.0, 1.0],
    ]);

    assert!(!transform.as_serialized_batches().is_empty());
}

#[test]
fn transform3d_f64_emits_rerun_components() {
    let transform = Transform3D::<f64>::from_matrix([
        [1.0, 0.0, 0.0, 1.1],
        [0.0, 1.0, 0.0, 1.2],
        [0.0, 0.0, 1.0, 1.3],
        [0.0, 0.0, 0.0, 1.0],
    ]);

    assert!(!transform.as_serialized_batches().is_empty());
}

#[test]
fn geodetic_position_emits_rerun_components() {
    let position = GeodeticPosition::from_degrees(59.319_221, 18.075_631);

    assert!(!position.as_serialized_batches().is_empty());
}
