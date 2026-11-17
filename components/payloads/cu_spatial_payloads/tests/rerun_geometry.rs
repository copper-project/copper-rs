#![cfg(feature = "rerun")]

use cu_spatial_payloads::*;
use cu29::bevy_reflect::{Reflect, Typed};
use cu29::debug_representation::{DebugRepresentation, DebugRepresentationContext};
use rerun::AsComponents;

fn check<T: Reflect + Typed + AsComponents>(value: T, view: &str) {
    let representation = DebugRepresentation::for_type(T::type_info()).expect("representation");
    assert_eq!(representation.view_class.to_string(), view);
    let bytes = (representation.encode)(
        &value,
        &DebugRepresentationContext {
            entity_path: "geometry",
            culistid: 42,
            time_ns: 123,
        },
    )
    .expect("encode");
    assert!(!bytes.is_empty());
    assert!(!value.as_serialized_batches().is_empty());
}

#[test]
fn standard_geometry_has_debug_representations() {
    check(Point2f::from_meters(1.0, 2.0), "2D");
    check(Point2d::from_meters(1.0, 2.0), "2D");
    check(Point2i::new(-1, 2), "2D");
    check(Point2u::new(1, 2), "2D");
    check(Point3f::from_meters(1.0, 2.0, 3.0), "3D");
    check(Point3d::from_meters(1.0, 2.0, 3.0), "3D");
    check(BBox2f::default(), "2D");
    check(BBox2d::default(), "2D");
    check(BBox2i::default(), "2D");
    check(BBox2u::default(), "2D");
    check(BBox3f::default(), "3D");
    check(BBox3d::default(), "3D");
    check(Point2fSoa::<2>::default(), "2D");
    check(Point2dSoa::<2>::default(), "2D");
    check(Point2iSoa::<2>::default(), "2D");
    check(Point2uSoa::<2>::default(), "2D");
    check(Point3fSoa::<2>::default(), "3D");
    check(Point3dSoa::<2>::default(), "3D");
}

#[test]
fn other_coordinate_types_remain_reflectable_without_a_representation() {
    assert!(DebugRepresentation::for_type(Point2::<f32>::type_info()).is_none());
    assert!(DebugRepresentation::for_type(Point3::<f64>::type_info()).is_none());
    assert!(DebugRepresentation::for_type(BBox::<Point2<f32>>::type_info()).is_none());
}

#[test]
fn oversized_soa_length_is_clamped() {
    let points = Point2iSoa::<2> {
        len: 3,
        x: [1, 2],
        y: [3, 4],
    };
    assert_eq!(points.as_serialized_batches()[0].array.len(), 2);
    check(points, "2D");
}

#[test]
fn double_precision_box_extents_survive_large_origins() {
    let bounds = BBox3d::new(
        Point3d::from_meters(1.0e12, 1.0e12, 1.0e12),
        Point3d::from_meters(1.0e12 + 1.0, 1.0e12 + 2.0, 1.0e12 + 3.0),
    );
    let expected =
        rerun::Boxes3D::from_centers_and_half_sizes([[1.0e12_f32; 3]], [[0.5, 1.0, 1.5]]);
    assert_eq!(
        bounds.as_serialized_batches(),
        expected.as_serialized_batches()
    );
}
