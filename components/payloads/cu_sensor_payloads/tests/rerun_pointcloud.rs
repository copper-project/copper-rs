#![cfg(feature = "rerun")]
use cu_sensor_payloads::{PointCloud, PointCloudSoa, PointCloudSoaHandle};
use cu29::bevy_reflect::{Reflect, Typed};
use cu29::debug_representation::{DebugRepresentation, DebugRepresentationContext};
use rerun::AsComponents;

fn check<T: Reflect + Typed + AsComponents>(value: T) {
    let representation = DebugRepresentation::for_type(T::type_info()).expect("representation");
    assert_eq!(representation.view_class.to_string(), "3D");
    let bytes = (representation.encode)(
        &value,
        &DebugRepresentationContext {
            entity_path: "cloud",
            culistid: 42,
            time_ns: 123,
        },
    )
    .expect("encode");
    assert!(!bytes.is_empty());
    assert!(!value.as_serialized_batches().is_empty());
}

#[test]
fn cloud_scalar_soa_and_handle_have_debug_representations() {
    check(PointCloud::default());
    check(PointCloudSoa::<4>::default());
    check(PointCloudSoaHandle::<4>::default());
}

#[test]
fn handle_representation_reads_current_shared_payload() {
    let mut payload = PointCloudSoaHandle::<4>::default();
    payload.with_inner_mut(|inner| inner.push(PointCloud::default()));
    assert_eq!(payload.as_serialized_batches()[0].array.len(), 1);
    check(payload.clone());
    payload.with_inner_mut(|inner| inner.len = 100);
    assert_eq!(payload.as_serialized_batches()[0].array.len(), 4);
    check(payload);
}
