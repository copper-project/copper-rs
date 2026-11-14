//! Rerun representations usable both with `RecordingStream::log` and Copper's debugger.
//!
//! Implement `rerun::AsComponents` and attach
//! `#[reflect(@cu_rerun::representation::<Self, cu_rerun::views::Spatial3DView>())]`
//! to the payload or task debug-state type. Enable this crate only in host/debug builds.

use bevy_reflect::Reflect;
use cu29_traits::CuError;
use cu29_traits::CuResult;
use cu29_traits::debug_representation::{DebugRepresentation, DebugRepresentationContext};
pub use rerun;
use rerun::external::re_sdk_types::View;
pub use rerun::external::re_sdk_types::blueprint::views;

/// Versioned wire format understood by the Timetraveler Rerun receiver.
pub const FORMAT: &str = "rerun.rrd/0.43";

/// Attaches a standard `AsComponents` implementation to Copper reflection.
pub fn representation<T: Reflect + rerun::AsComponents, V: View>() -> DebugRepresentation {
    DebugRepresentation {
        format: FORMAT,
        view_class: V::identifier().as_str(),
        encode: encode::<T>,
    }
}

fn encode<T: Reflect + rerun::AsComponents>(
    value: &dyn Reflect,
    context: &DebugRepresentationContext<'_>,
) -> CuResult<Vec<u8>> {
    let value = value
        .downcast_ref::<T>()
        .ok_or_else(|| CuError::from("representation type mismatch"))?;
    let rec = rerun::RecordingStreamBuilder::new("copper-debug-representation")
        .send_properties(false)
        .buffered()
        .map_err(|e| CuError::new_with_cause("create Rerun recording", e))?;
    let storage = rec.binary_stream();
    let cl = i64::try_from(context.culistid)
        .map_err(|_| CuError::from("CopperList ID exceeds Rerun timeline range"))?;
    let time = i64::try_from(context.time_ns)
        .map_err(|_| CuError::from("time exceeds Rerun timeline range"))?;
    rec.set_time_sequence("culistid", cl);
    rec.set_time("cu_time", std::time::Duration::from_nanos(time as u64));
    rec.log(context.entity_path, value)
        .map_err(|e| CuError::new_with_cause("serialize Rerun representation", e))?;
    storage
        .flush(std::time::Duration::from_secs(10))
        .map_err(|e| CuError::new_with_cause("flush Rerun representation", e))?;
    storage
        .read()
        .ok_or_else(|| CuError::from("Rerun representation encoder produced no data"))
}
