//! Rerun representations usable both with `RecordingStream::log` and Copper's debugger.
//!
//! Implement [`rerun::AsComponents`] on your payload or task debug-state type to
//! describe its Rerun data. Then choose a view for Copper's debugger by attaching
//! a reflection attribute. For example, for 3D data, use
//! `#[reflect(@cu_rerun::representation::<Self, cu_rerun::views::Spatial3DView>())]`
//! to your type.
//!
//! `Spatial3DView` is just an example: [`representation`] accepts any Rerun view
//! type. Replace it with a type from [`views`] that matches your components:
//!
//! - [`views::Spatial3DView`]: 3D points, meshes, and poses.
//! - [`views::Spatial2DView`]: images and 2D geometry.
//! - [`views::TimeSeriesView`]: scalar values over time.
//! - [`views::BarChartView`]: bar charts.
//! - [`views::TextLogView`]: text log entries.
//! - [`views::TextDocumentView`]: text or Markdown documents.
//! - [`views::TensorView`]: multidimensional arrays.
//! - [`views::MapView`]: geographic data.
//! - [`views::GraphView`]: nodes and edges.
//! - [`views::DataframeView`]: tabular inspection.
//!
//! See Rerun's [view guide](https://rerun.io/docs/reference/types/views) for
//! examples and the archetypes each view can display, and [`views`] for the Rust
//! API available in this crate's Rerun version. The debugger must support the
//! chosen view; the attribute selects its view class, not its layout or settings.
//!
//! The same `AsComponents` implementation works with `RecordingStream::log`;
//! the reflection attribute is only needed for Copper's debugger.
//! Enable this crate only in host/debug builds.

use bevy_reflect::Reflect;
use cu29_traits::CuError;
use cu29_traits::CuResult;
use cu29_traits::debug_representation::{DebugRepresentation, DebugRepresentationContext};
pub use rerun;
use rerun::external::re_sdk_types::View;
pub use rerun::external::re_sdk_types::blueprint::views;

/// Attaches a standard `AsComponents` implementation to Copper reflection.
pub fn representation<T: Reflect + rerun::AsComponents, V: View>() -> DebugRepresentation {
    DebugRepresentation {
        view_class: V::identifier(),
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
    encode_components(value, context)
}

fn encode_components(
    value: &dyn rerun::AsComponents,
    context: &DebugRepresentationContext<'_>,
) -> CuResult<Vec<u8>> {
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

mod composition;
pub use composition::{
    Composition, composite, container, entity, field, horizontal, overlay, represent, vertical,
};
