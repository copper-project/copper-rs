//! Optional Rerun representations attached to reflected types (`rerun` feature).

use crate::CuResult;
use alloc::vec::Vec;
use bevy_reflect::{Reflect, TypeInfo};
pub use re_types_core::ViewClassIdentifier;

/// Placement and sample time supplied by the debugger to a representation encoder.
pub struct DebugRepresentationContext<'a> {
    /// Destination entity assigned by the viewer.
    pub entity_path: &'a str,
    /// CopperList sequence number used for the representation timeline.
    pub culistid: u64,
    /// Copper clock time in nanoseconds.
    pub time_ns: u64,
}

/// Optional reflection attribute containing a debug-only serialization callback.
///
/// The optional `cu-rerun` adapter constructs this attribute. Encoding produces RRD bytes
/// on explicit debug requests and is never called by task execution.
#[derive(Clone, Reflect)]
#[reflect(opaque)]
pub struct DebugRepresentation {
    /// Rerun view class suggested for these components.
    pub view_class: ViewClassIdentifier,
    /// Serializes a concrete reflected value outside task execution.
    pub encode: fn(&dyn Reflect, &DebugRepresentationContext<'_>) -> CuResult<Vec<u8>>,
}

impl DebugRepresentation {
    /// Looks up an explicit representation on a user-defined reflected type.
    pub fn for_type(info: &TypeInfo) -> Option<&Self> {
        match info {
            TypeInfo::Struct(info) => info.get_attribute::<Self>(),
            TypeInfo::TupleStruct(info) => info.get_attribute::<Self>(),
            TypeInfo::Enum(info) => info.get_attribute::<Self>(),
            _ => None,
        }
    }
}
