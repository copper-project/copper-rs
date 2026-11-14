//! Optional host-side representations attached to reflected types.

use crate::CuResult;
use alloc::vec::Vec;
use bevy_reflect::{Reflect, TypeInfo};

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
/// Integration crates construct this attribute; the runtime transports its opaque bytes
/// without depending on the visualization backend. Encoding is never called by task execution.
#[derive(Clone, Reflect)]
#[reflect(opaque)]
pub struct DebugRepresentation {
    /// Versioned backend wire format identifier.
    pub format: &'static str,
    /// Backend-owned view class identifier; Copper does not interpret it.
    pub view_class: &'static str,
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
