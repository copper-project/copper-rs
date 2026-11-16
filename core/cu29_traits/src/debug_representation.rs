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

/// One independently selectable entity. Paths include the requested root.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct RepresentationEntity {
    pub path: alloc::string::String,
    pub view_class: ViewClassIdentifier,
    #[serde(with = "serde_bytes")]
    pub bytes: Vec<u8>,
}

/// Native Rerun blueprint recording, applied only when adding a view.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct BlueprintPayload {
    #[serde(with = "serde_bytes")]
    pub bytes: Vec<u8>,
}

/// Complete subtree snapshot. An empty entity list clears the owned subtree.
/// A blueprint is a default presentation, not an instruction to reset user layout.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct CompositionSnapshot {
    pub entities: Vec<RepresentationEntity>,
    pub blueprint: Option<BlueprintPayload>,
}

/// Host adapter that combines discovered children using native viewer blueprints.
#[doc(hidden)]
pub type CompositionMerger = fn(Vec<CompositionSnapshot>) -> CuResult<CompositionSnapshot>;

/// Type-erased callback used to evaluate a composed representation.
pub type CompositionEncoder = dyn Fn(&dyn Reflect, &DebugRepresentationContext<'_>) -> CuResult<CompositionSnapshot>
    + Send
    + Sync;

/// Debug-only composite callback attached by `cu_rerun::composite`.
#[derive(Clone, Reflect)]
#[reflect(opaque)]
pub struct DebugComposite {
    pub compose: alloc::sync::Arc<CompositionEncoder>,
}

impl DebugComposite {
    /// Retrieves a composite adapter from reflected type metadata.
    pub fn for_type(info: &TypeInfo) -> Option<&Self> {
        match info {
            TypeInfo::Struct(info) => info.get_attribute::<Self>(),
            TypeInfo::TupleStruct(info) => info.get_attribute::<Self>(),
            TypeInfo::Enum(info) => info.get_attribute::<Self>(),
            _ => None,
        }
    }
}

/// Discover representations in a value, or use its explicit composite override.
/// This runs exclusively in the debug reader. Opaque values require an adapter.
#[doc(hidden)]
pub fn represent(
    value: &dyn Reflect,
    context: &DebugRepresentationContext<'_>,
    merge: CompositionMerger,
) -> CuResult<CompositionSnapshot> {
    represent_partial(value.as_partial_reflect(), context, 0, merge)
}

fn represent_partial(
    value: &dyn bevy_reflect::PartialReflect,
    context: &DebugRepresentationContext<'_>,
    depth: usize,
    merge: CompositionMerger,
) -> CuResult<CompositionSnapshot> {
    if depth > 64 {
        return Err(crate::CuError::from(
            "representation nesting exceeds 64 levels",
        ));
    }
    if let Some(info) = value.get_represented_type_info()
        && let Some(value) = value.try_as_reflect()
    {
        if let Some(composite) = DebugComposite::for_type(info) {
            return (composite.compose)(value, context);
        }
        if let Some(representation) = DebugRepresentation::for_type(info) {
            return Ok(CompositionSnapshot {
                entities: alloc::vec![RepresentationEntity {
                    path: context.entity_path.into(),
                    view_class: representation.view_class,
                    bytes: (representation.encode)(value, context)?,
                }],
                blueprint: None,
            });
        }
    }
    let mut children = Vec::new();
    let mut visit =
        |name: alloc::string::String, child: &dyn bevy_reflect::PartialReflect| -> CuResult<()> {
            let path = alloc::format!("{}/{name}", context.entity_path);
            let child_context = DebugRepresentationContext {
                entity_path: &path,
                culistid: context.culistid,
                time_ns: context.time_ns,
            };
            children.push(represent_partial(child, &child_context, depth + 1, merge)?);
            Ok(())
        };
    match value.reflect_ref() {
        bevy_reflect::ReflectRef::Struct(value) => {
            for index in 0..value.field_len() {
                if let (Some(name), Some(child)) = (value.name_at(index), value.field_at(index)) {
                    visit(name.into(), child)?;
                }
            }
        }
        bevy_reflect::ReflectRef::TupleStruct(value) => {
            for (index, child) in value.iter_fields().enumerate() {
                visit(alloc::format!("{index}"), child)?;
            }
        }
        bevy_reflect::ReflectRef::Tuple(value) => {
            for (index, child) in value.iter_fields().enumerate() {
                visit(alloc::format!("{index}"), child)?;
            }
        }
        bevy_reflect::ReflectRef::Enum(value) => {
            for index in 0..value.field_len() {
                if let Some(child) = value.field_at(index) {
                    let name = value
                        .name_at(index)
                        .map(alloc::string::String::from)
                        .unwrap_or_else(|| alloc::format!("{index}"));
                    visit(alloc::format!("{}/{name}", value.variant_name()), child)?;
                }
            }
        }
        _ => {}
    }
    merge(children)
}

/// Whether a type or one of its reflected children has an explicit adapter.
/// Recursive types are visited once; opaque storage requires an explicit adapter.
pub fn has_representation(info: &TypeInfo) -> bool {
    fn visit(info: &TypeInfo, seen: &mut Vec<core::any::TypeId>) -> bool {
        if DebugRepresentation::for_type(info).is_some() || DebugComposite::for_type(info).is_some()
        {
            return true;
        }
        if seen.contains(&info.type_id()) {
            return false;
        }
        seen.push(info.type_id());
        let mut child = |info: Option<&TypeInfo>| info.is_some_and(|info| visit(info, seen));
        match info {
            TypeInfo::Struct(info) => info.iter().any(|field| child(field.type_info())),
            TypeInfo::TupleStruct(info) => info.iter().any(|field| child(field.type_info())),
            TypeInfo::Tuple(info) => info.iter().any(|field| child(field.type_info())),
            TypeInfo::Enum(info) => info.iter().any(|variant| match variant {
                bevy_reflect::enums::VariantInfo::Struct(info) => {
                    info.iter().any(|field| child(field.type_info()))
                }
                bevy_reflect::enums::VariantInfo::Tuple(info) => {
                    info.iter().any(|field| child(field.type_info()))
                }
                _ => false,
            }),
            _ => false,
        }
    }
    visit(info, &mut Vec::new())
}
