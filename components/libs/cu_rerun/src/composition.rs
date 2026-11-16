//! Named representations compiled into native Rerun blueprint archetypes.

use bevy_reflect::Reflect;
use cu29_traits::debug_representation::{
    BlueprintPayload, CompositionSnapshot, DebugComposite, DebugRepresentationContext,
    RepresentationEntity, ViewClassIdentifier,
};
use cu29_traits::{CuError, CuResult};
use rerun::external::re_sdk_types::blueprint::archetypes::{
    ContainerBlueprint, ViewBlueprint, ViewContents, ViewportBlueprint,
};
use rerun::external::re_sdk_types::blueprint::components::{ContainerKind, RootContainer};
use rerun::external::re_sdk_types::encodings::Uuid;
use std::collections::HashSet;
use std::sync::Arc;

/// Borrowed composition evaluated only for explicit debug requests.
pub enum Composition<'a> {
    Field(&'a str, &'a dyn Reflect),
    Entity(
        &'a str,
        ViewClassIdentifier,
        Box<dyn rerun::AsComponents + 'a>,
    ),
    Container(ContainerKind, Vec<Composition<'a>>),
    /// Several entities selected by one native Rerun view.
    Overlay(Vec<Composition<'a>>),
}

/// Reuse a reflected field's representation under its stable field name.
pub fn field<'a>(name: &'a str, value: &'a dyn Reflect) -> Composition<'a> {
    Composition::Field(name, value)
}

/// Emit computed data with an explicit native Rerun view class.
pub fn entity<'a, V: rerun::external::re_sdk_types::View>(
    name: &'a str,
    value: impl rerun::AsComponents + 'a,
) -> Composition<'a> {
    Composition::Entity(name, V::identifier(), Box::new(value))
}

/// Arrange children with any native Rerun container kind, including tabs and grids.
pub fn container<'a>(
    kind: ContainerKind,
    children: impl IntoIterator<Item = Composition<'a>>,
) -> Composition<'a> {
    Composition::Container(kind, children.into_iter().collect())
}

/// Place children side by side using a native Rerun horizontal container.
pub fn horizontal<'a>(children: impl IntoIterator<Item = Composition<'a>>) -> Composition<'a> {
    container(ContainerKind::Horizontal, children)
}

/// Stack children using a native Rerun vertical container.
pub fn vertical<'a>(children: impl IntoIterator<Item = Composition<'a>>) -> Composition<'a> {
    container(ContainerKind::Vertical, children)
}

/// Select compatible entities in one native view sharing the parent's coordinates.
pub fn overlay<'a>(children: impl IntoIterator<Item = Composition<'a>>) -> Composition<'a> {
    Composition::Overlay(children.into_iter().collect())
}

/// Attach a composition with `#[reflect(@cu_rerun::composite(Self::compose))]`.
pub fn composite<T: Reflect>(compose: for<'a> fn(&'a T) -> Composition<'a>) -> DebugComposite {
    DebugComposite {
        compose: Arc::new(move |value, context| {
            let value = value
                .downcast_ref::<T>()
                .ok_or_else(|| CuError::from("composition type mismatch"))?;
            snapshot(|rec, entities| {
                encode(
                    compose(value),
                    context,
                    Some(rec),
                    entities,
                    &mut HashSet::new(),
                )
            })
        }),
    }
}

/// Discover nested represented values and retain native composite blueprints.
pub fn represent(
    value: &dyn Reflect,
    context: &DebugRepresentationContext<'_>,
) -> CuResult<CompositionSnapshot> {
    cu29_traits::debug_representation::represent(value, context, merge)
}

fn merge(children: Vec<CompositionSnapshot>) -> CuResult<CompositionSnapshot> {
    if children.is_empty() {
        return Ok(CompositionSnapshot::default());
    }
    snapshot(|rec, entities| {
        let paths = children
            .into_iter()
            .map(|child| import(child, rec, entities))
            .collect::<CuResult<Vec<_>>>()?
            .into_iter()
            .flatten()
            .collect();
        log_container(rec, ContainerKind::Vertical, paths)
    })
}

fn snapshot(
    build: impl FnOnce(
        &rerun::RecordingStream,
        &mut Vec<RepresentationEntity>,
    ) -> CuResult<Option<String>>,
) -> CuResult<CompositionSnapshot> {
    let rec = rerun::RecordingStreamBuilder::new("copper-debug-representation")
        .blueprint()
        .send_properties(false)
        .buffered()
        .map_err(|e| CuError::new_with_cause("create blueprint recording", e))?;
    let storage = rec.binary_stream();
    rec.set_time_sequence("blueprint", 0);
    let mut entities = Vec::new();
    let root = build(&rec, &mut entities)?;
    let Some(root) = root else {
        return Ok(CompositionSnapshot {
            entities,
            blueprint: None,
        });
    };
    // The native viewport always references a container, including single views.
    let root = log_container(&rec, ContainerKind::Tabs, vec![root])?
        .ok_or_else(|| CuError::from("missing blueprint root"))?;
    let id = root
        .trim_start_matches("container/")
        .parse::<rerun::external::uuid::Uuid>()
        .map_err(|e| CuError::new_with_cause("invalid blueprint root", e))?;
    rec.log(
        "viewport",
        &ViewportBlueprint::new()
            .with_root_container(RootContainer(id.into()))
            .with_auto_layout(false)
            .with_auto_views(false),
    )
    .map_err(|e| CuError::new_with_cause("serialize blueprint viewport", e))?;
    storage
        .flush(std::time::Duration::from_secs(10))
        .map_err(|e| CuError::new_with_cause("flush blueprint", e))?;
    let bytes = storage
        .read()
        .ok_or_else(|| CuError::from("blueprint encoder produced no data"))?;
    Ok(CompositionSnapshot {
        entities,
        blueprint: Some(BlueprintPayload { bytes }),
    })
}

fn log_container(
    rec: &rerun::RecordingStream,
    kind: ContainerKind,
    children: Vec<String>,
) -> CuResult<Option<String>> {
    if children.is_empty() {
        return Ok(None);
    }
    let path = format!("container/{}", Uuid::random());
    rec.log(
        path.clone(),
        &ContainerBlueprint::new(kind).with_contents(children),
    )
    .map_err(|e| CuError::new_with_cause("serialize blueprint container", e))?;
    Ok(Some(path))
}

fn log_view(
    rec: &rerun::RecordingStream,
    class: ViewClassIdentifier,
    origin: &str,
    paths: Vec<String>,
) -> CuResult<String> {
    let path = format!("view/{}", Uuid::random());
    rec.log(
        path.clone(),
        &ViewBlueprint::new(class.as_str()).with_space_origin(origin),
    )
    .map_err(|e| CuError::new_with_cause("serialize blueprint view", e))?;
    rec.log(
        format!("{path}/ViewContents"),
        &ViewContents::new(
            paths
                .into_iter()
                .map(|path| format!("/{}", path.trim_start_matches('/'))),
        ),
    )
    .map_err(|e| CuError::new_with_cause("serialize blueprint contents", e))?;
    Ok(path)
}

fn child_context<'a>(
    name: &str,
    context: &DebugRepresentationContext<'_>,
    names: &mut HashSet<String>,
    path: &'a mut String,
) -> CuResult<DebugRepresentationContext<'a>> {
    if name.is_empty()
        || name.contains('/')
        || name == "."
        || name == ".."
        || !names.insert(name.to_owned())
    {
        return Err(CuError::from(
            "composition child names must be unique nonempty path segments",
        ));
    }
    *path = format!("{}/{name}", context.entity_path);
    Ok(DebugRepresentationContext {
        entity_path: path,
        culistid: context.culistid,
        time_ns: context.time_ns,
    })
}

fn encode(
    composition: Composition<'_>,
    context: &DebugRepresentationContext<'_>,
    rec: Option<&rerun::RecordingStream>,
    entities: &mut Vec<RepresentationEntity>,
    names: &mut HashSet<String>,
) -> CuResult<Option<String>> {
    match composition {
        Composition::Field(name, value) => {
            let mut path = String::new();
            let context = child_context(name, context, names, &mut path)?;
            let snapshot = represent(value, &context)?;
            if let Some(rec) = rec {
                import(snapshot, rec, entities)
            } else {
                entities.extend(snapshot.entities);
                Ok(None)
            }
        }
        Composition::Entity(name, class, value) => {
            let mut path = String::new();
            let context = child_context(name, context, names, &mut path)?;
            let bytes = super::encode_components(value.as_ref(), &context)?;
            let view = rec
                .map(|rec| log_view(rec, class, &path, vec![path.clone()]))
                .transpose()?;
            entities.push(RepresentationEntity {
                path,
                view_class: class,
                bytes,
            });
            Ok(view)
        }
        Composition::Container(kind, children) => {
            let paths = children
                .into_iter()
                .map(|child| encode(child, context, rec, entities, names))
                .collect::<CuResult<Vec<_>>>()?
                .into_iter()
                .flatten()
                .collect();
            match rec {
                Some(rec) => log_container(rec, kind, paths),
                None => Ok(None),
            }
        }
        Composition::Overlay(children) => {
            let start = entities.len();
            for child in children {
                encode(child, context, None, entities, names)?;
            }
            let added = &entities[start..];
            let Some(first) = added.first() else {
                return Ok(None);
            };
            if added
                .iter()
                .any(|entity| entity.view_class != first.view_class)
            {
                return Err(CuError::from(
                    "overlay entities must use the same view class",
                ));
            }
            let Some(rec) = rec else {
                return Ok(None);
            };
            Ok(Some(log_view(
                rec,
                first.view_class,
                context.entity_path,
                added.iter().map(|entity| entity.path.clone()).collect(),
            )?))
        }
    }
}

/// Import native blueprint components without importing a child's viewport or activation.
fn import(
    snapshot: CompositionSnapshot,
    rec: &rerun::RecordingStream,
    entities: &mut Vec<RepresentationEntity>,
) -> CuResult<Option<String>> {
    use rerun::FromArrow;
    use rerun::external::arrow::array::{Array, ListArray};
    use rerun::external::re_log_encoding::Decoder;
    use rerun::external::re_log_msg::LogMsg;
    let root = if let Some(blueprint) = snapshot.blueprint {
        let mut root = None;
        let mut decoder = Decoder::<LogMsg>::decode_lazy(blueprint.bytes.as_slice());
        for message in decoder.by_ref() {
            let message =
                message.map_err(|e| CuError::new_with_cause("decode nested blueprint", e))?;
            if let LogMsg::ArrowMsg(_, chunk) = message {
                let path = chunk.batch.entity_path().to_string();
                for (descriptor, array) in chunk.batch.component_columns() {
                    let array = array
                        .as_any()
                        .downcast_ref::<ListArray>()
                        .ok_or_else(|| CuError::from("invalid blueprint component column"))?;
                    for row in 0..array.len() {
                        if array.is_null(row) {
                            continue;
                        }
                        let values = array.value(row);
                        if path.trim_start_matches('/') == "viewport" {
                            if descriptor.component_type.as_ref().is_some_and(|name| {
                                name.as_str() == "rerun.blueprint.components.RootContainer"
                            }) {
                                let roots =
                                    RootContainer::from_arrow(values.as_ref()).map_err(|e| {
                                        CuError::new_with_cause("decode blueprint root", e)
                                    })?;
                                if let Some(id) = roots.first() {
                                    root = Some(format!("container/{}", id.0));
                                }
                            }
                        } else {
                            rec.log_serialized_batches(
                                path.clone(),
                                false,
                                [rerun::SerializedComponentBatch::new(
                                    values,
                                    descriptor.component_descriptor(),
                                )],
                            )
                            .map_err(|e| CuError::new_with_cause("import nested blueprint", e))?;
                        }
                    }
                }
            }
        }
        if decoder.num_bytes_processed() != blueprint.bytes.len() as u64 || root.is_none() {
            return Err(CuError::from("incomplete nested blueprint"));
        }
        root
    } else {
        let views = snapshot
            .entities
            .iter()
            .map(|entity| {
                log_view(
                    rec,
                    entity.view_class,
                    &entity.path,
                    vec![entity.path.clone()],
                )
            })
            .collect::<CuResult<Vec<_>>>()?;
        log_container(rec, ContainerKind::Vertical, views)?
    };
    entities.extend(snapshot.entities);
    Ok(root)
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy_reflect::PartialReflect;
    use cu29_traits::debug_representation::has_representation;

    #[derive(Reflect)]
    #[reflect(@crate::representation::<Self, crate::views::Spatial3DView>())]
    struct Pose;

    impl rerun::AsComponents for Pose {
        fn as_serialized_batches(&self) -> Vec<rerun::SerializedComponentBatch> {
            rerun::Points3D::new([[1.0, 2.0, 3.0]]).as_serialized_batches()
        }
    }

    #[derive(Reflect)]
    #[reflect(@composite(Self::compose))]
    struct Pair {
        left: Pose,
        right: Option<Pose>,
    }

    impl Pair {
        fn compose(&self) -> Composition<'_> {
            horizontal([field("left", &self.left), field("right", &self.right)])
        }
    }

    fn snapshot(value: &dyn Reflect) -> CompositionSnapshot {
        represent(
            value,
            &DebugRepresentationContext {
                entity_path: "logged/test",
                culistid: 42,
                time_ns: 100,
            },
        )
        .unwrap()
    }

    fn paths(snapshot: &CompositionSnapshot) -> Vec<String> {
        snapshot
            .entities
            .iter()
            .map(|entity| {
                assert!(!entity.bytes.is_empty());
                entity.path.clone()
            })
            .collect()
    }

    fn composed(
        composition: Composition<'_>,
        context: &DebugRepresentationContext<'_>,
    ) -> CuResult<CompositionSnapshot> {
        super::snapshot(|rec, entities| {
            encode(
                composition,
                context,
                Some(rec),
                entities,
                &mut Default::default(),
            )
        })
    }

    fn blueprint_components<T: rerun::external::re_types_core::Component + rerun::FromArrow>(
        snapshot: &CompositionSnapshot,
    ) -> Vec<T> {
        use rerun::external::arrow::array::{Array, ListArray};
        use rerun::external::re_log_encoding::Decoder;
        use rerun::external::re_log_msg::LogMsg;
        let blueprint = snapshot.blueprint.as_ref().expect("native blueprint");
        let mut result = Vec::new();
        for message in Decoder::<LogMsg>::decode_lazy(blueprint.bytes.as_slice()) {
            match message.unwrap() {
                LogMsg::SetStoreInfo(info) => {
                    assert_eq!(info.info.store_id.kind(), rerun::StoreKind::Blueprint)
                }
                LogMsg::ArrowMsg(_, chunk) => {
                    for (descriptor, array) in chunk.batch.component_columns() {
                        if descriptor.component_type.as_ref() == Some(&T::name()) {
                            let array = array.as_any().downcast_ref::<ListArray>().unwrap();
                            for row in 0..array.len() {
                                if !array.is_null(row) {
                                    result
                                        .extend(T::from_arrow(array.value(row).as_ref()).unwrap());
                                }
                            }
                        }
                    }
                }
                LogMsg::BlueprintActivationCommand(_) => {
                    panic!("snapshot must not activate a blueprint")
                }
            }
        }
        result
    }

    #[test]
    fn composite_preserves_layout_identity_and_absence() {
        let mut pair = Pair {
            left: Pose,
            right: Some(Pose),
        };
        assert!(has_representation(
            pair.get_represented_type_info().unwrap()
        ));
        let first = snapshot(&pair);
        assert!(blueprint_components::<ContainerKind>(&first).contains(&ContainerKind::Horizontal));
        assert_eq!(
            paths(&first),
            ["logged/test/left", "logged/test/right/Some/0"]
        );
        pair.right = None;
        assert_eq!(paths(&snapshot(&pair)), ["logged/test/left"]);
        pair.right = Some(Pose);
        assert_eq!(paths(&snapshot(&pair)), paths(&first));
        assert_eq!(paths(&snapshot(&pair.left)), ["logged/test"]);
    }

    #[derive(Reflect)]
    struct Wrapper {
        pose: Pose,
        confidence: f32,
    }

    #[test]
    fn discovers_represented_child_inside_unannotated_wrapper() {
        let value = Wrapper {
            pose: Pose,
            confidence: 0.9,
        };
        assert!(has_representation(
            value.get_represented_type_info().unwrap()
        ));
        assert_eq!(paths(&snapshot(&value)), ["logged/test/pose"]);
    }

    #[test]
    fn rejects_duplicate_names_even_across_layout_groups() {
        let pose = Pose;
        let composition = horizontal([field("pose", &pose), vertical([field("pose", &pose)])]);
        let context = DebugRepresentationContext {
            entity_path: "test",
            culistid: 1,
            time_ns: 1,
        };
        assert!(composed(composition, &context).is_err());
    }

    #[test]
    fn computed_entity_carries_view_and_data() {
        let composition =
            entity::<crate::views::Spatial3DView>("points", rerun::Points3D::new([[0., 0., 0.]]));
        let context = DebugRepresentationContext {
            entity_path: "test",
            culistid: 1,
            time_ns: 1,
        };
        assert_eq!(
            paths(&composed(composition, &context).unwrap()),
            ["test/points"]
        );
    }
    #[test]
    fn nested_composites_keep_native_layout_in_one_blueprint() {
        #[derive(Reflect)]
        struct Outer {
            pair: Pair,
        }
        let outer = Outer {
            pair: Pair {
                left: Pose,
                right: Some(Pose),
            },
        };
        let result = snapshot(&outer);
        let kinds = blueprint_components::<ContainerKind>(&result);
        assert!(kinds.contains(&ContainerKind::Horizontal));
        assert!(kinds.contains(&ContainerKind::Vertical));
        assert_eq!(blueprint_components::<RootContainer>(&result).len(), 1);
        assert_eq!(
            paths(&result),
            ["logged/test/pair/left", "logged/test/pair/right/Some/0"]
        );
    }

    #[test]
    fn overlay_is_one_native_view_selecting_both_entities() {
        use rerun::external::re_sdk_types::blueprint::components::{QueryExpression, ViewClass};
        let pose = Pose;
        let context = DebugRepresentationContext {
            entity_path: "root",
            culistid: 1,
            time_ns: 1,
        };
        let result = composed(
            overlay([field("left", &pose), field("right", &pose)]),
            &context,
        )
        .unwrap();
        assert_eq!(blueprint_components::<ViewClass>(&result).len(), 1);
        assert_eq!(blueprint_components::<QueryExpression>(&result).len(), 2);
        assert_eq!(paths(&result), ["root/left", "root/right"]);
    }

    #[test]
    fn native_container_kinds_are_not_restricted_to_helpers() {
        let pose = Pose;
        let context = DebugRepresentationContext {
            entity_path: "root",
            culistid: 1,
            time_ns: 1,
        };
        let result = composed(
            container(ContainerKind::Grid, [field("pose", &pose)]),
            &context,
        )
        .unwrap();
        assert!(blueprint_components::<ContainerKind>(&result).contains(&ContainerKind::Grid));
    }

    #[test]
    fn overlay_rejects_incompatible_views() {
        let context = DebugRepresentationContext {
            entity_path: "root",
            culistid: 1,
            time_ns: 1,
        };
        let result = composed(
            overlay([
                entity::<crate::views::Spatial3DView>(
                    "points",
                    rerun::Points3D::new([[0., 0., 0.]]),
                ),
                entity::<crate::views::TimeSeriesView>("scalar", rerun::Scalars::new([1.])),
            ]),
            &context,
        );
        assert!(result.is_err());
    }
}
