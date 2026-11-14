//! On-demand serialization of explicit payload and task-state representations.

use super::*;
use cu29_traits::debug_representation::{
    DebugRepresentation, DebugRepresentationContext, ViewClassIdentifier,
};

/// Typed value whose explicit representation is requested.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RepresentationTarget {
    Message { output_index: usize },
    TaskState { task_id: String },
}

impl RepresentationTarget {
    /// Entity namespace shared by debug producers and viewers.
    pub fn entity_path(&self, root: &str) -> String {
        match self {
            Self::Message { output_index } => {
                format!("{root}/representations/messages/{output_index}")
            }
            Self::TaskState { task_id } => format!("{root}/representations/tasks/{task_id}"),
        }
    }
}

/// Representation captured before the replay cursor advances to the next frame.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RepresentationFrame {
    pub target: RepresentationTarget,
    pub culistid: u64,
    pub payload: Option<RrdPayload>,
}

/// An explicitly registered representation advertised by the debug server.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RepresentationDescriptor {
    pub target: RepresentationTarget,
    pub task_id: String,
    pub type_path: String,
    pub view_class: ViewClassIdentifier,
}

/// Rerun recording data carried as a CBOR byte string.
/// RRD headers carry the Rerun format version.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RrdPayload {
    #[serde(with = "serde_bytes")]
    pub bytes: Vec<u8>,
}

#[derive(Deserialize)]
struct ReadParams {
    target: RepresentationTarget,
    #[serde(default)]
    at: Option<Target>,
    #[serde(default)]
    replayed: bool,
    entity_path: String,
}

impl<App, P, CB, TF, S, L, AF> RemoteDebugZenohServer<App, P, CB, TF, S, L, AF>
where
    App: CuSimApplication<S, L> + ReflectTaskIntrospection + CurrentRuntimeCopperList<P>,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple + 'static,
    CB: for<'a> Fn(
            &'a crate::copperlist::CopperList<P>,
            RobotClock,
            RobotClockMock,
        )
            -> Box<dyn for<'z> FnMut(App::Step<'z>) -> crate::simulation::SimOverride + 'a>
        + Clone,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
    AF: Fn(&SessionOpenParams) -> CuResult<(App, RobotClock, RobotClockMock)>,
{
    pub(super) fn handle_representation_list(
        &mut self,
        request: &DebugRpcRequest,
    ) -> DebugRpcResponse {
        if let Err(e) = self.session_mut(request.session_id.as_deref()) {
            return err_response(
                request.request_id.clone(),
                "SessionNotFound",
                &e.to_string(),
            );
        }
        let mut registry = TypeRegistry::default();
        populate_debug_type_registry::<App>(&mut registry);
        let mut entries = Vec::new();
        for (output_index, spec) in P::get_output_specs().iter().enumerate() {
            append_descriptor(
                &registry,
                spec.payload_type_path(),
                spec.task_id,
                RepresentationTarget::Message { output_index },
                &mut entries,
            );
        }
        for task_id in P::get_all_task_ids() {
            if let Some(type_path) = App::debug_state_type_path(task_id) {
                append_descriptor(
                    &registry,
                    type_path,
                    task_id,
                    RepresentationTarget::TaskState {
                        task_id: task_id.to_string(),
                    },
                    &mut entries,
                );
            }
        }
        ok_response(
            request.request_id.clone(),
            json!({"representations": entries}),
            None,
            None,
        )
    }

    pub(super) fn handle_representation_read(
        &mut self,
        request: &DebugRpcRequest,
    ) -> DebugRpcResponse {
        let parsed: ReadParams = match from_params(&request.params) {
            Ok(parsed) => parsed,
            Err(e) => return param_err_response(request.request_id.clone(), e),
        };
        if parsed.replayed && parsed.at.is_some() {
            return err_response(
                request.request_id.clone(),
                "InvalidParams",
                "replayed representations require the current runtime state; omit at",
            );
        }
        let time_of = self.time_of.clone();
        let state = match self.session_mut(request.session_id.as_deref()) {
            Ok(state) => state,
            Err(e) => {
                return err_response(
                    request.request_id.clone(),
                    "SessionNotFound",
                    &e.to_string(),
                );
            }
        };
        let result = (|| -> CuResult<(Option<RrdPayload>, ResolvedAt)> {
            let resolved = if let Some(target) = &parsed.at {
                resolve_target(&mut state.session, target, ResolveMode::Exact, &time_of)?
            } else {
                let cursor = cursor_snapshot(state, &time_of)?;
                ResolvedAt {
                    cl: cursor.cl,
                    ts_ns: cursor.ts_ns,
                    idx: cursor.idx,
                }
            };
            // Task state must be reconstructed, while recorded messages can be read directly.
            // This explicit seek mutates the session; callers must invalidate cached remote state.
            if parsed.at.is_some()
                && matches!(parsed.target, RepresentationTarget::TaskState { .. })
            {
                let jump = seek_to_index(&mut state.session, resolved.idx)?;
                update_after_jump(state, &jump);
            }
            let context = DebugRepresentationContext {
                entity_path: &parsed.entity_path,
                culistid: resolved.cl,
                time_ns: resolved.ts_ns.unwrap_or(0),
            };
            let payload = encode_target(
                state,
                &parsed.target,
                parsed.replayed,
                resolved.idx,
                &context,
            )?;
            Ok((payload, resolved))
        })();
        match result {
            Ok((payload, resolved)) => {
                let mut response = ok_response(
                    request.request_id.clone(),
                    json!({"present": payload.is_some()}),
                    Some(state.cursor_rev),
                    Some(resolved),
                );
                response.representation = payload;
                response
            }
            Err(e) => err_response(
                request.request_id.clone(),
                "RepresentationFailed",
                &e.to_string(),
            ),
        }
    }
}

fn append_descriptor(
    registry: &TypeRegistry,
    type_path: &str,
    task_id: &str,
    target: RepresentationTarget,
    entries: &mut Vec<RepresentationDescriptor>,
) {
    if let Some(representation) = registry
        .get_with_type_path(type_path)
        .and_then(|registration| DebugRepresentation::for_type(registration.type_info()))
    {
        entries.push(RepresentationDescriptor {
            target,
            task_id: task_id.to_string(),
            type_path: type_path.to_string(),
            view_class: representation.view_class,
        });
    }
}

fn encode(
    value: &dyn Reflect,
    context: &DebugRepresentationContext<'_>,
) -> CuResult<Option<RrdPayload>> {
    let representation = value
        .get_represented_type_info()
        .and_then(DebugRepresentation::for_type)
        .ok_or_else(|| CuError::from("type has no explicit debug representation"))?;
    Ok(Some(RrdPayload {
        bytes: (representation.encode)(value, context)?,
    }))
}

pub(super) fn encode_target<App, P, CB, TF, S, L>(
    state: &mut SessionState<App, P, CB, TF, S, L>,
    target: &RepresentationTarget,
    replayed: bool,
    index: usize,
    context: &DebugRepresentationContext<'_>,
) -> CuResult<Option<RrdPayload>>
where
    App: CuSimApplication<S, L> + ReflectTaskIntrospection + CurrentRuntimeCopperList<P>,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple + 'static,
    CB: for<'a> Fn(
        &'a crate::copperlist::CopperList<P>,
        RobotClock,
        RobotClockMock,
    )
        -> Box<dyn for<'z> FnMut(App::Step<'z>) -> crate::simulation::SimOverride + 'a>,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
{
    Ok(match target {
        RepresentationTarget::TaskState { task_id } => state
            .session
            .with_debug_state(task_id, |value| encode(value, context))??,
        RepresentationTarget::Message { output_index } => {
            if *output_index >= P::get_output_specs().len() {
                return Err("representation output index out of range".into());
            }
            let cl = if replayed {
                state.session.with_app(|app| {
                    app.current_runtime_copperlist_bytes()
                        .map(|bytes| {
                            bincode::decode_from_slice::<crate::copperlist::CopperList<P>, _>(
                                bytes,
                                bincode::config::standard(),
                            )
                            .map(|(cl, _)| Arc::new(cl))
                            .map_err(|e| {
                                CuError::new_with_cause("decode replayed representation input", e)
                            })
                        })
                        .transpose()
                })?
            } else {
                state.session.cl_at(index)?
            };
            match cl {
                Some(cl) => match cl
                    .cumsgs()
                    .get(*output_index)
                    .and_then(|msg| msg.payload_reflect())
                {
                    Some(value) => encode(value, context)?,
                    None => None,
                },
                None => None,
            }
        }
    })
}
