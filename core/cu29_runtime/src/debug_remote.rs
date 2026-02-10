//! Remote debug API (`debug.v1`) over Zenoh.
//!
//! This module exposes a JSON-RPC-like protocol over Zenoh pub/sub to control
//! [`CuDebugSession`](crate::debug::CuDebugSession) remotely.

use crate::app::CuSimApplication;
use crate::config::{BridgeChannelConfigRepresentation, Flavor, read_configuration_str};
use crate::debug::{CuDebugSession, JumpOutcome};
use crate::reflect::{ReflectTaskIntrospection, TypeInfo, TypeRegistry};
use cu29_clock::{CuTime, RobotClock, RobotClockMock, Tov};
use cu29_traits::{CopperListTuple, CuError, CuMsgMetadataTrait, CuResult, ErasedCuStampedDataSet};
use cu29_unifiedlog::{SectionStorage, UnifiedLogWrite};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};
use zenoh::bytes::Encoding;
use zenoh::key_expr::KeyExpr;
use zenoh::{Config as ZenohConfig, Error as ZenohError};

const API_VERSION: &str = "debug.v1";

type ZenohSubscriber =
    zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum WireCodec {
    #[default]
    Cbor,
    Json,
}

impl WireCodec {
    fn encoding(self) -> Encoding {
        match self {
            WireCodec::Cbor => Encoding::APPLICATION_CBOR,
            WireCodec::Json => Encoding::APPLICATION_JSON,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebugRpcRequest {
    pub api: String,
    pub request_id: String,
    #[serde(default)]
    pub session_id: Option<String>,
    pub method: String,
    #[serde(default)]
    pub params: Value,
    pub reply_to: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebugRpcError {
    pub code: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResolvedAt {
    pub cl: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ts_ns: Option<u64>,
    pub idx: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebugRpcResponse {
    pub request_id: String,
    pub ok: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<DebugRpcError>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cursor_rev: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resolved_at: Option<ResolvedAt>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Target {
    Cl { cl: u32 },
    Ts { ts_ns: u64 },
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum ResolveMode {
    Exact,
    #[default]
    AtOrAfter,
    AtOrBefore,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct At {
    pub target: Target,
    #[serde(default)]
    pub mutate_cursor: bool,
    #[serde(default)]
    pub resolve: ResolveMode,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum WatchMode {
    #[default]
    OnCursorChange,
    OnRunProgress,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SessionOpenParams {
    pub log_base: String,
    #[serde(default)]
    pub cache_cap: Option<usize>,
    #[serde(default)]
    pub role: Option<String>,
    #[serde(default)]
    pub codecs: Option<Vec<WireCodec>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SessionCancelParams {
    op_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct NavSeekParams {
    target: Target,
    #[serde(default)]
    resolve: ResolveMode,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct NavRunUntilParams {
    target: Target,
    #[serde(default)]
    resolve: ResolveMode,
    #[serde(default)]
    max_steps: Option<usize>,
    #[serde(default)]
    timeout_ms: Option<u64>,
    #[serde(default)]
    progress_every_n_steps: Option<usize>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct NavStepParams {
    delta: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct NavReplayParams {
    #[serde(default)]
    at: Option<At>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct TimelineGetClParams {
    #[serde(default)]
    at: Option<At>,
    #[serde(default)]
    include_payloads: bool,
    #[serde(default)]
    include_metadata: bool,
    #[serde(default)]
    include_raw: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct TimelineListParams {
    from: Target,
    to: Target,
    #[serde(default)]
    page: Option<Page>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SchemaListTypesParams {
    #[serde(default)]
    filter: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SchemaGetTypeParams {
    type_path: String,
    #[serde(default = "default_schema_format")]
    format: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct StateInspectParams {
    #[serde(default = "default_root_path")]
    path: String,
    #[serde(default)]
    at: Option<At>,
    #[serde(default)]
    depth: Option<u32>,
    #[serde(default)]
    page: Option<Page>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct StateReadParams {
    #[serde(default = "default_root_path")]
    path: String,
    #[serde(default)]
    at: Option<At>,
    #[serde(default)]
    depth: Option<u32>,
    #[serde(default)]
    page: Option<Page>,
    #[serde(default = "default_state_format")]
    format: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct StateSearchParams {
    query: String,
    #[serde(default)]
    at: Option<At>,
    #[serde(default)]
    page: Option<Page>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct StateWatchOpenParams {
    path: String,
    #[serde(default)]
    at: Option<At>,
    #[serde(default)]
    mode: WatchMode,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct StateWatchCloseParams {
    watch_id: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[allow(dead_code)]
struct HealthPingParams {}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[allow(dead_code)]
struct HealthStatsParams {}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct Page {
    offset: u32,
    limit: u32,
}

impl Default for Page {
    fn default() -> Self {
        Self {
            offset: 0,
            limit: 100,
        }
    }
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
struct StateWatch {
    id: u64,
    path: String,
    mode: WatchMode,
}

#[derive(Debug, Clone, Serialize)]
struct CursorSnapshot {
    cl: u32,
    idx: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    ts_ns: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    keyframe_cl: Option<u32>,
    replayed: usize,
    rev: u64,
}

struct SessionState<App, P, CB, TF, S, L>
where
    P: CopperListTuple,
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
{
    session: CuDebugSession<App, P, CB, TF, S, L>,
    opened_at: Instant,
    cursor_rev: u64,
    last_keyframe: Option<u32>,
    last_replayed: usize,
    watches: HashMap<u64, StateWatch>,
    #[allow(dead_code)]
    wire_codec: WireCodec,
}

impl<App, P, CB, TF, S, L> SessionState<App, P, CB, TF, S, L>
where
    P: CopperListTuple,
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
{
    fn bump_rev(&mut self) {
        self.cursor_rev = self.cursor_rev.saturating_add(1);
    }
}

#[derive(Debug, Clone)]
pub struct RemoteDebugPaths {
    pub base: String,
    pub rpc_request: String,
    pub cursor_events: String,
    pub run_events: String,
    pub watch_events: String,
    pub health_events: String,
}

impl RemoteDebugPaths {
    pub fn new(base: &str) -> Self {
        let base = base.trim_end_matches('/').to_string();
        Self {
            rpc_request: format!("{base}/rpc/request"),
            cursor_events: format!("{base}/events/cursor"),
            run_events: format!("{base}/events/run"),
            watch_events: format!("{base}/events/watch"),
            health_events: format!("{base}/events/health"),
            base,
        }
    }
}

#[allow(dead_code)]
struct EventPublishers {
    cursor: zenoh::pubsub::Publisher<'static>,
    run: zenoh::pubsub::Publisher<'static>,
    watch: zenoh::pubsub::Publisher<'static>,
    health: zenoh::pubsub::Publisher<'static>,
}

pub struct RemoteDebugZenohServer<App, P, CB, TF, S, L, AF>
where
    App: CuSimApplication<S, L> + ReflectTaskIntrospection,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple,
    CB: for<'a> Fn(
            &'a crate::copperlist::CopperList<P>,
            RobotClock,
        )
            -> Box<dyn for<'z> FnMut(App::Step<'z>) -> crate::simulation::SimOverride + 'a>
        + Clone,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
    AF: Fn(&SessionOpenParams) -> CuResult<(App, RobotClock, RobotClockMock)>,
{
    paths: RemoteDebugPaths,
    session: zenoh::Session,
    request_sub: ZenohSubscriber,
    event_publishers: EventPublishers,
    app_factory: AF,
    build_callback: CB,
    time_of: TF,
    sessions: HashMap<String, SessionState<App, P, CB, TF, S, L>>,
    next_session_id: u64,
    next_watch_id: u64,
    next_op_id: u64,
    stop_requested: bool,
    stack_schema: Value,
}

impl<App, P, CB, TF, S, L, AF> RemoteDebugZenohServer<App, P, CB, TF, S, L, AF>
where
    App: CuSimApplication<S, L> + ReflectTaskIntrospection,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple,
    CB: for<'a> Fn(
            &'a crate::copperlist::CopperList<P>,
            RobotClock,
        )
            -> Box<dyn for<'z> FnMut(App::Step<'z>) -> crate::simulation::SimOverride + 'a>
        + Clone,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
    AF: Fn(&SessionOpenParams) -> CuResult<(App, RobotClock, RobotClockMock)>,
{
    pub fn new(
        zenoh_config: ZenohConfig,
        paths: RemoteDebugPaths,
        app_factory: AF,
        build_callback: CB,
        time_of: TF,
    ) -> CuResult<Self> {
        let session = zenoh::Wait::wait(zenoh::open(zenoh_config))
            .map_err(cu_error_map("RemoteDebug: failed to open Zenoh session"))?;

        let request_sub =
            zenoh::Wait::wait(session.declare_subscriber(keyexpr(&paths.rpc_request)?)).map_err(
                cu_error_map("RemoteDebug: failed to declare RPC subscriber"),
            )?;

        let cursor_pub =
            zenoh::Wait::wait(session.declare_publisher(keyexpr(&paths.cursor_events)?)).map_err(
                cu_error_map("RemoteDebug: failed to declare cursor events publisher"),
            )?;
        let run_pub = zenoh::Wait::wait(session.declare_publisher(keyexpr(&paths.run_events)?))
            .map_err(cu_error_map(
                "RemoteDebug: failed to declare run events publisher",
            ))?;
        let watch_pub = zenoh::Wait::wait(session.declare_publisher(keyexpr(&paths.watch_events)?))
            .map_err(cu_error_map(
                "RemoteDebug: failed to declare watch events publisher",
            ))?;
        let health_pub =
            zenoh::Wait::wait(session.declare_publisher(keyexpr(&paths.health_events)?)).map_err(
                cu_error_map("RemoteDebug: failed to declare health events publisher"),
            )?;

        let stack_schema = build_stack_schema::<App, S, L>()?;

        Ok(Self {
            paths,
            session,
            request_sub,
            event_publishers: EventPublishers {
                cursor: cursor_pub,
                run: run_pub,
                watch: watch_pub,
                health: health_pub,
            },
            app_factory,
            build_callback,
            time_of,
            sessions: HashMap::new(),
            next_session_id: 1,
            next_watch_id: 1,
            next_op_id: 1,
            stop_requested: false,
            stack_schema,
        })
    }

    pub fn serve_until_stopped(&mut self) -> CuResult<()> {
        while !self.stop_requested {
            self.serve_next()?;
        }
        Ok(())
    }

    pub fn serve_next(&mut self) -> CuResult<()> {
        let sample = self.request_sub.recv().map_err(|e| {
            CuError::from(format!("RemoteDebug: failed to receive RPC request: {e}"))
        })?;
        let payload = sample.payload().to_bytes();
        let (request, codec) = decode_request(payload.as_ref())?;

        let response = self.handle_request(request.clone());
        self.publish_reply(&request.reply_to, &response, codec)?;
        Ok(())
    }

    fn handle_request(&mut self, request: DebugRpcRequest) -> DebugRpcResponse {
        let request_id = request.request_id.clone();

        if request.api != API_VERSION {
            return err_response(
                request_id,
                "InvalidApi",
                &format!(
                    "Unsupported API '{}', expected '{}'.",
                    request.api, API_VERSION
                ),
            );
        }

        if request.method == "admin.stop" {
            self.stop_requested = true;
            return ok_response(request_id, json!({"stopping": true}), None, None);
        }

        self.dispatch_request(&request)
    }

    fn dispatch_request(&mut self, request: &DebugRpcRequest) -> DebugRpcResponse {
        let request_id = request.request_id.clone();

        match request.method.as_str() {
            "session.open" => self.handle_session_open(request_id, &request.params),
            "session.close" => self.handle_session_close(request_id, request.session_id.as_deref()),
            "session.capabilities" => {
                self.handle_session_capabilities(request_id, request.session_id.as_deref())
            }
            "session.cancel" => self.handle_session_cancel(
                request_id,
                request.session_id.as_deref(),
                &request.params,
            ),
            "nav.seek" => {
                self.handle_nav_seek(request_id, request.session_id.as_deref(), &request.params)
            }
            "nav.run_until" => self.handle_nav_run_until(
                request_id,
                request.session_id.as_deref(),
                &request.params,
            ),
            "nav.step" => {
                self.handle_nav_step(request_id, request.session_id.as_deref(), &request.params)
            }
            "nav.replay" => {
                self.handle_nav_replay(request_id, request.session_id.as_deref(), &request.params)
            }
            "timeline.get_cursor" => {
                self.handle_timeline_get_cursor(request_id, request.session_id.as_deref())
            }
            "timeline.get_cl" => self.handle_timeline_get_cl(
                request_id,
                request.session_id.as_deref(),
                &request.params,
            ),
            "timeline.list" => self.handle_timeline_list(
                request_id,
                request.session_id.as_deref(),
                &request.params,
            ),
            "schema.get_stack" => {
                self.handle_schema_get_stack(request_id, request.session_id.as_deref())
            }
            "schema.list_types" => self.handle_schema_list_types(
                request_id,
                request.session_id.as_deref(),
                &request.params,
            ),
            "schema.get_type" => self.handle_schema_get_type(
                request_id,
                request.session_id.as_deref(),
                &request.params,
            ),
            "schema.get_payload_map" => {
                self.handle_schema_get_payload_map(request_id, request.session_id.as_deref())
            }
            "state.inspect" => self.handle_state_inspect(
                request_id,
                request.session_id.as_deref(),
                &request.params,
            ),
            "state.read" => {
                self.handle_state_read(request_id, request.session_id.as_deref(), &request.params)
            }
            "state.search" => {
                self.handle_state_search(request_id, request.session_id.as_deref(), &request.params)
            }
            "state.watch.open" => self.handle_state_watch_open(
                request_id,
                request.session_id.as_deref(),
                &request.params,
            ),
            "state.watch.close" => self.handle_state_watch_close(
                request_id,
                request.session_id.as_deref(),
                &request.params,
            ),
            "health.ping" => {
                self.handle_health_ping(request_id, request.session_id.as_deref(), &request.params)
            }
            "health.stats" => {
                self.handle_health_stats(request_id, request.session_id.as_deref(), &request.params)
            }
            _ => err_response(
                request_id,
                "UnknownMethod",
                &format!("Unknown RPC method '{}'.", request.method),
            ),
        }
    }

    fn handle_session_open(&mut self, request_id: String, params: &Value) -> DebugRpcResponse {
        let parsed: SessionOpenParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };
        let wire_codec = negotiate_codec(parsed.codecs.as_deref()).unwrap_or(WireCodec::Cbor);

        let path = PathBuf::from(&parsed.log_base);
        let (app, clock, clock_mock) = match (self.app_factory)(&parsed) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionOpenFailed", &e.to_string()),
        };

        let session = match parsed.cache_cap {
            Some(cap) => CuDebugSession::<App, P, CB, TF, S, L>::from_log_with_cache_cap(
                path.as_path(),
                app,
                clock,
                clock_mock,
                self.build_callback.clone(),
                self.time_of.clone(),
                cap,
            ),
            None => CuDebugSession::<App, P, CB, TF, S, L>::from_log(
                path.as_path(),
                app,
                clock,
                clock_mock,
                self.build_callback.clone(),
                self.time_of.clone(),
            ),
        };

        let session = match session {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionOpenFailed", &e.to_string()),
        };

        let session_id = format!("s{}", self.next_session_id);
        self.next_session_id = self.next_session_id.saturating_add(1);

        self.sessions.insert(
            session_id.clone(),
            SessionState {
                session,
                opened_at: Instant::now(),
                cursor_rev: 0,
                last_keyframe: None,
                last_replayed: 0,
                watches: HashMap::new(),
                wire_codec,
            },
        );

        ok_response(
            request_id,
            json!({
                "session_id": session_id,
                "capabilities": capabilities_json(),
                "wire_codec": wire_codec,
                "initial_cursor": Value::Null,
            }),
            None,
            None,
        )
    }

    fn handle_session_close(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
    ) -> DebugRpcResponse {
        let sid = match session_id {
            Some(v) => v,
            None => return err_response(request_id, "MissingSession", "session_id is required"),
        };
        let removed = self.sessions.remove(sid).is_some();
        ok_response(
            request_id,
            json!({
                "session_id": sid,
                "closed": removed,
            }),
            None,
            None,
        )
    }

    fn handle_session_capabilities(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
    ) -> DebugRpcResponse {
        let sid = match session_id {
            Some(v) => v,
            None => return err_response(request_id, "MissingSession", "session_id is required"),
        };
        if !self.sessions.contains_key(sid) {
            return err_response(request_id, "SessionNotFound", "session not found");
        }

        ok_response(request_id, capabilities_json(), None, None)
    }

    fn handle_session_cancel(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let sid = match session_id {
            Some(v) => v,
            None => return err_response(request_id, "MissingSession", "session_id is required"),
        };
        if !self.sessions.contains_key(sid) {
            return err_response(request_id, "SessionNotFound", "session not found");
        }
        let parsed: SessionCancelParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        ok_response(
            request_id,
            json!({
                "op_id": parsed.op_id,
                "cancelled": false,
            }),
            None,
            None,
        )
    }

    fn handle_nav_seek(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let parsed: NavSeekParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };
        let time_of = self.time_of.clone();

        let state = match self.session_mut(session_id) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        let resolved =
            match resolve_target(&mut state.session, &parsed.target, parsed.resolve, &time_of) {
                Ok(v) => v,
                Err(e) => return err_response(request_id, "ResolveFailed", &e.to_string()),
            };

        let jump = match seek_to_index(&mut state.session, resolved.idx) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SeekFailed", &e.to_string()),
        };

        update_after_jump(state, &jump);
        let cursor = match cursor_snapshot(state, &time_of) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SeekFailed", &e.to_string()),
        };

        ok_response(
            request_id,
            json!({"cursor": cursor}),
            Some(state.cursor_rev),
            Some(resolved),
        )
    }

    fn handle_nav_run_until(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let parsed: NavRunUntilParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let sid = match session_id {
            Some(v) => v,
            None => return err_response(request_id, "MissingSession", "session_id is required"),
        };
        let time_of = self.time_of.clone();
        let op_id = format!("op{}", self.next_op_id);
        self.next_op_id = self.next_op_id.saturating_add(1);

        let state = match self.session_mut(Some(sid)) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        let resolved =
            match resolve_target(&mut state.session, &parsed.target, parsed.resolve, &time_of) {
                Ok(v) => v,
                Err(e) => return err_response(request_id, "ResolveFailed", &e.to_string()),
            };

        let jump = match seek_to_index(&mut state.session, resolved.idx) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "RunUntilFailed", &e.to_string()),
        };
        update_after_jump(state, &jump);

        let cursor = match cursor_snapshot(state, &time_of) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "RunUntilFailed", &e.to_string()),
        };

        let stopped_reason = if let Some(max_steps) = parsed.max_steps {
            if jump.replayed > max_steps {
                "max_steps"
            } else {
                "target_reached"
            }
        } else {
            "target_reached"
        };

        ok_response(
            request_id,
            json!({
                "op_id": op_id,
                "cursor": cursor,
                "steps": jump.replayed,
                "stopped_reason": stopped_reason,
            }),
            Some(state.cursor_rev),
            Some(resolved),
        )
    }

    fn handle_nav_step(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let parsed: NavStepParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let sid = match session_id {
            Some(v) => v,
            None => return err_response(request_id, "MissingSession", "session_id is required"),
        };
        let time_of = self.time_of.clone();

        let state = match self.session_mut(Some(sid)) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        let jump = match state.session.step(parsed.delta) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "StepFailed", &e.to_string()),
        };
        update_after_jump(state, &jump);

        let cursor = match cursor_snapshot(state, &time_of) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "StepFailed", &e.to_string()),
        };

        ok_response(
            request_id,
            json!({"cursor": cursor}),
            Some(state.cursor_rev),
            None,
        )
    }

    fn handle_nav_replay(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let parsed: NavReplayParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let sid = match session_id {
            Some(v) => v,
            None => return err_response(request_id, "MissingSession", "session_id is required"),
        };
        let time_of = self.time_of.clone();

        let state = match self.session_mut(Some(sid)) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        let mut resolved_at = None;
        if let Some(at) = parsed.at.as_ref() {
            let resolved =
                match resolve_target(&mut state.session, &at.target, at.resolve, &time_of) {
                    Ok(v) => v,
                    Err(e) => return err_response(request_id, "ResolveFailed", &e.to_string()),
                };
            resolved_at = Some(resolved.clone());
            let jump = match seek_to_index(&mut state.session, resolved.idx) {
                Ok(v) => v,
                Err(e) => return err_response(request_id, "ReplayFailed", &e.to_string()),
            };
            update_after_jump(state, &jump);
        }

        let replayed = match replay_current_step(&mut state.session) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "ReplayFailed", &e.to_string()),
        };
        update_after_jump(state, &replayed);

        let cursor = match cursor_snapshot(state, &time_of) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "ReplayFailed", &e.to_string()),
        };

        ok_response(
            request_id,
            json!({
                "cursor": cursor,
                "replayed": 1,
            }),
            Some(state.cursor_rev),
            resolved_at,
        )
    }

    fn handle_timeline_get_cursor(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
    ) -> DebugRpcResponse {
        let time_of = self.time_of.clone();
        let state = match self.session_mut(session_id) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        match cursor_snapshot(state, &time_of) {
            Ok(cursor) => ok_response(
                request_id,
                json!({"cursor": cursor}),
                Some(state.cursor_rev),
                None,
            ),
            Err(e) => err_response(request_id, "CursorFailed", &e.to_string()),
        }
    }

    fn handle_timeline_get_cl(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let time_of = self.time_of.clone();
        let parsed: TimelineGetClParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let state = match self.session_mut(session_id) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        let include_payloads = parsed.include_payloads;
        let include_metadata = parsed.include_metadata;
        let include_raw = parsed.include_raw;

        let mut resolved_at = None;
        let cl = if let Some(at) = parsed.at.as_ref() {
            let resolved =
                match resolve_target(&mut state.session, &at.target, at.resolve, &time_of) {
                    Ok(v) => v,
                    Err(e) => return err_response(request_id, "ResolveFailed", &e.to_string()),
                };
            resolved_at = Some(resolved.clone());

            if at.mutate_cursor {
                let jump = match seek_to_index(&mut state.session, resolved.idx) {
                    Ok(v) => v,
                    Err(e) => return err_response(request_id, "GetClFailed", &e.to_string()),
                };
                update_after_jump(state, &jump);
                match state.session.current_cl() {
                    Ok(Some(cl)) => cl,
                    Ok(None) => {
                        return err_response(
                            request_id,
                            "GetClFailed",
                            "no current copperlist after seek",
                        );
                    }
                    Err(e) => return err_response(request_id, "GetClFailed", &e.to_string()),
                }
            } else {
                match state.session.cl_at(resolved.idx) {
                    Ok(Some(cl)) => cl,
                    Ok(None) => {
                        return err_response(request_id, "GetClFailed", "target copperlist missing");
                    }
                    Err(e) => return err_response(request_id, "GetClFailed", &e.to_string()),
                }
            }
        } else {
            match state.session.current_cl() {
                Ok(Some(cl)) => cl,
                Ok(None) => return err_response(request_id, "GetClFailed", "no current cursor"),
                Err(e) => return err_response(request_id, "GetClFailed", &e.to_string()),
            }
        };

        let snapshot = match copperlist_snapshot::<P>(
            cl.as_ref(),
            &time_of,
            include_payloads,
            include_metadata,
            include_raw,
        ) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "GetClFailed", &e.to_string()),
        };

        let cursor = match cursor_snapshot(state, &time_of) {
            Ok(c) => c,
            Err(_) => CursorSnapshot {
                cl: cl.id,
                idx: resolved_at.as_ref().map(|r| r.idx).unwrap_or(0),
                ts_ns: (time_of)(cl.as_ref()).map(|t| t.as_nanos()),
                keyframe_cl: state.last_keyframe,
                replayed: state.last_replayed,
                rev: state.cursor_rev,
            },
        };

        ok_response(
            request_id,
            json!({
                "cursor": cursor,
                "cl_snapshot": snapshot,
            }),
            Some(state.cursor_rev),
            resolved_at,
        )
    }

    fn handle_timeline_list(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let time_of = self.time_of.clone();
        let parsed: TimelineListParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let page = parsed.page.unwrap_or_default();

        let state = match self.session_mut(session_id) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        let start = match resolve_target(
            &mut state.session,
            &parsed.from,
            ResolveMode::AtOrAfter,
            &time_of,
        ) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "ResolveFailed", &e.to_string()),
        };
        let end = match resolve_target(
            &mut state.session,
            &parsed.to,
            ResolveMode::AtOrBefore,
            &time_of,
        ) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "ResolveFailed", &e.to_string()),
        };

        if end.idx < start.idx {
            return ok_response(
                request_id,
                json!({"items": [], "next_offset": Value::Null}),
                Some(state.cursor_rev),
                None,
            );
        }

        let mut items = Vec::new();
        let mut idx = start.idx.saturating_add(page.offset as usize);
        let max_idx = end.idx;
        let mut emitted = 0usize;

        while idx <= max_idx && emitted < page.limit as usize {
            let cl = match state.session.cl_at(idx) {
                Ok(Some(cl)) => cl,
                Ok(None) => break,
                Err(e) => return err_response(request_id, "TimelineListFailed", &e.to_string()),
            };
            let ts = (time_of)(cl.as_ref()).map(|t| t.as_nanos());
            items.push(json!({
                "idx": idx,
                "cl": cl.id,
                "ts_ns": ts,
            }));
            idx = idx.saturating_add(1);
            emitted += 1;
        }

        let next_offset = if idx <= max_idx {
            Some((page.offset as usize + emitted) as u32)
        } else {
            None
        };

        ok_response(
            request_id,
            json!({
                "items": items,
                "next_offset": next_offset,
            }),
            Some(state.cursor_rev),
            None,
        )
    }

    fn handle_schema_get_stack(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
    ) -> DebugRpcResponse {
        if let Err(e) = self.session_mut(session_id) {
            return err_response(request_id, "SessionNotFound", &e.to_string());
        }
        ok_response(request_id, self.stack_schema.clone(), None, None)
    }

    fn handle_schema_list_types(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        if let Err(e) = self.session_mut(session_id) {
            return err_response(request_id, "SessionNotFound", &e.to_string());
        }

        let parsed: SchemaListTypesParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let mut registry = TypeRegistry::default();
        <App as ReflectTaskIntrospection>::register_reflect_types(&mut registry);

        let mut paths: Vec<String> = registry
            .iter()
            .map(|registration| registration.type_info().type_path().to_string())
            .collect();
        paths.sort();
        paths.dedup();

        if let Some(filter) = parsed.filter {
            paths.retain(|p| p.contains(&filter));
        }

        ok_response(request_id, json!({"type_paths": paths}), None, None)
    }

    fn handle_schema_get_type(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        if let Err(e) = self.session_mut(session_id) {
            return err_response(request_id, "SessionNotFound", &e.to_string());
        }

        let parsed: SchemaGetTypeParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let mut registry = TypeRegistry::default();
        <App as ReflectTaskIntrospection>::register_reflect_types(&mut registry);

        let info = match registry.get_with_type_path(&parsed.type_path) {
            Some(reg) => reg.type_info(),
            None => {
                return err_response(
                    request_id,
                    "TypeNotFound",
                    &format!("Type '{}' is not registered", parsed.type_path),
                );
            }
        };

        let schema = if parsed.format == "jsonschema" {
            simple_jsonschema_for_type(info)
        } else {
            json!({
                "type_path": info.type_path(),
                "reflect_dump": format!("{info:#?}"),
            })
        };

        ok_response(request_id, json!({"schema": schema}), None, None)
    }

    fn handle_schema_get_payload_map(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
    ) -> DebugRpcResponse {
        if let Err(e) = self.session_mut(session_id) {
            return err_response(request_id, "SessionNotFound", &e.to_string());
        }

        let outputs: Vec<Value> = P::get_all_task_ids()
            .iter()
            .enumerate()
            .map(|(index, task_id)| {
                json!({
                    "index": index,
                    "task_id": task_id,
                })
            })
            .collect();

        ok_response(request_id, json!({"outputs": outputs}), None, None)
    }

    fn handle_state_inspect(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let time_of = self.time_of.clone();
        let parsed: StateInspectParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let state = match self.session_mut(session_id) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        let (root, resolved_at) =
            match state_root_for_query::<App, P, CB, TF, S, L>(state, parsed.at.as_ref(), &time_of)
            {
                Ok(v) => v,
                Err(e) => return err_response(request_id, "StateFailed", &e.to_string()),
            };

        let node = match navigate_path(&root, &parsed.path) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "PathNotFound", &e),
        };

        let page = parsed.page.unwrap_or_default();
        let inspected = inspect_value(node, page);
        let cursor = cursor_snapshot(state, &time_of).ok();

        ok_response(
            request_id,
            json!({
                "cursor": cursor,
                "node_kind": inspected.kind,
                "type_path": Value::Null,
                "children": inspected.children,
                "total_children": inspected.total_children,
            }),
            Some(state.cursor_rev),
            resolved_at,
        )
    }

    fn handle_state_read(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let time_of = self.time_of.clone();
        let parsed: StateReadParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let state = match self.session_mut(session_id) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        let (root, resolved_at) =
            match state_root_for_query::<App, P, CB, TF, S, L>(state, parsed.at.as_ref(), &time_of)
            {
                Ok(v) => v,
                Err(e) => return err_response(request_id, "StateFailed", &e.to_string()),
            };

        let node = match navigate_path(&root, &parsed.path) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "PathNotFound", &e),
        };

        let page = parsed.page.unwrap_or_default();
        let value = apply_page(node.clone(), page);
        let cursor = cursor_snapshot(state, &time_of).ok();

        ok_response(
            request_id,
            json!({
                "cursor": cursor,
                "value": value,
                "format": parsed.format,
            }),
            Some(state.cursor_rev),
            resolved_at,
        )
    }

    fn handle_state_search(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let time_of = self.time_of.clone();
        let parsed: StateSearchParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let state = match self.session_mut(session_id) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        let (root, resolved_at) =
            match state_root_for_query::<App, P, CB, TF, S, L>(state, parsed.at.as_ref(), &time_of)
            {
                Ok(v) => v,
                Err(e) => return err_response(request_id, "StateFailed", &e.to_string()),
            };

        let mut matches = Vec::new();
        collect_matches(&root, "", &parsed.query.to_ascii_lowercase(), &mut matches);

        let page = parsed.page.unwrap_or_default();
        let start = page.offset as usize;
        let end = (start + page.limit as usize).min(matches.len());
        let sliced = if start < matches.len() {
            matches[start..end].to_vec()
        } else {
            Vec::new()
        };

        let cursor = cursor_snapshot(state, &time_of).ok();

        ok_response(
            request_id,
            json!({
                "cursor": cursor,
                "matches": sliced,
                "total": matches.len(),
            }),
            Some(state.cursor_rev),
            resolved_at,
        )
    }

    fn handle_state_watch_open(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let time_of = self.time_of.clone();
        let watch_events_topic = self.paths.watch_events.clone();
        let watch_id = self.next_watch_id;
        self.next_watch_id = self.next_watch_id.saturating_add(1);
        let parsed: StateWatchOpenParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let sid = match session_id {
            Some(v) => v,
            None => return err_response(request_id, "MissingSession", "session_id is required"),
        };

        let state = match self.session_mut(Some(sid)) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        if let Some(at) = parsed.at.as_ref() {
            let resolved =
                match resolve_target(&mut state.session, &at.target, at.resolve, &time_of) {
                    Ok(v) => v,
                    Err(e) => return err_response(request_id, "ResolveFailed", &e.to_string()),
                };
            if at.mutate_cursor {
                let jump = match seek_to_index(&mut state.session, resolved.idx) {
                    Ok(v) => v,
                    Err(e) => return err_response(request_id, "WatchOpenFailed", &e.to_string()),
                };
                update_after_jump(state, &jump);
            }
        }

        state.watches.insert(
            watch_id,
            StateWatch {
                id: watch_id,
                path: parsed.path.clone(),
                mode: parsed.mode.clone(),
            },
        );

        ok_response(
            request_id,
            json!({
                "watch_id": watch_id,
                "event_topic": watch_events_topic,
            }),
            Some(state.cursor_rev),
            None,
        )
    }

    fn handle_state_watch_close(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let parsed: StateWatchCloseParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

        let state = match self.session_mut(session_id) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        let removed = state.watches.remove(&parsed.watch_id).is_some();
        ok_response(
            request_id,
            json!({
                "watch_id": parsed.watch_id,
                "closed": removed,
            }),
            Some(state.cursor_rev),
            None,
        )
    }

    fn handle_health_ping(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        _params: &Value,
    ) -> DebugRpcResponse {
        if let Some(sid) = session_id
            && !self.sessions.contains_key(sid)
        {
            return err_response(request_id, "SessionNotFound", "session not found");
        }

        let now_ns = now_unix_ns();
        ok_response(
            request_id,
            json!({
                "now_ns": now_ns,
                "status": "ok",
            }),
            None,
            None,
        )
    }

    fn handle_health_stats(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        _params: &Value,
    ) -> DebugRpcResponse {
        let sid = match session_id {
            Some(v) => v,
            None => return err_response(request_id, "MissingSession", "session_id is required"),
        };

        let state = match self.sessions.get(sid) {
            Some(v) => v,
            None => return err_response(request_id, "SessionNotFound", "session not found"),
        };

        ok_response(
            request_id,
            json!({
                "cache": {
                    "note": "cache stats not yet exposed by CuDebugSession"
                },
                "replay_perf": {
                    "last_replayed": state.last_replayed,
                },
                "uptime_ms": state.opened_at.elapsed().as_millis() as u64,
                "active_watches": state.watches.len(),
                "cursor_rev": state.cursor_rev,
            }),
            Some(state.cursor_rev),
            None,
        )
    }

    fn publish_reply(
        &self,
        topic: &str,
        response: &DebugRpcResponse,
        codec: WireCodec,
    ) -> CuResult<()> {
        let payload = encode_payload(response, codec, "RemoteDebug: failed to serialize response")?;
        let publisher = zenoh::Wait::wait(self.session.declare_publisher(keyexpr(topic)?))
            .map_err(cu_error_map(
                "RemoteDebug: failed to declare reply publisher",
            ))?;
        zenoh::Wait::wait(publisher.put(payload).encoding(codec.encoding()))
            .map_err(cu_error_map("RemoteDebug: failed to publish reply"))?;
        zenoh::Wait::wait(publisher.undeclare()).map_err(cu_error_map(
            "RemoteDebug: failed to undeclare reply publisher",
        ))?;
        Ok(())
    }

    #[allow(dead_code)]
    fn publish_event(
        &self,
        publisher: &zenoh::pubsub::Publisher<'static>,
        value: &Value,
        codec: WireCodec,
    ) -> CuResult<()> {
        let payload = encode_payload(value, codec, "RemoteDebug: failed to serialize event")?;
        zenoh::Wait::wait(publisher.put(payload).encoding(codec.encoding()))
            .map_err(cu_error_map("RemoteDebug: failed to publish event"))?;
        Ok(())
    }

    #[allow(dead_code)]
    fn publish_cursor_event(
        &self,
        session_id: &str,
        cursor: &CursorSnapshot,
        cause: &str,
        codec: WireCodec,
    ) -> CuResult<()> {
        self.publish_event(
            &self.event_publishers.cursor,
            &json!({
                "session_id": session_id,
                "cursor": cursor,
                "cause": cause,
            }),
            codec,
        )
    }

    #[allow(dead_code)]
    fn publish_watch_events(
        &self,
        session_id: &str,
        state: &mut SessionState<App, P, CB, TF, S, L>,
        trigger_mode: WatchMode,
        codec: WireCodec,
    ) -> CuResult<()> {
        let root = build_state_root_json::<App, P, CB, TF, S, L>(state, &self.time_of)?;
        let cursor = cursor_snapshot(state, &self.time_of).ok();

        for watch in state.watches.values() {
            let mode_matches = matches!(
                (&watch.mode, &trigger_mode),
                (WatchMode::OnCursorChange, WatchMode::OnCursorChange)
                    | (WatchMode::OnRunProgress, WatchMode::OnRunProgress)
            );
            if !mode_matches {
                continue;
            }

            let preview = match navigate_path(&root, &watch.path) {
                Ok(v) => preview_json(v),
                Err(e) => Value::String(e),
            };

            self.publish_event(
                &self.event_publishers.watch,
                &json!({
                    "session_id": session_id,
                    "watch_id": watch.id,
                    "path": watch.path,
                    "cursor": cursor,
                    "value_preview": preview,
                }),
                codec,
            )?;
        }

        Ok(())
    }

    fn session_mut(
        &mut self,
        session_id: Option<&str>,
    ) -> CuResult<&mut SessionState<App, P, CB, TF, S, L>> {
        let sid = session_id.ok_or_else(|| CuError::from("session_id is required"))?;
        self.sessions
            .get_mut(sid)
            .ok_or_else(|| CuError::from(format!("Session '{sid}' not found")))
    }
}

fn ok_response(
    request_id: String,
    result: Value,
    cursor_rev: Option<u64>,
    resolved_at: Option<ResolvedAt>,
) -> DebugRpcResponse {
    DebugRpcResponse {
        request_id,
        ok: true,
        result: Some(result),
        error: None,
        cursor_rev,
        resolved_at,
    }
}

fn err_response(request_id: String, code: &str, message: &str) -> DebugRpcResponse {
    DebugRpcResponse {
        request_id,
        ok: false,
        result: None,
        error: Some(DebugRpcError {
            code: code.to_string(),
            message: message.to_string(),
            details: None,
        }),
        cursor_rev: None,
        resolved_at: None,
    }
}

fn param_err_response(request_id: String, message: String) -> DebugRpcResponse {
    err_response(request_id, "InvalidParams", &message)
}

pub struct RemoteDebugZenohClient {
    _paths: RemoteDebugPaths,
    session: zenoh::Session,
    request_pub: zenoh::pubsub::Publisher<'static>,
    reply_sub: ZenohSubscriber,
    reply_topic: String,
    next_request_id: AtomicU64,
    codec: WireCodec,
}

impl RemoteDebugZenohClient {
    pub fn new(
        zenoh_config: ZenohConfig,
        paths: RemoteDebugPaths,
        client_id: &str,
    ) -> CuResult<Self> {
        Self::new_with_codec(zenoh_config, paths, client_id, WireCodec::Cbor)
    }

    pub fn new_with_codec(
        zenoh_config: ZenohConfig,
        paths: RemoteDebugPaths,
        client_id: &str,
        codec: WireCodec,
    ) -> CuResult<Self> {
        let session = zenoh::Wait::wait(zenoh::open(zenoh_config)).map_err(cu_error_map(
            "RemoteDebugClient: failed to open Zenoh session",
        ))?;

        let request_pub =
            zenoh::Wait::wait(session.declare_publisher(keyexpr(&paths.rpc_request)?)).map_err(
                cu_error_map("RemoteDebugClient: failed to declare request publisher"),
            )?;

        let reply_topic = format!("{}/rpc/reply/{client_id}", paths.base);
        let reply_sub = zenoh::Wait::wait(session.declare_subscriber(keyexpr(&reply_topic)?))
            .map_err(cu_error_map(
                "RemoteDebugClient: failed to declare reply subscriber",
            ))?;

        Ok(Self {
            _paths: paths,
            session,
            request_pub,
            reply_sub,
            reply_topic,
            next_request_id: AtomicU64::new(1),
            codec,
        })
    }

    pub fn subscribe_events(&self, topic: &str) -> CuResult<ZenohSubscriber> {
        zenoh::Wait::wait(self.session.declare_subscriber(keyexpr(topic)?)).map_err(cu_error_map(
            "RemoteDebugClient: failed to declare events subscriber",
        ))
    }

    pub fn call(
        &self,
        session_id: Option<&str>,
        method: &str,
        params: Value,
    ) -> CuResult<DebugRpcResponse> {
        let request_id = format!("req{}", self.next_request_id.fetch_add(1, Ordering::SeqCst));
        let request = DebugRpcRequest {
            api: API_VERSION.to_string(),
            request_id: request_id.clone(),
            session_id: session_id.map(ToOwned::to_owned),
            method: method.to_string(),
            params,
            reply_to: self.reply_topic.clone(),
        };

        let payload = encode_payload(
            &request,
            self.codec,
            "RemoteDebugClient: request encode failed",
        )?;
        zenoh::Wait::wait(
            self.request_pub
                .put(payload)
                .encoding(self.codec.encoding()),
        )
        .map_err(cu_error_map("RemoteDebugClient: failed to send request"))?;

        loop {
            let sample = self.reply_sub.recv().map_err(|e| {
                CuError::from(format!("RemoteDebugClient: failed receiving reply: {e}"))
            })?;
            let payload = sample.payload().to_bytes();
            let response = decode_response(payload.as_ref(), self.codec)?;

            if response.request_id == request_id {
                return Ok(response);
            }
        }
    }
}

fn capabilities_json() -> Value {
    json!({
        "version": API_VERSION,
        "wire_codecs": ["cbor", "json"],
        "supports_targets": ["cl", "ts"],
        "supports_methods": [
            "session.open",
            "session.close",
            "session.capabilities",
            "session.cancel",
            "nav.seek",
            "nav.run_until",
            "nav.step",
            "nav.replay",
            "timeline.get_cursor",
            "timeline.get_cl",
            "timeline.list",
            "schema.get_stack",
            "schema.list_types",
            "schema.get_type",
            "schema.get_payload_map",
            "state.inspect",
            "state.read",
            "state.search",
            "state.watch.open",
            "state.watch.close",
            "health.ping",
            "health.stats"
        ]
    })
}

fn negotiate_codec(client: Option<&[WireCodec]>) -> Option<WireCodec> {
    let supported = [WireCodec::Cbor, WireCodec::Json];
    match client {
        Some(codecs) if !codecs.is_empty() => {
            codecs.iter().copied().find(|c| supported.contains(c))
        }
        _ => Some(WireCodec::Cbor),
    }
}

fn decode_request(bytes: &[u8]) -> CuResult<(DebugRpcRequest, WireCodec)> {
    if let Ok(request) = minicbor_serde::from_slice::<DebugRpcRequest>(bytes) {
        return Ok((request, WireCodec::Cbor));
    }
    if let Ok(request) = serde_json::from_slice::<DebugRpcRequest>(bytes) {
        return Ok((request, WireCodec::Json));
    }
    Err(CuError::from(
        "RemoteDebug: failed to decode request as CBOR or JSON",
    ))
}

fn decode_response(bytes: &[u8], codec: WireCodec) -> CuResult<DebugRpcResponse> {
    match codec {
        WireCodec::Cbor => minicbor_serde::from_slice::<DebugRpcResponse>(bytes)
            .map_err(|e| CuError::new_with_cause("RemoteDebugClient: invalid CBOR response", e)),
        WireCodec::Json => serde_json::from_slice::<DebugRpcResponse>(bytes)
            .map_err(|e| CuError::new_with_cause("RemoteDebugClient: invalid JSON response", e)),
    }
}

fn encode_payload<T: Serialize>(value: &T, codec: WireCodec, context: &str) -> CuResult<Vec<u8>> {
    match codec {
        WireCodec::Cbor => {
            minicbor_serde::to_vec(value).map_err(|e| CuError::new_with_cause(context, e))
        }
        WireCodec::Json => {
            serde_json::to_vec(value).map_err(|e| CuError::new_with_cause(context, e))
        }
    }
}

fn update_after_jump<App, P, CB, TF, S, L>(
    state: &mut SessionState<App, P, CB, TF, S, L>,
    jump: &JumpOutcome,
) where
    P: CopperListTuple,
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
{
    state.last_keyframe = jump.keyframe_culistid;
    state.last_replayed = jump.replayed;
    state.bump_rev();
}

fn seek_to_index<App, P, CB, TF, S, L>(
    session: &mut CuDebugSession<App, P, CB, TF, S, L>,
    idx: usize,
) -> CuResult<JumpOutcome>
where
    App: CuSimApplication<S, L>,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple,
    CB: for<'a> Fn(
        &'a crate::copperlist::CopperList<P>,
        RobotClock,
    )
        -> Box<dyn for<'z> FnMut(App::Step<'z>) -> crate::simulation::SimOverride + 'a>,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
{
    let cl = session
        .cl_at(idx)?
        .ok_or_else(|| CuError::from(format!("No copperlist at idx {idx}")))?;
    session.goto_cl(cl.id)
}

fn replay_current_step<App, P, CB, TF, S, L>(
    session: &mut CuDebugSession<App, P, CB, TF, S, L>,
) -> CuResult<JumpOutcome>
where
    App: CuSimApplication<S, L>,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple,
    CB: for<'a> Fn(
        &'a crate::copperlist::CopperList<P>,
        RobotClock,
    )
        -> Box<dyn for<'z> FnMut(App::Step<'z>) -> crate::simulation::SimOverride + 'a>,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
{
    let idx = session
        .current_index()
        .ok_or_else(|| CuError::from("Cannot replay before any jump/seek"))?;
    if idx == 0 {
        return Err(CuError::from(
            "Cannot replay first copperlist (no predecessor)",
        ));
    }
    session.step(-1)?;
    session.step(1)
}

fn cursor_snapshot<App, P, CB, TF, S, L>(
    state: &mut SessionState<App, P, CB, TF, S, L>,
    time_of: &TF,
) -> CuResult<CursorSnapshot>
where
    App: CuSimApplication<S, L>,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple,
    CB: for<'a> Fn(
        &'a crate::copperlist::CopperList<P>,
        RobotClock,
    )
        -> Box<dyn for<'z> FnMut(App::Step<'z>) -> crate::simulation::SimOverride + 'a>,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
{
    let idx = state
        .session
        .current_index()
        .ok_or_else(|| CuError::from("No current cursor"))?;
    let cl = state
        .session
        .cl_at(idx)?
        .ok_or_else(|| CuError::from("Current cursor points to missing copperlist"))?;
    Ok(CursorSnapshot {
        cl: cl.id,
        idx,
        ts_ns: time_of(cl.as_ref()).map(|t| t.as_nanos()),
        keyframe_cl: state.last_keyframe,
        replayed: state.last_replayed,
        rev: state.cursor_rev,
    })
}

fn resolve_target<App, P, CB, TF, S, L>(
    session: &mut CuDebugSession<App, P, CB, TF, S, L>,
    target: &Target,
    mode: ResolveMode,
    time_of: &TF,
) -> CuResult<ResolvedAt>
where
    App: CuSimApplication<S, L>,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple,
    CB: for<'a> Fn(
        &'a crate::copperlist::CopperList<P>,
        RobotClock,
    )
        -> Box<dyn for<'z> FnMut(App::Step<'z>) -> crate::simulation::SimOverride + 'a>,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
{
    let total = session.total_entries();

    match target {
        Target::Cl { cl } => {
            let mut best_after: Option<ResolvedAt> = None;
            let mut best_before: Option<ResolvedAt> = None;

            for idx in 0..total {
                let entry = session
                    .cl_at(idx)?
                    .ok_or_else(|| CuError::from("Corrupt session index"))?;
                let ts = time_of(entry.as_ref()).map(|t| t.as_nanos());
                let this = ResolvedAt {
                    cl: entry.id,
                    ts_ns: ts,
                    idx,
                };

                if entry.id == *cl {
                    return Ok(this);
                }
                if entry.id > *cl && best_after.is_none() {
                    best_after = Some(this.clone());
                }
                if entry.id < *cl {
                    best_before = Some(this);
                }
            }

            match mode {
                ResolveMode::Exact => Err(CuError::from(format!("No exact CL target for {cl}"))),
                ResolveMode::AtOrAfter => {
                    best_after.ok_or_else(|| CuError::from(format!("No CL at/after target {cl}")))
                }
                ResolveMode::AtOrBefore => {
                    best_before.ok_or_else(|| CuError::from(format!("No CL at/before target {cl}")))
                }
            }
        }
        Target::Ts { ts_ns } => {
            let target_time = CuTime::from(*ts_ns);

            let mut exact: Option<ResolvedAt> = None;
            let mut best_after: Option<ResolvedAt> = None;
            let mut best_before: Option<ResolvedAt> = None;

            for idx in 0..total {
                let entry = session
                    .cl_at(idx)?
                    .ok_or_else(|| CuError::from("Corrupt session index"))?;
                let ts = time_of(entry.as_ref()).map(|t| t.as_nanos());
                let this = ResolvedAt {
                    cl: entry.id,
                    ts_ns: ts,
                    idx,
                };

                if let Some(entry_ts) = ts {
                    if entry_ts == *ts_ns {
                        exact = Some(this);
                        break;
                    }
                    if entry_ts > *ts_ns && best_after.is_none() {
                        best_after = Some(this.clone());
                    }
                    if entry_ts < *ts_ns {
                        best_before = Some(this);
                    }
                } else {
                    let _ = target_time;
                }
            }

            if let Some(exact) = exact {
                return Ok(exact);
            }

            match mode {
                ResolveMode::Exact => Err(CuError::from(format!(
                    "No exact timestamp target for {ts_ns}"
                ))),
                ResolveMode::AtOrAfter => best_after
                    .ok_or_else(|| CuError::from(format!("No timestamp at/after {ts_ns}"))),
                ResolveMode::AtOrBefore => best_before
                    .ok_or_else(|| CuError::from(format!("No timestamp at/before {ts_ns}"))),
            }
        }
    }
}

fn copperlist_snapshot<P: CopperListTuple>(
    cl: &crate::copperlist::CopperList<P>,
    time_of: &impl Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime>,
    include_payloads: bool,
    include_metadata: bool,
    include_raw: bool,
) -> CuResult<Value> {
    let task_ids = P::get_all_task_ids();
    let msgs = cl.cumsgs();
    let mut entries = Vec::with_capacity(msgs.len());

    for (i, msg) in msgs.iter().enumerate() {
        let task_id = task_ids.get(i).copied().unwrap_or("<?>");
        let payload = if include_payloads {
            msg.payload()
                .map(erased_serialize_to_json)
                .transpose()?
                .unwrap_or(Value::Null)
        } else {
            Value::Null
        };

        let metadata = if include_metadata {
            metadata_to_json(msg.metadata(), msg.tov())
        } else {
            Value::Null
        };

        entries.push(json!({
            "index": i,
            "task_id": task_id,
            "payload": payload,
            "metadata": metadata,
            "payload_present": msg.payload().is_some(),
        }));
    }

    let raw = if include_raw {
        let encoded = bincode::encode_to_vec(cl, bincode::config::standard())
            .map_err(|e| CuError::new_with_cause("Failed to encode copperlist", e))?;
        Some(hex_string(&encoded))
    } else {
        None
    };

    Ok(json!({
        "cl": cl.id,
        "state": format!("{}", cl.get_state()),
        "ts_ns": time_of(cl).map(|t| t.as_nanos()),
        "messages": entries,
        "raw_bincode_hex": raw,
    }))
}

fn metadata_to_json(metadata: &dyn CuMsgMetadataTrait, tov: Tov) -> Value {
    let process = metadata.process_time();
    let start: Option<CuTime> = process.start.into();
    let end: Option<CuTime> = process.end.into();
    json!({
        "tov": tov_to_json(tov),
        "process_time": {
            "start_ns": start.map(|t| t.as_nanos()),
            "end_ns": end.map(|t| t.as_nanos()),
        },
        "status_txt": metadata.status_txt().0.to_string(),
    })
}

fn tov_to_json(tov: Tov) -> Value {
    match tov {
        Tov::None => json!({"kind": "none"}),
        Tov::Time(t) => json!({"kind": "time", "time_ns": t.as_nanos()}),
        Tov::Range(r) => json!({
            "kind": "range",
            "start_ns": r.start.as_nanos(),
            "end_ns": r.end.as_nanos(),
        }),
    }
}

fn erased_serialize_to_json(value: &dyn erased_serde::Serialize) -> CuResult<Value> {
    let mut bytes = Vec::new();
    {
        let mut serializer = serde_json::Serializer::new(&mut bytes);
        erased_serde::serialize(value, &mut serializer)
            .map_err(|e| CuError::from(format!("Failed to serialize erased payload: {e}")))?;
    }
    serde_json::from_slice(&bytes)
        .map_err(|e| CuError::new_with_cause("Failed to parse serialized payload JSON", e))
}

fn build_state_root_json<App, P, CB, TF, S, L>(
    state: &mut SessionState<App, P, CB, TF, S, L>,
    time_of: &TF,
) -> CuResult<Value>
where
    App: CuSimApplication<S, L> + ReflectTaskIntrospection,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple,
    CB: for<'a> Fn(
        &'a crate::copperlist::CopperList<P>,
        RobotClock,
    )
        -> Box<dyn for<'z> FnMut(App::Step<'z>) -> crate::simulation::SimOverride + 'a>,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
{
    let mut tasks = serde_json::Map::new();

    for task_id in P::get_all_task_ids() {
        if let Ok(task) = state.session.reflected_task(task_id) {
            tasks.insert(task_id.to_string(), reflect_value_to_json(task));
        }
    }

    let current_cl = match state.session.current_cl()? {
        Some(cl) => copperlist_snapshot::<P>(cl.as_ref(), time_of, true, true, false)?,
        None => Value::Null,
    };

    let cursor = cursor_snapshot(state, time_of)
        .map(|c| serde_json::to_value(c).unwrap_or(Value::Null))
        .unwrap_or(Value::Null);

    Ok(json!({
        "tasks": tasks,
        "current_cl": current_cl,
        "cursor": cursor,
    }))
}

fn state_root_for_query<App, P, CB, TF, S, L>(
    state: &mut SessionState<App, P, CB, TF, S, L>,
    at: Option<&At>,
    time_of: &TF,
) -> CuResult<(Value, Option<ResolvedAt>)>
where
    App: CuSimApplication<S, L> + ReflectTaskIntrospection,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple,
    CB: for<'a> Fn(
        &'a crate::copperlist::CopperList<P>,
        RobotClock,
    )
        -> Box<dyn for<'z> FnMut(App::Step<'z>) -> crate::simulation::SimOverride + 'a>,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
{
    if let Some(at) = at {
        let resolved = resolve_target(&mut state.session, &at.target, at.resolve, time_of)?;

        if at.mutate_cursor {
            let jump = seek_to_index(&mut state.session, resolved.idx)?;
            update_after_jump(state, &jump);
            let root = build_state_root_json::<App, P, CB, TF, S, L>(state, time_of)?;
            return Ok((root, Some(resolved)));
        }

        let previous_idx = state.session.current_index();
        let previous = match previous_idx {
            Some(idx) => state.session.cl_at(idx)?,
            None => None,
        };

        if previous.is_none() {
            return Err(CuError::from(
                "Non-mutating at queries require an initialized cursor",
            ));
        }

        let jump = seek_to_index(&mut state.session, resolved.idx)?;
        update_after_jump(state, &jump);

        let root = build_state_root_json::<App, P, CB, TF, S, L>(state, time_of)?;

        if let Some(prev) = previous {
            let _ = state.session.goto_cl(prev.id)?;
            state.bump_rev();
        }

        return Ok((root, Some(resolved)));
    }

    let root = build_state_root_json::<App, P, CB, TF, S, L>(state, time_of)?;
    Ok((root, None))
}

fn navigate_path<'a>(root: &'a Value, path: &str) -> Result<&'a Value, String> {
    let mut current = root;
    for segment in path_segments(path) {
        match current {
            Value::Object(map) => {
                current = map
                    .get(segment)
                    .ok_or_else(|| format!("Path segment '{segment}' not found"))?;
            }
            Value::Array(list) => {
                let idx: usize = segment
                    .parse()
                    .map_err(|_| format!("Array path segment '{segment}' is not an index"))?;
                current = list
                    .get(idx)
                    .ok_or_else(|| format!("Array index {idx} out of bounds"))?;
            }
            _ => {
                return Err(format!(
                    "Cannot descend into non-container at segment '{segment}'"
                ));
            }
        }
    }
    Ok(current)
}

fn path_segments(path: &str) -> Vec<&str> {
    path.trim()
        .trim_start_matches('/')
        .split('/')
        .filter(|s| !s.is_empty())
        .collect()
}

#[derive(Debug)]
struct InspectResult {
    kind: &'static str,
    children: Vec<Value>,
    total_children: usize,
}

fn inspect_value(value: &Value, page: Page) -> InspectResult {
    match value {
        Value::Object(map) => {
            let mut keys: Vec<&String> = map.keys().collect();
            keys.sort();
            let total = keys.len();
            let start = page.offset as usize;
            let end = (start + page.limit as usize).min(total);
            let children = if start < total {
                keys[start..end]
                    .iter()
                    .map(|k| {
                        let v = map.get(*k).unwrap_or(&Value::Null);
                        json!({"key": k, "kind": kind_of(v), "preview": preview_json(v)})
                    })
                    .collect()
            } else {
                Vec::new()
            };
            InspectResult {
                kind: "object",
                children,
                total_children: total,
            }
        }
        Value::Array(list) => {
            let total = list.len();
            let start = page.offset as usize;
            let end = (start + page.limit as usize).min(total);
            let children = if start < total {
                list[start..end]
                    .iter()
                    .enumerate()
                    .map(|(local, v)| {
                        let idx = start + local;
                        json!({"index": idx, "kind": kind_of(v), "preview": preview_json(v)})
                    })
                    .collect()
            } else {
                Vec::new()
            };
            InspectResult {
                kind: "array",
                children,
                total_children: total,
            }
        }
        _ => InspectResult {
            kind: kind_of(value),
            children: Vec::new(),
            total_children: 0,
        },
    }
}

fn apply_page(value: Value, page: Page) -> Value {
    match value {
        Value::Object(map) => {
            let mut keys: Vec<String> = map.keys().cloned().collect();
            keys.sort();
            let start = page.offset as usize;
            let end = (start + page.limit as usize).min(keys.len());
            let mut filtered = serde_json::Map::new();
            if start < keys.len() {
                for key in &keys[start..end] {
                    if let Some(v) = map.get(key) {
                        filtered.insert(key.clone(), v.clone());
                    }
                }
            }
            Value::Object(filtered)
        }
        Value::Array(list) => {
            let start = page.offset as usize;
            let end = (start + page.limit as usize).min(list.len());
            if start < list.len() {
                Value::Array(list[start..end].to_vec())
            } else {
                Value::Array(Vec::new())
            }
        }
        primitive => primitive,
    }
}

fn collect_matches(value: &Value, path: &str, needle: &str, out: &mut Vec<Value>) {
    match value {
        Value::Object(map) => {
            for (k, v) in map {
                let child_path = if path.is_empty() {
                    format!("/{k}")
                } else {
                    format!("{path}/{k}")
                };
                if k.to_ascii_lowercase().contains(needle) {
                    out.push(json!({
                        "path": child_path,
                        "kind": kind_of(v),
                        "preview": preview_json(v),
                    }));
                }
                collect_matches(v, &child_path, needle, out);
            }
        }
        Value::Array(list) => {
            for (i, v) in list.iter().enumerate() {
                let child_path = if path.is_empty() {
                    format!("/{i}")
                } else {
                    format!("{path}/{i}")
                };
                collect_matches(v, &child_path, needle, out);
            }
        }
        _ => {
            let text = value.to_string().to_ascii_lowercase();
            if text.contains(needle) {
                out.push(json!({
                    "path": path,
                    "kind": kind_of(value),
                    "preview": preview_json(value),
                }));
            }
        }
    }
}

fn kind_of(value: &Value) -> &'static str {
    match value {
        Value::Null => "null",
        Value::Bool(_) => "bool",
        Value::Number(_) => "number",
        Value::String(_) => "string",
        Value::Array(_) => "array",
        Value::Object(_) => "object",
    }
}

fn preview_json(value: &Value) -> Value {
    match value {
        Value::Object(map) => {
            let keys: Vec<&String> = map.keys().take(8).collect();
            json!({
                "kind": "object",
                "keys": keys,
                "len": map.len(),
            })
        }
        Value::Array(list) => json!({
            "kind": "array",
            "len": list.len(),
        }),
        primitive => primitive.clone(),
    }
}

fn build_stack_schema<App, S, L>() -> CuResult<Value>
where
    App: ReflectTaskIntrospection + CuSimApplication<S, L>,
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
{
    let config_str = <App as CuSimApplication<S, L>>::get_original_config();
    let config = read_configuration_str(config_str, None)?;
    let graph = config.get_graph(None)?;

    let mut nodes = Vec::new();
    for (node_id, node) in graph.get_all_nodes() {
        let flavor = match node.get_flavor() {
            Flavor::Task => "task",
            Flavor::Bridge => "bridge",
        };
        let task_type = match crate::curuntime::find_task_type_for_id(graph, node_id) {
            crate::curuntime::CuTaskType::Source => "source",
            crate::curuntime::CuTaskType::Regular => "regular",
            crate::curuntime::CuTaskType::Sink => "sink",
        };
        nodes.push(json!({
            "node_id": node_id,
            "id": node.get_id(),
            "type": node.get_type(),
            "flavor": flavor,
            "task_type": task_type,
        }));
    }

    let mut edges = Vec::new();
    for (edge_id, edge) in graph.edges().enumerate() {
        edges.push(json!({
            "edge_id": edge_id,
            "src": edge.src,
            "dst": edge.dst,
            "msg": edge.msg,
            "src_channel": edge.src_channel,
            "dst_channel": edge.dst_channel,
        }));
    }

    let bridges: Vec<Value> = config
        .bridges
        .iter()
        .map(|bridge| {
            let channels: Vec<Value> = bridge
                .channels
                .iter()
                .map(|channel| match channel {
                    BridgeChannelConfigRepresentation::Rx { id, route, .. } => {
                        json!({"direction": "rx", "id": id, "route": route})
                    }
                    BridgeChannelConfigRepresentation::Tx { id, route, .. } => {
                        json!({"direction": "tx", "id": id, "route": route})
                    }
                })
                .collect();
            json!({
                "id": bridge.id,
                "type": bridge.type_,
                "channels": channels,
            })
        })
        .collect();

    let mut registry = TypeRegistry::default();
    <App as ReflectTaskIntrospection>::register_reflect_types(&mut registry);
    let mut types: Vec<String> = registry
        .iter()
        .map(|registration| registration.type_info().type_path().to_string())
        .collect();
    types.sort();
    types.dedup();

    Ok(json!({
        "tasks": nodes,
        "bridges": bridges,
        "edges": edges,
        "message_types": types,
    }))
}

fn simple_jsonschema_for_type(info: &'static TypeInfo) -> Value {
    json!({
        "$schema": "https://json-schema.org/draft-07/schema#",
        "title": info.type_path(),
        "description": format!("{}", info.type_path()),
        "reflect": format!("{info:#?}"),
    })
}

#[cfg(feature = "reflect")]
fn reflect_value_to_json(value: &dyn crate::reflect::Reflect) -> Value {
    let mut registry = TypeRegistry::default();
    let _ = &mut registry;
    let serializer = crate::reflect::serde::ReflectSerializer::new(value, &registry);
    serde_json::to_value(serializer)
        .unwrap_or_else(|_| Value::String("reflect serialization failed".to_string()))
}

#[cfg(not(feature = "reflect"))]
fn reflect_value_to_json(_value: &dyn crate::reflect::Reflect) -> Value {
    Value::String("Reflect feature disabled".to_string())
}

fn from_params<T: serde::de::DeserializeOwned>(params: &Value) -> Result<T, String> {
    serde_json::from_value::<T>(params.clone()).map_err(|e| format!("Could not decode params: {e}"))
}

fn cu_error(msg: &str, error: ZenohError) -> CuError {
    CuError::from(format!("{msg}: {error}"))
}

fn cu_error_map(msg: &str) -> impl FnOnce(ZenohError) -> CuError + '_ {
    move |error| cu_error(msg, error)
}

fn keyexpr(path: &str) -> CuResult<KeyExpr<'static>> {
    KeyExpr::<'static>::new(path.to_string())
        .map_err(|e| CuError::from(format!("Invalid key expression '{path}': {e}")))
}

fn default_schema_format() -> String {
    "jsonschema".to_string()
}

fn default_root_path() -> String {
    "/".to_string()
}

fn default_state_format() -> String {
    "json".to_string()
}

fn now_unix_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

fn hex_string(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len() * 2);
    for b in data {
        out.push(hex_digit(b >> 4));
        out.push(hex_digit(b & 0x0f));
    }
    out
}

fn hex_digit(n: u8) -> char {
    match n {
        0..=9 => (b'0' + n) as char,
        _ => (b'a' + (n - 10)) as char,
    }
}

pub struct RemoteDebugServerHandle {
    stop: Arc<AtomicBool>,
}

impl RemoteDebugServerHandle {
    pub fn stop(&self) {
        self.stop.store(true, Ordering::SeqCst);
    }

    pub fn is_stopped(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }
}
