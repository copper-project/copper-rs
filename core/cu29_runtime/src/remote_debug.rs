//! Remote debug protocol (`debug.v1`) over Zenoh.
//!
//! This module exposes a JSON-RPC-like request/reply protocol over Zenoh pub/sub
//! to control [`CuDebugSession`](crate::debug::CuDebugSession) remotely.
//!
//! ## Entry Points
//!
//! | Kind | Item | Purpose |
//! | --- | --- | --- |
//! | Server | [`RemoteDebugZenohServer::new`] | Build server with local-only default transport (Unix socket + SHM optimization). |
//! | Server | [`RemoteDebugZenohServer::new_with_config`] | Build server with fully custom Zenoh config. |
//! | Server | [`RemoteDebugZenohServer::serve_next`] | Process a single RPC request. |
//! | Server | [`RemoteDebugZenohServer::serve_next_timeout`] | Poll for one RPC request with bounded wait. |
//! | Server | [`RemoteDebugZenohServer::serve_until_stopped`] | Serve loop until `admin.stop`. |
//! | Client | [`RemoteDebugZenohClient::new`] | Client with local-only default transport and CBOR codec. |
//! | Client | [`RemoteDebugZenohClient::builder`] | Builder for selecting codec and/or custom Zenoh config. |
//! | Client | [`RemoteDebugZenohClient::call`] | Send one RPC call and wait for matching response. |
//! | Shared | [`RemoteDebugPaths::new`] | Build request/event topic tree from a base keyexpr prefix. |
//!
//! ## Transport and Topics
//!
//! With `base = "copper/examples/my_app/debug/v1"`, the protocol uses:
//!
//! | Topic | Direction | Payload |
//! | --- | --- | --- |
//! | `{base}/rpc/request` | client -> server | [`DebugRpcRequest`] |
//! | `{base}/rpc/reply/{client_id}` | server -> client | [`DebugRpcResponse`] |
//! | `{base}/events/cursor` | server -> client(s) | event JSON (reserved, currently not emitted by handlers) |
//! | `{base}/events/run` | server -> client(s) | event JSON (reserved, currently not emitted by handlers) |
//! | `{base}/events/watch` | server -> client(s) | watch event JSON (topic exposed, emission helper exists) |
//! | `{base}/events/health` | server -> client(s) | event JSON (reserved, currently not emitted by handlers) |
//!
//! ## Wire Format
//!
//! The wire payload can be CBOR or JSON ([`WireCodec`]):
//!
//! - Request decoding is tolerant: server tries `CBOR` first, then `JSON`.
//! - Response encoding matches the request codec used for that call.
//! - `session.open` also returns a `wire_codec` field after codec negotiation.
//! - Handle-backed payload contents are never embedded in `result`. They are
//!   deferred by default and, when explicitly requested over CBOR, returned as
//!   out-of-line CBOR byte strings in `attachments`.
//! - There is no byte-size threshold: ordinary payloads are always serialized,
//!   while every [`CuHandle`](crate::pool::CuHandle) follows the explicit handle
//!   content policy.
//!
//! Default constructor transport profile:
//!
//! - local Unix socket endpoint derived from `paths.base`
//! - multicast + gossip scouting disabled
//! - on Linux, shared memory is mandatory on both ends
//! - on Linux, server transport optimization uses a 1 GiB pool and a 256 KiB
//!   threshold
//! - on Linux, startup refuses to continue unless `RLIMIT_MEMLOCK` and
//!   `/dev/shm` can support the configured pool
//! - on Linux, large replies are explicitly allocated in SHM; allocation
//!   failure is returned as an RPC error instead of silently copying over the
//!   socket
//! - other platforms retain the existing 4 MiB pool and 3 KiB transport
//!   optimization threshold without Linux-specific preflight
//!
//! ## Envelope Schema
//!
//! ### Request: [`DebugRpcRequest`]
//!
//! | Field | Type | Required | Notes |
//! | --- | --- | --- | --- |
//! | `api` | `string` | yes | Must be `"debug.v1"`. |
//! | `request_id` | `string` | yes | Correlation key echoed in response. |
//! | `session_id` | `string \| null` | no | Required for most methods except `session.open` and `admin.stop`. |
//! | `method` | `string` | yes | RPC method name (`"nav.seek"`, `"state.read"`, ...). |
//! | `params` | `object` | yes | Method-specific params (can be `{}`). |
//! | `reply_to` | `string` | yes | Reply topic keyexpr. |
//!
//! ### Response: [`DebugRpcResponse`]
//!
//! | Field | Type | Present when | Notes |
//! | --- | --- | --- | --- |
//! | `request_id` | `string` | always | Matches request. |
//! | `ok` | `bool` | always | `true` on success, `false` on failure. |
//! | `result` | `json` | `ok == true` | Method result payload. |
//! | `error` | [`DebugRpcError`] | `ok == false` | `{ code, message, details? }`. |
//! | `cursor_rev` | `u64` | cursor-aware methods | Monotonic cursor revision for a session. |
//! | `resolved_at` | [`ResolvedAt`] | target-resolving methods | Concrete `(cl, ts_ns?, idx)` resolution outcome. |
//! | `attachments` | [`DebugRpcAttachment`] array | explicitly requested handle contents | Binary attachment metadata and CBOR byte strings. |
//!
//! A serialized handle in `result` is a small descriptor containing
//! `__cu_handle__`, `attachment_id`, `encoding`, `element_type`, `len_elements`,
//! and `byte_len`. With `include_handle_contents=false` (the default),
//! `attachment_id` is null and no bytes are read or copied. With
//! `include_handle_contents=true`, `attachment_id` references an entry in
//! `attachments`. Primitive buffers use raw little-endian bytes; other
//! handle-backed values use CBOR bytes. Binary attachments require the CBOR wire
//! codec; requesting them over JSON returns `BinaryAttachmentsRequireCbor`.
//!
//! ## Session and Cursor State Model
//!
//! This protocol is stateful per `session_id`.
//!
//! | State | Meaning | Entered by | Exited by |
//! | --- | --- | --- | --- |
//! | `NoSession` | Session id not known to server. | startup / `session.close` | `session.open` |
//! | `OpenCursorUnset` | Session opened, cursor not initialized yet. | `session.open` | first successful nav that positions cursor |
//! | `OpenCursorSet` | Cursor points to a concrete copperlist index. | `nav.seek`, `nav.step`, `nav.run_until`, mutating `timeline.get_cl`, replay, mutating `state.* at` | `session.close` |
//! | `ServerStopping` | Server loop is terminating. | `admin.stop` | process shutdown / loop exit |
//!
//! Cursor revision (`cursor_rev`) behavior:
//!
//! - Increments whenever replay/navigation mutates session cursor via `update_after_jump`.
//! - Non-mutating `state.*` queries with `at` perform temporary seek/restore internally
//!   while preserving `cursor_rev`.
//!
//! Session lifecycle limits (default):
//!
//! - Idle timeout: 15 minutes since last request touching the session.
//! - Maximum active sessions: 64.
//! - Cleanup policy: stale sessions are evicted opportunistically on incoming requests
//!   and on server receive-timeout ticks.
//!
//! ## Target and Resolution Semantics
//!
//! `Target` is either:
//!
//! - `{"kind":"cl","cl":<u32>}`
//! - `{"kind":"ts","ts_ns":<u64>}`
//!
//! `ResolveMode`:
//!
//! | Mode | Meaning |
//! | --- | --- |
//! | `exact` | Require exact match. |
//! | `at_or_after` | First entry with value >= target. |
//! | `at_or_before` | Last entry with value <= target. |
//!
//! Resolution always yields a concrete [`ResolvedAt`] (`cl`, optional `ts_ns`, and backing `idx`).
//!
//! ## RPC Method Matrix
//!
//! `admin.stop` is handled specially before normal dispatch.
//!
//! ### Session/Admin
//!
//! | Method | Session required | Params | Result (key fields) |
//! | --- | --- | --- | --- |
//! | `session.open` | no | [`SessionOpenParams`] | `session_id`, `capabilities`, `wire_codec`, `initial_cursor` |
//! | `session.close` | yes | `{}` | `closed: bool` |
//! | `session.capabilities` | yes | `{}` | protocol capability descriptor |
//! | `session.cancel` | yes | `{ op_id }` | `cancelled` (currently always `false`, placeholder) |
//! | `admin.stop` | no | `{}` | `stopping: true` |
//!
//! ### Navigation
//!
//! | Method | Session required | Params | Cursor mutation |
//! | --- | --- | --- | --- |
//! | `nav.seek` | yes | `{ target, resolve? }` | yes |
//! | `nav.run_until` | yes | `{ target, resolve?, max_steps?, timeout_ms?, progress_every_n_steps? }` | yes |
//! | `nav.step` | yes | `{ delta }` | yes |
//! | `nav.replay` | yes | `{ at?, limit?, include_cl_snapshot?, include_payloads?, include_handle_contents?, output_indices?, include_metadata?, include_raw?, include_replayed_cl?, include_state? }` | yes (replays current step/page; optional pre-seek) |
//!
//! Notes:
//!
//! - Current `nav.run_until` is synchronous and implemented as a resolved seek/jump,
//!   returning an `op_id` for protocol shape consistency.
//! - Current `session.cancel` does not cancel active work yet (reserved behavior).
//!
//! ### Timeline
//!
//! | Method | Session required | Params | Result |
//! | --- | --- | --- | --- |
//! | `timeline.get_cursor` | yes | `{}` | current cursor snapshot |
//! | `timeline.get_cl` | yes | `{ at?, include_payloads?, include_handle_contents?, output_indices?, include_metadata?, include_raw? }` | `cl_snapshot`, `query_cursor` (+ optional resolution) |
//! | `timeline.list` | yes | `{ from, to, page?, include_handle_contents?, output_indices? }` | list of timeline rows with CL snapshots and keyframe metadata + `next_offset` |
//!
//! ### Logs
//!
//! | Method | Session required | Params | Result |
//! | --- | --- | --- | --- |
//! | `logs.strings` | yes | `{}` | interned string table and resolved index path |
//! | `logs.list` | yes | `{ source?, offset?, limit? }` | structured log entries + `next_offset` |
//!
//! ### Schema
//!
//! | Method | Session required | Params | Result |
//! | --- | --- | --- | --- |
//! | `schema.get_stack` | yes | `{}` | stack config schema JSON |
//! | `schema.list_types` | yes | `{ filter? }` | `type_paths[]` |
//! | `schema.get_type` | yes | `{ type_path, format? }` | schema/reflect dump |
//! | `schema.get_outputs` | yes | `{ page?: { offset, limit }, field_page?: { offset, limit } }` | paged output index -> task/message/payload field metadata |
//!
//! ### State
//!
//! | Method | Session required | Params | Result |
//! | --- | --- | --- | --- |
//! | `state.inspect` | yes | `{ path="/", at?, depth?, page? }` | node kind + children preview + `query_cursor` |
//! | `state.read` | yes | `{ path="/", at?, depth?, page?, format="json" }` | JSON value slice + `query_cursor` |
//! | `state.search` | yes | `{ query, at?, page? }` | path/kind/preview matches + `query_cursor` |
//! | `state.watch.open` | yes | `{ path, at?, mode? }` | `watch_id`, `event_topic` |
//! | `state.watch.close` | yes | `{ watch_id }` | `closed: bool` |
//!
//! ### Health
//!
//! | Method | Session required | Params | Result |
//! | --- | --- | --- | --- |
//! | `health.ping` | no (or valid session if provided) | `{}` | `now_ns`, `status` |
//! | `health.stats` | yes | `{}` | uptime, replay perf, watch count, cursor revision |
//!
//! ## Expected Call Flows
//!
//! Minimal control flow:
//!
//! 1. `session.open` -> get `session_id`
//! 2. `session.capabilities`
//! 3. one or more `nav.*` / `timeline.*` / `schema.*` / `state.*` / `health.*`
//! 4. `session.close`
//! 5. `admin.stop` (if caller owns server lifecycle)
//!
//! Example replay-focused flow:
//!
//! 1. `session.open`
//! 2. `nav.seek` to a known CL
//! 3. `timeline.get_cl` with payloads/metadata
//! 4. `state.inspect` or `state.read` at same/derived target
//! 5. `nav.step` or `nav.replay`
//! 6. `health.stats`
//! 7. `session.close`
//!
//! ## Error Model
//!
//! Errors are returned as:
//!
//! - `ok = false`
//! - `error.code` + `error.message`
//!
//! Common codes include:
//!
//! | Code | Typical cause |
//! | --- | --- |
//! | `InvalidApi` | `api` field is not `"debug.v1"`. |
//! | `UnknownMethod` | Method not in dispatch table. |
//! | `MissingSession` | Session-required call omitted `session_id`. |
//! | `SessionNotFound` | Unknown session id. |
//! | `SessionLimitReached` | `session.open` denied because active session cap is reached. |
//! | `InvalidParams` | JSON decode error for method params. |
//! | `ResolveFailed` | Target could not be resolved with requested mode. |
//! | `SeekFailed` / `StepFailed` / `ReplayFailed` / `RunUntilFailed` | Navigation/replay operation failed. |
//! | `GetClFailed` / `TimelineListFailed` / `CursorFailed` | Timeline query failure. |
//! | `StateFailed` / `PathNotFound` | State-tree query failure. |
//! | `TypeNotFound` | `schema.get_type` requested unknown type path. |
//!
//! ## Current Implementation Notes
//!
//! - Protocol version constant is `debug.v1`.
//! - Method capabilities reported by `session.capabilities` do not list `admin.stop`
//!   even though server handles it explicitly.
//! - `session.capabilities` and `session.open` expose lifecycle limits under
//!   `session_lifecycle` in the capability payload.
//! - Event publishers are declared and helper methods exist, but core handlers are
//!   currently request/reply oriented and do not emit continuous event streams yet.

use crate::app::{CuSimApplication, CurrentRuntimeCopperList};
use crate::config::{BridgeChannelConfigRepresentation, Flavor, read_configuration_str};
use crate::debug::{CuDebugSession, IndexedResolveMode, JumpOutcome};
use crate::pool::{
    CollectedDebugHandleAttachment, CuSharedMemoryElementType, DebugHandleEncoding,
    collect_debug_handle_attachments, with_debug_handle_contents,
};
use crate::reflect::{
    EnumInfo, Reflect, ReflectTaskIntrospection, StructInfo, TupleInfo, TupleStructInfo, Type,
    TypeInfo, TypeRegistry, VariantInfo, serde::SerializationData,
};
use bincode::config::standard;
use bincode::decode_from_std_read;
use bincode::error::DecodeError;
use cu29_clock::{
    CuTime, CuTimeRange, OptionCuTime, PartialCuTimeRange, RobotClock, RobotClockMock, Tov,
};
use cu29_log::CuLogEntry;
use cu29_log_runtime::capture_live_logs;
use cu29_traits::{
    CopperListTuple, CuCompactString, CuError, CuMsgMetadataTrait, CuMsgOrigin, CuResult,
    DebugEnumVariantDescriptor, DebugEnumVariantKind, DebugFieldDescriptor, DebugFieldKind,
    DebugFieldSemantics, DebugScalarKind, ErasedCuStampedDataSet, UnifiedLogType,
};
use cu29_unifiedlog::{
    SectionStorage, UnifiedLogWrite, UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader,
};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value, json};
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fs;
use std::hash::{Hash, Hasher};
use std::io;
use std::path::{Path, PathBuf};
#[cfg(target_os = "linux")]
use std::sync::Arc;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use zenoh::bytes::Encoding;
use zenoh::key_expr::KeyExpr;
#[cfg(target_os = "linux")]
use zenoh::shm::{PosixShmProviderBackend, ShmProvider, ShmProviderState, ZShmMut};
use zenoh::{Config as ZenohConfig, Error as ZenohError};

const API_VERSION: &str = "debug.v1";
const DEFAULT_SESSION_IDLE_TIMEOUT: Duration = Duration::from_secs(15 * 60);
const DEFAULT_MAX_ACTIVE_SESSIONS: usize = 64;
const MAX_PAGE_LIMIT: u32 = 1000;
const MAX_REPLAY_BATCH_LIMIT: u32 = 128;
const MAX_LOG_PAGE_LIMIT: usize = 5000;
const SERVER_RECV_POLL_TIMEOUT: Duration = Duration::from_millis(250);
#[cfg(target_os = "linux")]
const SHM_PROVIDER_INIT_TIMEOUT: Duration = Duration::from_secs(10);
#[cfg(target_os = "linux")]
const SHM_PROVIDER_INIT_POLL_INTERVAL: Duration = Duration::from_millis(10);
#[cfg(target_os = "linux")]
const SHM_REPLY_ALLOC_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(target_os = "linux")]
const SHM_REPLY_ALLOC_POLL_INTERVAL: Duration = Duration::from_millis(1);
#[cfg(any(target_os = "linux", test))]
const REMOTE_DEBUG_SHM_MEMLOCK_OVERHEAD_BYTES: usize = 2 * 1024 * 1024;
#[cfg(any(target_os = "linux", test))]
const REMOTE_DEBUG_SHM_UNLIMITED: u64 = u64::MAX;

/// Shared-memory requirements for the local remote-debug transport.
///
/// On Linux, the default reserves a 1 GiB Zenoh pool for large TimeTraveler
/// replies. Both processes need enough `RLIMIT_MEMLOCK` to map that pool. The
/// client does not create a second transport-optimization pool; it only
/// advertises SHM support and maps the server's pool.
///
/// Other platforms retain the previous 4 MiB pool and 3 KiB optimization
/// threshold without Linux-specific resource preflight.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RemoteDebugShmConfig {
    pub pool_size_bytes: usize,
    pub message_size_threshold_bytes: usize,
}

impl RemoteDebugShmConfig {
    #[cfg(target_os = "linux")]
    pub const DEFAULT_POOL_SIZE_BYTES: usize = 1024 * 1024 * 1024;
    #[cfg(not(target_os = "linux"))]
    pub const DEFAULT_POOL_SIZE_BYTES: usize = 4 * 1024 * 1024;
    #[cfg(target_os = "linux")]
    pub const DEFAULT_MESSAGE_SIZE_THRESHOLD_BYTES: usize = 256 * 1024;
    #[cfg(not(target_os = "linux"))]
    pub const DEFAULT_MESSAGE_SIZE_THRESHOLD_BYTES: usize = 3 * 1024;

    pub const fn new(pool_size_bytes: usize, message_size_threshold_bytes: usize) -> Self {
        Self {
            pool_size_bytes,
            message_size_threshold_bytes,
        }
    }

    fn validate(self) -> CuResult<Self> {
        if self.pool_size_bytes == 0 {
            return Err(CuError::from(
                "RemoteDebug shared-memory configuration requires a non-zero pool",
            ));
        }
        if self.message_size_threshold_bytes == 0 {
            return Err(CuError::from(
                "RemoteDebug shared-memory configuration requires a non-zero message threshold",
            ));
        }
        if self.message_size_threshold_bytes > self.pool_size_bytes {
            return Err(CuError::from(format!(
                "RemoteDebug shared-memory threshold ({}) exceeds its pool size ({})",
                format_bytes(self.message_size_threshold_bytes),
                format_bytes(self.pool_size_bytes),
            )));
        }
        Ok(self)
    }

    #[cfg(any(target_os = "linux", test))]
    fn required_memlock_bytes(self) -> CuResult<usize> {
        self.pool_size_bytes
            .checked_add(REMOTE_DEBUG_SHM_MEMLOCK_OVERHEAD_BYTES)
            .ok_or_else(|| CuError::from("RemoteDebug shared-memory size overflow"))
    }
}

impl Default for RemoteDebugShmConfig {
    fn default() -> Self {
        Self::new(
            Self::DEFAULT_POOL_SIZE_BYTES,
            Self::DEFAULT_MESSAGE_SIZE_THRESHOLD_BYTES,
        )
    }
}

#[cfg(target_os = "linux")]
type RemoteDebugShmProvider = ShmProvider<PosixShmProviderBackend>;

#[derive(Clone, Debug, Serialize, Reflect)]
struct DebugMessageMetadataView {
    tov: Tov,
    process_time: PartialCuTimeRange,
    status_txt: String,
    origin: Option<CuMsgOrigin>,
}

fn register_debug_support_types(registry: &mut TypeRegistry) {
    registry.register::<DebugMessageMetadataView>();
    registry.register::<crate::cutask::CuMsgMetadata>();
    registry.register::<Tov>();
    registry.register::<CuTimeRange>();
    registry.register::<PartialCuTimeRange>();
    registry.register::<OptionCuTime>();
    registry.register::<CuTime>();
    registry.register::<CuMsgOrigin>();
    registry.register::<Option<CuMsgOrigin>>();
}

fn populate_debug_type_registry<App>(registry: &mut TypeRegistry)
where
    App: ReflectTaskIntrospection,
{
    <App as ReflectTaskIntrospection>::register_reflect_types(registry);
    register_debug_support_types(registry);
}

fn append_builtin_debug_type_paths(paths: &mut Vec<String>) {
    let compact_string = core::any::type_name::<CuCompactString>().to_string();
    if !paths.iter().any(|path| path == &compact_string) {
        paths.push(compact_string);
    }
}

fn builtin_debug_type_schema(type_path: &str, format: &str) -> Option<Value> {
    (type_path == core::any::type_name::<CuCompactString>()).then(|| {
        if format == "jsonschema" {
            json!({
                "$schema": "https://json-schema.org/draft-07/schema#",
                "title": type_path,
                "description": "Copper compact status string wrapper",
                "type": "string",
            })
        } else {
            json!({
                "type_path": type_path,
                "reflect_dump": "opaque scalar string wrapper",
            })
        }
    })
}
const REGISTRY_DIR_NAME: &str = "cu29_remote_debug_registry";

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
    pub cl: u64,
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
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub attachments: Vec<DebugRpcAttachment>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebugRpcAttachment {
    pub id: u32,
    pub encoding: DebugHandleEncoding,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub element_type: Option<CuSharedMemoryElementType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub len_elements: Option<usize>,
    #[serde(with = "serde_bytes")]
    pub data: Vec<u8>,
}

impl From<CollectedDebugHandleAttachment> for DebugRpcAttachment {
    fn from(attachment: CollectedDebugHandleAttachment) -> Self {
        Self {
            id: attachment.id,
            encoding: attachment.encoding,
            element_type: attachment.element_type,
            len_elements: attachment.len_elements,
            data: attachment.data,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Target {
    Cl { cl: u64 },
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

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct LogsListParams {
    #[serde(default)]
    offset: usize,
    #[serde(default)]
    limit: Option<usize>,
    #[serde(default)]
    source: LogsSource,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum LogsSource {
    #[default]
    Recorded,
    Replay,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RemoteDebugProcessInfo {
    pub pid: u32,
    pub debug_base: String,
    pub executable_path: PathBuf,
    pub working_dir: PathBuf,
    pub argv: Vec<String>,
    #[serde(default)]
    pub restart_env: BTreeMap<String, String>,
    #[serde(default)]
    pub log_base: Option<PathBuf>,
    pub registered_at_unix_ns: u64,
}

impl RemoteDebugProcessInfo {
    pub fn display_name(&self) -> String {
        self.executable_path
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or("target")
            .to_owned()
    }

    pub fn command_args(&self) -> &[String] {
        self.argv.get(1..).unwrap_or(&[])
    }
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
    #[serde(default = "default_nav_replay_limit")]
    limit: u32,
    #[serde(default)]
    include_cl_snapshot: bool,
    #[serde(default)]
    include_payloads: bool,
    #[serde(default)]
    include_handle_contents: bool,
    #[serde(default)]
    output_indices: Option<Vec<usize>>,
    #[serde(default)]
    include_metadata: bool,
    #[serde(default)]
    include_raw: bool,
    #[serde(default)]
    include_replayed_cl: bool,
    #[serde(default)]
    include_state: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct TimelineGetClParams {
    #[serde(default)]
    at: Option<At>,
    #[serde(default)]
    include_payloads: bool,
    #[serde(default)]
    include_handle_contents: bool,
    #[serde(default)]
    output_indices: Option<Vec<usize>>,
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
    #[serde(default)]
    include_handle_contents: bool,
    #[serde(default)]
    output_indices: Option<Vec<usize>>,
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

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[allow(dead_code)]
struct ProcessDescribeParams {}

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

fn validated_page(page: Page) -> Result<Page, String> {
    if page.limit > MAX_PAGE_LIMIT {
        return Err(format!(
            "page.limit must be <= {MAX_PAGE_LIMIT}, got {}",
            page.limit
        ));
    }
    Ok(page)
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
    cl: u64,
    idx: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    ts_ns: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    keyframe_cl: Option<u64>,
    replayed: usize,
    rev: u64,
}

#[derive(Debug, Clone, Serialize)]
struct QueryCursorSnapshot {
    cl: u64,
    idx: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    ts_ns: Option<u64>,
    is_keyframe: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    keyframe_cl: Option<u64>,
}

struct SessionState<App, P, CB, TF, S, L>
where
    P: CopperListTuple + 'static,
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
{
    session: CuDebugSession<App, P, CB, TF, S, L>,
    log_base: PathBuf,
    log_index_path: Option<PathBuf>,
    interned_strings: Option<Vec<String>>,
    replay_structured_logs: Vec<CuLogEntry>,
    opened_at: Instant,
    last_touched_at: Instant,
    cursor_rev: u64,
    last_keyframe: Option<u64>,
    last_replayed: usize,
    watches: HashMap<u64, StateWatch>,
    #[allow(dead_code)]
    wire_codec: WireCodec,
}

impl<App, P, CB, TF, S, L> SessionState<App, P, CB, TF, S, L>
where
    P: CopperListTuple + 'static,
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
{
    fn bump_rev(&mut self) {
        self.cursor_rev = self.cursor_rev.saturating_add(1);
    }
}

#[derive(Debug, Clone, Copy)]
struct SessionLifecycleLimits {
    idle_timeout: Duration,
    max_sessions: usize,
}

impl Default for SessionLifecycleLimits {
    fn default() -> Self {
        Self {
            idle_timeout: DEFAULT_SESSION_IDLE_TIMEOUT,
            max_sessions: DEFAULT_MAX_ACTIVE_SESSIONS,
        }
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

fn local_socket_path(base: &str) -> PathBuf {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    base.hash(&mut hasher);
    let id = hasher.finish();
    PathBuf::from(format!("/tmp/cu29_remote_debug_{id:016x}.sock"))
}

fn local_endpoint(base: &str) -> String {
    format!("unixsock-stream/{}", local_socket_path(base).display())
}

pub fn remote_debug_registry_dir() -> PathBuf {
    std::env::var_os("XDG_RUNTIME_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir)
        .join(REGISTRY_DIR_NAME)
}

pub fn remove_remote_debug_registry_entry(debug_base: &str) -> std::io::Result<()> {
    let path = remote_debug_registry_entry_path(debug_base);
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(err) => Err(err),
    }
}

pub fn load_remote_debug_registry_entries() -> CuResult<Vec<RemoteDebugProcessInfo>> {
    let dir = remote_debug_registry_dir();
    let entries = match fs::read_dir(&dir) {
        Ok(entries) => entries,
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(err) => {
            return Err(CuError::from(format!(
                "RemoteDebug: failed to read registry dir '{}': {err}",
                dir.display()
            )));
        }
    };

    let mut processes = Vec::new();
    for entry in entries {
        let Ok(entry) = entry else {
            continue;
        };
        let path = entry.path();
        if path.extension().and_then(|ext| ext.to_str()) != Some("json") {
            continue;
        }
        let Ok(contents) = fs::read_to_string(&path) else {
            continue;
        };
        let Ok(process) = serde_json::from_str::<RemoteDebugProcessInfo>(&contents) else {
            continue;
        };
        processes.push(process);
    }

    Ok(processes)
}

fn remote_debug_registry_entry_path(debug_base: &str) -> PathBuf {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    debug_base.hash(&mut hasher);
    let id = hasher.finish();
    remote_debug_registry_dir().join(format!("{id:016x}.json"))
}

fn capture_current_process_info(default_debug_base: &str) -> RemoteDebugProcessInfo {
    let argv = std::env::args_os()
        .map(|arg| arg.to_string_lossy().into_owned())
        .collect::<Vec<_>>();
    let executable_path = std::env::current_exe().unwrap_or_else(|_| {
        argv.first()
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("unknown"))
    });
    let working_dir = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    let debug_base =
        extract_flag_value(&argv, "--debug-base").unwrap_or_else(|| default_debug_base.to_owned());
    let log_base = extract_flag_value(&argv, "--log-base")
        .map(PathBuf::from)
        .map(|path| {
            if path.is_absolute() {
                path
            } else {
                working_dir.join(path)
            }
        })
        .map(normalize_remote_debug_log_base_path);

    RemoteDebugProcessInfo {
        pid: std::process::id(),
        debug_base,
        executable_path,
        working_dir,
        argv,
        restart_env: collect_restart_env(),
        log_base,
        registered_at_unix_ns: now_unix_ns(),
    }
}

fn extract_flag_value(args: &[String], flag: &str) -> Option<String> {
    for (idx, arg) in args.iter().enumerate() {
        if arg == flag {
            return args.get(idx + 1).cloned();
        }
        if let Some((prefix, value)) = arg.split_once('=')
            && prefix == flag
        {
            return Some(value.to_owned());
        }
    }
    None
}

fn normalize_remote_debug_log_base_path(path: PathBuf) -> PathBuf {
    let Some(extension) = path.extension().and_then(|ext| ext.to_str()) else {
        return path;
    };
    let Some(stem) = path.file_stem().and_then(|stem| stem.to_str()) else {
        return path;
    };
    let Some((base_stem, slab_suffix)) = stem.rsplit_once('_') else {
        return path;
    };

    if slab_suffix.is_empty() || !slab_suffix.chars().all(|c| c.is_ascii_digit()) {
        return path;
    }

    let normalized = path.with_file_name(format!("{base_stem}.{extension}"));
    let Some(normalized_stem) = normalized.file_stem().and_then(|stem| stem.to_str()) else {
        return path;
    };
    let Some(normalized_extension) = normalized.extension().and_then(|ext| ext.to_str()) else {
        return path;
    };
    let slab_zero =
        normalized.with_file_name(format!("{normalized_stem}_0.{normalized_extension}"));
    if slab_zero.exists() { normalized } else { path }
}

fn collect_restart_env() -> BTreeMap<String, String> {
    std::env::vars()
        .filter(|(key, _)| should_capture_restart_env(key))
        .collect()
}

fn should_capture_restart_env(key: &str) -> bool {
    matches!(
        key,
        "DYLD_FALLBACK_LIBRARY_PATH"
            | "DYLD_LIBRARY_PATH"
            | "LD_LIBRARY_PATH"
            | "PATH"
            | "PYTHONPATH"
            | "LOG_INDEX_DIR"
            | "RUST_BACKTRACE"
            | "RUST_LOG"
    ) || key.starts_with("COPPER_")
        || key.starts_with("ZENOH_")
}

struct RemoteDebugRegistryRegistration {
    entry_path: PathBuf,
}

impl RemoteDebugRegistryRegistration {
    fn register(process: &RemoteDebugProcessInfo) -> std::io::Result<Self> {
        let entry_path = remote_debug_registry_entry_path(&process.debug_base);
        if let Some(parent) = entry_path.parent() {
            fs::create_dir_all(parent)?;
        }

        let temp_path = entry_path.with_extension("json.tmp");
        let contents = serde_json::to_vec_pretty(process)
            .map_err(|err| std::io::Error::other(err.to_string()))?;
        fs::write(&temp_path, contents)?;
        fs::rename(temp_path, &entry_path)?;

        Ok(Self { entry_path })
    }
}

impl Drop for RemoteDebugRegistryRegistration {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.entry_path);
    }
}

fn set_config_json5(config: &mut ZenohConfig, key: &str, value: &str) -> CuResult<()> {
    config
        .insert_json5(key, value)
        .map_err(|e| CuError::from(format!("RemoteDebug: invalid zenoh config '{key}': {e}")))
}

#[derive(Debug, Clone, Copy)]
enum RemoteDebugShmRole {
    Server,
    Client,
}

impl RemoteDebugShmRole {
    #[cfg(any(target_os = "linux", test))]
    const fn label(self) -> &'static str {
        match self {
            Self::Server => "server",
            Self::Client => "client",
        }
    }
}

#[cfg(any(target_os = "linux", test))]
#[derive(Debug, Clone, Copy)]
struct RemoteDebugShmSystemLimits {
    memlock_soft_bytes: u64,
    memlock_hard_bytes: u64,
    shm_available_bytes: u64,
}

fn format_bytes(bytes: usize) -> String {
    const KIB: f64 = 1024.0;
    const MIB: f64 = KIB * 1024.0;
    const GIB: f64 = MIB * 1024.0;
    let bytes = bytes as f64;
    if bytes >= GIB {
        format!("{:.2} GiB", bytes / GIB)
    } else if bytes >= MIB {
        format!("{:.2} MiB", bytes / MIB)
    } else if bytes >= KIB {
        format!("{:.2} KiB", bytes / KIB)
    } else {
        format!("{bytes:.0} B")
    }
}

#[cfg(any(target_os = "linux", test))]
fn format_limit_bytes(bytes: u64) -> String {
    if bytes == REMOTE_DEBUG_SHM_UNLIMITED {
        "unlimited".to_owned()
    } else {
        usize::try_from(bytes)
            .map(format_bytes)
            .unwrap_or_else(|_| format!("{bytes} bytes"))
    }
}

#[cfg(target_os = "linux")]
fn remote_debug_shm_system_limits() -> CuResult<RemoteDebugShmSystemLimits> {
    let mut memlock = libc::rlimit {
        rlim_cur: 0,
        rlim_max: 0,
    };
    // SAFETY: `memlock` points to valid writable storage for getrlimit.
    if unsafe { libc::getrlimit(libc::RLIMIT_MEMLOCK, &mut memlock) } != 0 {
        return Err(CuError::new_with_cause(
            "RemoteDebug shared-memory preflight could not read RLIMIT_MEMLOCK",
            io::Error::last_os_error(),
        ));
    }

    let shm_path = b"/dev/shm\0";
    let mut stats = std::mem::MaybeUninit::<libc::statvfs>::uninit();
    // SAFETY: the path is NUL-terminated and `stats` points to writable storage.
    if unsafe { libc::statvfs(shm_path.as_ptr().cast::<libc::c_char>(), stats.as_mut_ptr()) } != 0 {
        return Err(CuError::new_with_cause(
            "RemoteDebug shared-memory preflight could not inspect /dev/shm",
            io::Error::last_os_error(),
        ));
    }
    // SAFETY: statvfs succeeded and initialized `stats`.
    let stats = unsafe { stats.assume_init() };
    let shm_available_bytes = stats.f_bavail.saturating_mul(stats.f_frsize);

    Ok(RemoteDebugShmSystemLimits {
        memlock_soft_bytes: memlock.rlim_cur,
        memlock_hard_bytes: memlock.rlim_max,
        shm_available_bytes,
    })
}

#[cfg(any(target_os = "linux", test))]
fn validate_remote_debug_shm_limits(
    shm: RemoteDebugShmConfig,
    role: RemoteDebugShmRole,
    limits: RemoteDebugShmSystemLimits,
) -> CuResult<()> {
    let shm = shm.validate()?;
    let required = shm.required_memlock_bytes()?;
    let required_u64 = u64::try_from(required).map_err(|_| {
        CuError::from("RemoteDebug shared-memory size is unsupported on this target")
    })?;
    let memlock_ok = limits.memlock_soft_bytes == REMOTE_DEBUG_SHM_UNLIMITED
        || limits.memlock_soft_bytes >= required_u64;
    let shm_ok = limits.shm_available_bytes >= shm.pool_size_bytes as u64;
    if memlock_ok && shm_ok {
        return Ok(());
    }

    let recommended_kib = 2 * 1024 * 1024;
    let mut failures = Vec::new();
    if !memlock_ok {
        failures.push(format!(
            "RLIMIT_MEMLOCK soft/hard is {}/{}; this profile requires at least {} \
             ({} pool + {} Zenoh metadata headroom)",
            format_limit_bytes(limits.memlock_soft_bytes),
            format_limit_bytes(limits.memlock_hard_bytes),
            format_bytes(required),
            format_bytes(shm.pool_size_bytes),
            format_bytes(REMOTE_DEBUG_SHM_MEMLOCK_OVERHEAD_BYTES),
        ));
    }
    if !shm_ok {
        failures.push(format!(
            "/dev/shm has {} available; this profile requires at least {}",
            format_limit_bytes(limits.shm_available_bytes),
            format_bytes(shm.pool_size_bytes),
        ));
    }

    Err(CuError::from(format!(
        "RemoteDebug shared-memory preflight failed for the {}: {}. \
         Refusing to start because Zenoh can otherwise silently copy large TimeTraveler \
         replies over the socket. Configure BOTH the Copper server and TimeTraveler process \
         before launch. Recommended interactive-shell setting: \
         `sudo prlimit --pid $$ --memlock=2147483648:2147483648`, then verify \
         `ulimit -l {recommended_kib}` (2 GiB, value is KiB). A plain `ulimit` cannot exceed \
         the current hard limit. For systemd use `LimitMEMLOCK=2G`; for Docker also use \
         `--ulimit memlock=2147483648:2147483648 --shm-size=2g`. Verify with `ulimit -l` \
         and `df -h /dev/shm`.",
        role.label(),
        failures.join("; "),
    )))
}

/// Validate the host resources required by the default local remote-debug SHM
/// transport before opening Zenoh.
///
/// This performs only local configuration/resource checks; it sends no probe
/// message and does not require the other endpoint to be running.
pub fn remote_debug_shm_preflight(shm: RemoteDebugShmConfig) -> CuResult<()> {
    #[cfg(not(target_os = "linux"))]
    {
        shm.validate()?;
        Ok(())
    }

    #[cfg(target_os = "linux")]
    validate_remote_debug_shm_limits(
        shm,
        RemoteDebugShmRole::Client,
        remote_debug_shm_system_limits()?,
    )
}

fn preflight_remote_debug_shm(shm: RemoteDebugShmConfig, role: RemoteDebugShmRole) -> CuResult<()> {
    #[cfg(not(target_os = "linux"))]
    {
        let _ = role;
        shm.validate()?;
        Ok(())
    }

    #[cfg(target_os = "linux")]
    validate_remote_debug_shm_limits(shm, role, remote_debug_shm_system_limits()?)
}

fn validate_server_zenoh_shm_config(config: &ZenohConfig) -> CuResult<RemoteDebugShmConfig> {
    let shared_memory = &config.transport.shared_memory;
    if !*shared_memory.enabled() {
        return Err(CuError::from(
            "RemoteDebug server requires Zenoh transport/shared_memory/enabled=true",
        ));
    }
    if !*shared_memory.transport_optimization.enabled() {
        return Err(CuError::from(
            "RemoteDebug server requires Zenoh shared-memory transport optimization",
        ));
    }
    RemoteDebugShmConfig::new(
        shared_memory.transport_optimization.pool_size().get(),
        *shared_memory
            .transport_optimization
            .message_size_threshold(),
    )
    .validate()
}

fn validate_client_zenoh_shm_config(config: &ZenohConfig) -> CuResult<()> {
    if !*config.transport.shared_memory.enabled() {
        return Err(CuError::from(
            "RemoteDebug client requires Zenoh transport/shared_memory/enabled=true",
        ));
    }
    Ok(())
}

fn local_server_zenoh_config(
    paths: &RemoteDebugPaths,
    shm: RemoteDebugShmConfig,
) -> CuResult<ZenohConfig> {
    let shm = shm.validate()?;
    let endpoint = local_endpoint(&paths.base);
    let endpoint_json = serde_json::to_string(&endpoint)
        .map_err(|e| CuError::new_with_cause("RemoteDebug: endpoint encoding failed", e))?;

    let mut config = ZenohConfig::default();
    set_config_json5(&mut config, "mode", "\"peer\"")?;
    set_config_json5(&mut config, "scouting/multicast/enabled", "false")?;
    set_config_json5(&mut config, "scouting/gossip/enabled", "false")?;
    set_config_json5(&mut config, "connect/endpoints", "[]")?;
    set_config_json5(
        &mut config,
        "listen/endpoints",
        &format!("[{endpoint_json}]"),
    )?;
    set_config_json5(&mut config, "transport/shared_memory/enabled", "true")?;
    #[cfg(target_os = "linux")]
    set_config_json5(&mut config, "transport/shared_memory/mode", "\"init\"")?;
    set_config_json5(
        &mut config,
        "transport/shared_memory/transport_optimization/enabled",
        "true",
    )?;
    set_config_json5(
        &mut config,
        "transport/shared_memory/transport_optimization/pool_size",
        &shm.pool_size_bytes.to_string(),
    )?;
    set_config_json5(
        &mut config,
        "transport/shared_memory/transport_optimization/message_size_threshold",
        &shm.message_size_threshold_bytes.to_string(),
    )?;
    Ok(config)
}

fn local_client_zenoh_config(
    paths: &RemoteDebugPaths,
    shm: RemoteDebugShmConfig,
) -> CuResult<ZenohConfig> {
    let shm = shm.validate()?;
    let endpoint = local_endpoint(&paths.base);
    let endpoint_json = serde_json::to_string(&endpoint)
        .map_err(|e| CuError::new_with_cause("RemoteDebug: endpoint encoding failed", e))?;

    let mut config = ZenohConfig::default();
    set_config_json5(&mut config, "mode", "\"client\"")?;
    set_config_json5(&mut config, "scouting/multicast/enabled", "false")?;
    set_config_json5(&mut config, "scouting/gossip/enabled", "false")?;
    set_config_json5(&mut config, "listen/endpoints", "[]")?;
    set_config_json5(
        &mut config,
        "connect/endpoints",
        &format!("[{endpoint_json}]"),
    )?;
    set_config_json5(&mut config, "transport/shared_memory/enabled", "true")?;
    #[cfg(target_os = "linux")]
    set_config_json5(&mut config, "transport/shared_memory/mode", "\"init\"")?;
    // The client maps server-owned reply buffers but does not need a second
    // 1 GiB provider for its small control requests.
    set_config_json5(
        &mut config,
        "transport/shared_memory/transport_optimization/enabled",
        if cfg!(target_os = "linux") {
            "false"
        } else {
            "true"
        },
    )?;
    set_config_json5(
        &mut config,
        "transport/shared_memory/transport_optimization/pool_size",
        &shm.pool_size_bytes.to_string(),
    )?;
    set_config_json5(
        &mut config,
        "transport/shared_memory/transport_optimization/message_size_threshold",
        &shm.message_size_threshold_bytes.to_string(),
    )?;
    Ok(config)
}

#[cfg(target_os = "linux")]
fn initialize_remote_debug_shm_provider(
    session: &zenoh::Session,
) -> CuResult<Arc<RemoteDebugShmProvider>> {
    let started = Instant::now();
    loop {
        match session.get_shm_provider() {
            ShmProviderState::Ready(provider) => return Ok(provider),
            ShmProviderState::Initializing if started.elapsed() < SHM_PROVIDER_INIT_TIMEOUT => {
                std::thread::sleep(SHM_PROVIDER_INIT_POLL_INTERVAL);
            }
            ShmProviderState::Initializing => {
                return Err(CuError::from(format!(
                    "RemoteDebug server timed out after {:?} initializing its mandatory \
                     shared-memory provider",
                    SHM_PROVIDER_INIT_TIMEOUT
                )));
            }
            ShmProviderState::Disabled => {
                return Err(CuError::from(
                    "RemoteDebug server's mandatory Zenoh shared-memory provider is disabled",
                ));
            }
            ShmProviderState::Error => {
                return Err(CuError::from(
                    "RemoteDebug server failed to allocate and lock its mandatory Zenoh \
                     shared-memory pool; check `ulimit -l`, `df -h /dev/shm`, and the startup \
                     preflight values",
                ));
            }
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
    paths: RemoteDebugPaths,
    session: zenoh::Session,
    #[cfg(target_os = "linux")]
    shm_provider: Arc<RemoteDebugShmProvider>,
    #[cfg(target_os = "linux")]
    shm_config: RemoteDebugShmConfig,
    request_sub: ZenohSubscriber,
    #[cfg(target_os = "linux")]
    reply_publishers: HashMap<String, zenoh::pubsub::Publisher<'static>>,
    event_publishers: EventPublishers,
    app_factory: AF,
    build_callback: CB,
    time_of: TF,
    sessions: HashMap<String, SessionState<App, P, CB, TF, S, L>>,
    next_session_id: u64,
    next_watch_id: u64,
    next_op_id: u64,
    stop_requested: bool,
    session_lifecycle: SessionLifecycleLimits,
    process_info: RemoteDebugProcessInfo,
    _registry_registration: Option<RemoteDebugRegistryRegistration>,
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
    pub fn new(
        paths: RemoteDebugPaths,
        app_factory: AF,
        build_callback: CB,
        time_of: TF,
    ) -> CuResult<Self> {
        Self::new_with_shm_config(
            paths,
            app_factory,
            build_callback,
            time_of,
            RemoteDebugShmConfig::default(),
        )
    }

    pub fn new_with_shm_config(
        paths: RemoteDebugPaths,
        app_factory: AF,
        build_callback: CB,
        time_of: TF,
        shm_config: RemoteDebugShmConfig,
    ) -> CuResult<Self> {
        let zenoh_config = local_server_zenoh_config(&paths, shm_config)?;
        Self::new_with_config(zenoh_config, paths, app_factory, build_callback, time_of)
    }

    pub fn new_with_config(
        zenoh_config: ZenohConfig,
        paths: RemoteDebugPaths,
        app_factory: AF,
        build_callback: CB,
        time_of: TF,
    ) -> CuResult<Self> {
        let shm_config = validate_server_zenoh_shm_config(&zenoh_config)?;
        preflight_remote_debug_shm(shm_config, RemoteDebugShmRole::Server)?;
        let session = zenoh::Wait::wait(zenoh::open(zenoh_config))
            .map_err(cu_error_map("RemoteDebug: failed to open Zenoh session"))?;
        #[cfg(target_os = "linux")]
        let shm_provider = initialize_remote_debug_shm_provider(&session)?;

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

        let process_info = capture_current_process_info(&paths.base);
        let registry_registration = match RemoteDebugRegistryRegistration::register(&process_info) {
            Ok(registration) => Some(registration),
            Err(err) => {
                eprintln!("RemoteDebug: failed to register process metadata: {err}");
                None
            }
        };

        Ok(Self {
            paths,
            session,
            #[cfg(target_os = "linux")]
            shm_provider,
            #[cfg(target_os = "linux")]
            shm_config,
            request_sub,
            #[cfg(target_os = "linux")]
            reply_publishers: HashMap::new(),
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
            session_lifecycle: SessionLifecycleLimits::default(),
            process_info,
            _registry_registration: registry_registration,
        })
    }

    pub fn serve_until_stopped(&mut self) -> CuResult<()> {
        while !self.stop_requested {
            let _ = self.serve_next_timeout(SERVER_RECV_POLL_TIMEOUT)?;
        }
        Ok(())
    }

    pub fn serve_next(&mut self) -> CuResult<()> {
        let sample = self.request_sub.recv().map_err(|e| {
            CuError::from(format!("RemoteDebug: failed to receive RPC request: {e}"))
        })?;
        let payload = sample.payload().to_bytes();
        let (request, codec) = match decode_request(payload.as_ref()) {
            Ok(v) => v,
            Err(err) => {
                eprintln!("RemoteDebug: dropping malformed request frame: {err}");
                return Ok(());
            }
        };

        let response = self.handle_request(request.clone(), codec);
        if let Err(err) = self.publish_reply(&request.reply_to, &response, codec) {
            eprintln!(
                "RemoteDebug: failed to publish reply to '{}': {err}",
                request.reply_to
            );
        }
        Ok(())
    }

    pub fn serve_next_timeout(&mut self, timeout: Duration) -> CuResult<bool> {
        let maybe_sample = self.request_sub.recv_timeout(timeout).map_err(|e| {
            CuError::from(format!(
                "RemoteDebug: failed to receive RPC request with timeout: {e}"
            ))
        })?;

        let sample = match maybe_sample {
            Some(sample) => sample,
            None => {
                self.cleanup_expired_sessions();
                return Ok(false);
            }
        };

        let payload = sample.payload().to_bytes();
        let (request, codec) = match decode_request(payload.as_ref()) {
            Ok(v) => v,
            Err(err) => {
                eprintln!("RemoteDebug: dropping malformed request frame: {err}");
                return Ok(true);
            }
        };
        let response = self.handle_request(request.clone(), codec);
        if let Err(err) = self.publish_reply(&request.reply_to, &response, codec) {
            eprintln!(
                "RemoteDebug: failed to publish reply to '{}': {err}",
                request.reply_to
            );
            return Ok(true);
        }
        Ok(true)
    }

    fn handle_request(&mut self, request: DebugRpcRequest, codec: WireCodec) -> DebugRpcResponse {
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

        let include_handle_contents = request
            .params
            .get("include_handle_contents")
            .and_then(Value::as_bool)
            .unwrap_or(false);
        if include_handle_contents && codec != WireCodec::Cbor {
            return err_response(
                request_id,
                "BinaryAttachmentsRequireCbor",
                "handle contents are available only with the CBOR wire codec",
            );
        }

        self.cleanup_expired_sessions();
        let (mut response, attachments) =
            collect_debug_handle_attachments(|| self.dispatch_request(&request));
        response.attachments = attachments
            .into_iter()
            .map(DebugRpcAttachment::from)
            .collect();
        response
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
            "logs.strings" => self.handle_logs_strings(request_id, request.session_id.as_deref()),
            "logs.list" => {
                self.handle_logs_list(request_id, request.session_id.as_deref(), &request.params)
            }
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
            "schema.get_outputs" => self.handle_schema_get_outputs(
                request_id,
                request.session_id.as_deref(),
                &request.params,
            ),
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
            "process.describe" => self.handle_process_describe(
                request_id,
                request.session_id.as_deref(),
                &request.params,
            ),
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
        self.cleanup_expired_sessions();
        if self.sessions.len() >= self.session_lifecycle.max_sessions {
            return err_response(
                request_id,
                "SessionLimitReached",
                &format!(
                    "Maximum active sessions ({}) reached",
                    self.session_lifecycle.max_sessions
                ),
            );
        }
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
                log_base: path,
                log_index_path: None,
                interned_strings: None,
                replay_structured_logs: Vec::new(),
                opened_at: Instant::now(),
                last_touched_at: Instant::now(),
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
                "capabilities": capabilities_json(self.session_lifecycle),
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
        if let Err(e) = self.session_mut(Some(sid)) {
            return err_response(request_id, "SessionNotFound", &e.to_string());
        }

        ok_response(
            request_id,
            capabilities_json(self.session_lifecycle),
            None,
            None,
        )
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
        if let Err(e) = self.session_mut(Some(sid)) {
            return err_response(request_id, "SessionNotFound", &e.to_string());
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

        let (jump_result, captured_logs) =
            capture_live_logs(|| seek_to_index(&mut state.session, resolved.idx));
        let jump = match jump_result {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SeekFailed", &e.to_string()),
        };
        state.replay_structured_logs.extend(captured_logs);

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

        let (jump_result, captured_logs) =
            capture_live_logs(|| seek_to_index(&mut state.session, resolved.idx));
        let jump = match jump_result {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "RunUntilFailed", &e.to_string()),
        };
        state.replay_structured_logs.extend(captured_logs);
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

        let (jump_result, captured_logs) = capture_live_logs(|| state.session.step(parsed.delta));
        let jump = match jump_result {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "StepFailed", &e.to_string()),
        };
        state.replay_structured_logs.extend(captured_logs);
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
        if parsed.limit == 0 {
            return param_err_response(request_id, "limit must be greater than zero".to_string());
        }
        if parsed.limit > MAX_REPLAY_BATCH_LIMIT {
            return param_err_response(
                request_id,
                format!(
                    "limit must be <= {MAX_REPLAY_BATCH_LIMIT}, got {}",
                    parsed.limit
                ),
            );
        }

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
            let (jump_result, captured_logs) =
                capture_live_logs(|| seek_to_index(&mut state.session, resolved.idx));
            let jump = match jump_result {
                Ok(v) => v,
                Err(e) => return err_response(request_id, "ReplayFailed", &e.to_string()),
            };
            state.replay_structured_logs.extend(captured_logs);
            update_after_jump(state, &jump);
        }

        let mut items = Vec::with_capacity(parsed.limit as usize);
        for step_idx in 0..parsed.limit {
            let (replay_result, captured_logs) = if step_idx == 0 {
                capture_live_logs(|| replay_current_step(&mut state.session))
            } else {
                capture_live_logs(|| state.session.step(1))
            };
            let replayed = match replay_result {
                Ok(v) => v,
                Err(e) => return err_response(request_id, "ReplayFailed", &e.to_string()),
            };
            state.replay_structured_logs.extend(captured_logs);
            update_after_jump(state, &replayed);

            let item =
                match nav_replay_item_snapshot::<App, P, CB, TF, S, L>(state, &time_of, &parsed) {
                    Ok(v) => v,
                    Err(e) => return err_response(request_id, "ReplayFailed", &e.to_string()),
                };
            items.push(item);
        }

        let cursor = match cursor_snapshot(state, &time_of) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "ReplayFailed", &e.to_string()),
        };

        ok_response(
            request_id,
            json!({
                "cursor": cursor,
                "replayed": items.len(),
                "items": items,
                "next_offset": Value::Null,
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
        let include_handle_contents = parsed.include_handle_contents;
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
                        return err_response(
                            request_id,
                            "GetClFailed",
                            "target copperlist missing",
                        );
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
            include_handle_contents,
            parsed.output_indices.as_deref(),
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
        let query_cursor = resolved_at.as_ref().and_then(|resolved| {
            query_cursor_snapshot_from_resolved(&mut state.session, resolved, &time_of).ok()
        });

        ok_response(
            request_id,
            json!({
                "cursor": cursor,
                "query_cursor": query_cursor,
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

        let page = match validated_page(parsed.page.unwrap_or_default()) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };

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
                json!({"items": [], "next_offset": Value::Null, "total": 0}),
                Some(state.cursor_rev),
                None,
            );
        }

        let max_idx = end.idx;
        let total = max_idx.saturating_sub(start.idx).saturating_add(1);
        let mut items = Vec::new();
        let mut idx = start.idx.saturating_add(page.offset as usize);
        let mut emitted = 0usize;

        while idx <= max_idx && emitted < page.limit as usize {
            let cl = match state.session.cl_at(idx) {
                Ok(Some(cl)) => cl,
                Ok(None) => break,
                Err(e) => return err_response(request_id, "TimelineListFailed", &e.to_string()),
            };
            let mut item = match copperlist_snapshot::<P>(
                cl.as_ref(),
                &time_of,
                true,
                false,
                false,
                parsed.include_handle_contents,
                parsed.output_indices.as_deref(),
            ) {
                Ok(v) => v,
                Err(e) => return err_response(request_id, "TimelineListFailed", &e.to_string()),
            };
            if let Value::Object(map) = &mut item {
                map.insert("idx".to_owned(), json!(idx));
                map.insert(
                    "is_keyframe".to_owned(),
                    json!(state.session.is_keyframe_culistid(cl.id)),
                );
                map.insert(
                    "keyframe_cl".to_owned(),
                    json!(state.session.nearest_keyframe_culistid(cl.id)),
                );
            }
            items.push(item);
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
                "total": total,
            }),
            Some(state.cursor_rev),
            None,
        )
    }

    fn handle_logs_strings(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
    ) -> DebugRpcResponse {
        let process_info = self.process_info.clone();
        let state = match self.session_mut(session_id) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        match ensure_session_interned_strings(state, &process_info) {
            Ok(strings) => ok_response(
                request_id,
                json!({
                    "strings": strings,
                    "index_path": state.log_index_path.as_ref().map(|path| path.display().to_string()),
                }),
                Some(state.cursor_rev),
                None,
            ),
            Err(err) => err_response(request_id, "StructuredLogStringsFailed", &err.to_string()),
        }
    }

    fn handle_logs_list(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        let parsed: LogsListParams = match from_params(params) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };
        let process_info = self.process_info.clone();
        let state = match self.session_mut(session_id) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };

        let strings = match ensure_session_interned_strings(state, &process_info) {
            Ok(strings) => strings.to_vec(),
            Err(err) => {
                return err_response(request_id, "StructuredLogStringsFailed", &err.to_string());
            }
        };
        if parsed.source == LogsSource::Replay {
            let limit = parsed
                .limit
                .unwrap_or(MAX_LOG_PAGE_LIMIT)
                .min(MAX_LOG_PAGE_LIMIT);
            let total = state.replay_structured_logs.len();
            let entries = state
                .replay_structured_logs
                .iter()
                .enumerate()
                .skip(parsed.offset)
                .take(limit)
                .map(|(index, entry)| structured_log_entry_json(index, &strings, entry))
                .collect::<Vec<_>>();
            let next_offset = parsed.offset.saturating_add(entries.len());

            return ok_response(
                request_id,
                json!({
                    "entries": entries,
                    "next_offset": (next_offset < total).then_some(next_offset),
                    "index_path": state.log_index_path.as_ref().map(|path| path.display().to_string()),
                    "source": "replay",
                }),
                Some(state.cursor_rev),
                None,
            );
        }

        let mut reader = match open_structured_log_reader(&state.log_base) {
            Ok(reader) => reader,
            Err(err) => {
                return err_response(request_id, "StructuredLogOpenFailed", &err.to_string());
            }
        };

        let limit = parsed
            .limit
            .unwrap_or(MAX_LOG_PAGE_LIMIT)
            .min(MAX_LOG_PAGE_LIMIT);
        let mut entries = Vec::new();
        let mut decoded_index = 0usize;
        let mut emitted = 0usize;
        let mut reached_end = false;

        while emitted < limit {
            let entry = match read_next_structured_log_entry(&mut reader) {
                Ok(Some(entry)) => entry,
                Ok(None) => {
                    reached_end = true;
                    break;
                }
                Err(err) => {
                    return err_response(request_id, "StructuredLogReadFailed", &err.to_string());
                }
            };

            if decoded_index >= parsed.offset {
                entries.push(structured_log_entry_json(decoded_index, &strings, &entry));
                emitted += 1;
            }
            decoded_index = decoded_index.saturating_add(1);
        }

        ok_response(
            request_id,
            json!({
                "entries": entries,
                "next_offset": (!reached_end).then_some(parsed.offset.saturating_add(emitted)),
                "index_path": state.log_index_path.as_ref().map(|path| path.display().to_string()),
                "source": "recorded",
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

        match build_stack_schema::<App, S, L>() {
            Ok(schema) => ok_response(request_id, schema, None, None),
            Err(err) => err_response(request_id, "SchemaFailed", &err.to_string()),
        }
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
        populate_debug_type_registry::<App>(&mut registry);

        let mut paths: Vec<String> = registry
            .iter()
            .map(|registration| registration.type_info().type_path().to_string())
            .collect();
        append_builtin_debug_type_paths(&mut paths);
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
        populate_debug_type_registry::<App>(&mut registry);

        if let Some(schema) = builtin_debug_type_schema(&parsed.type_path, &parsed.format) {
            return ok_response(request_id, json!({ "schema": schema }), None, None);
        }

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

    fn handle_schema_get_outputs(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        params: &Value,
    ) -> DebugRpcResponse {
        if let Err(e) = self.session_mut(session_id) {
            return err_response(request_id, "SessionNotFound", &e.to_string());
        }

        let mut registry = TypeRegistry::default();
        populate_debug_type_registry::<App>(&mut registry);

        let offset = params
            .get("page")
            .and_then(|page| page.get("offset"))
            .and_then(Value::as_u64)
            .unwrap_or(0) as usize;
        let limit = params
            .get("page")
            .and_then(|page| page.get("limit"))
            .and_then(Value::as_u64)
            .and_then(|limit| usize::try_from(limit).ok())
            .unwrap_or(usize::MAX)
            .max(1);
        let total = P::get_output_specs().len();

        match build_output_schema_entries_page::<P>(&registry, offset, limit) {
            Ok(mut outputs) => {
                let consumed = outputs.len();
                let next_offset = offset
                    .saturating_add(consumed)
                    .lt(&total)
                    .then_some(offset.saturating_add(consumed));
                let field_offset = params
                    .get("field_page")
                    .and_then(|page| page.get("offset"))
                    .and_then(Value::as_u64)
                    .unwrap_or(0) as usize;
                let field_limit = params
                    .get("field_page")
                    .and_then(|page| page.get("limit"))
                    .and_then(Value::as_u64)
                    .and_then(|limit| usize::try_from(limit).ok())
                    .unwrap_or(usize::MAX)
                    .max(1);
                let mut total_fields = None;
                let mut next_field_offset = None;
                if params.get("field_page").is_some()
                    && let Some(fields) = outputs
                        .first_mut()
                        .and_then(|output| output.get_mut("payload_fields"))
                        .and_then(Value::as_array_mut)
                {
                    total_fields = Some(fields.len());
                    let page = fields
                        .iter()
                        .skip(field_offset)
                        .take(field_limit)
                        .cloned()
                        .collect::<Vec<_>>();
                    next_field_offset = field_offset
                        .saturating_add(page.len())
                        .lt(&fields.len())
                        .then_some(field_offset.saturating_add(page.len()));
                    *fields = page;
                }
                ok_response(
                    request_id,
                    json!({
                        "outputs": outputs,
                        "total": total,
                        "next_offset": next_offset,
                        "total_fields": total_fields,
                        "next_field_offset": next_field_offset,
                    }),
                    None,
                    None,
                )
            }
            Err(err) => err_response(request_id, "SchemaFailed", &err.to_string()),
        }
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

        let page = match validated_page(parsed.page.unwrap_or_default()) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };
        let inspected = inspect_value(node, page);
        let cursor = cursor_snapshot(state, &time_of).ok();
        let query_cursor = resolved_at.as_ref().and_then(|resolved| {
            query_cursor_snapshot_from_resolved(&mut state.session, resolved, &time_of).ok()
        });

        ok_response(
            request_id,
            json!({
                "cursor": cursor,
                "query_cursor": query_cursor,
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

        let page = match validated_page(parsed.page.unwrap_or_default()) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };
        let value = apply_page(node.clone(), page);
        let cursor = cursor_snapshot(state, &time_of).ok();
        let query_cursor = resolved_at.as_ref().and_then(|resolved| {
            query_cursor_snapshot_from_resolved(&mut state.session, resolved, &time_of).ok()
        });

        ok_response(
            request_id,
            json!({
                "cursor": cursor,
                "query_cursor": query_cursor,
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

        let page = match validated_page(parsed.page.unwrap_or_default()) {
            Ok(v) => v,
            Err(err) => return param_err_response(request_id, err),
        };
        let start = page.offset as usize;
        let end = start.saturating_add(page.limit as usize);
        let needle = parsed.query.to_ascii_lowercase();
        let mut total = 0usize;
        let mut matches = Vec::with_capacity(page.limit as usize);
        collect_matches_paged(&root, "", &needle, start, end, &mut total, &mut matches);

        let cursor = cursor_snapshot(state, &time_of).ok();
        let query_cursor = resolved_at.as_ref().and_then(|resolved| {
            query_cursor_snapshot_from_resolved(&mut state.session, resolved, &time_of).ok()
        });

        ok_response(
            request_id,
            json!({
                "cursor": cursor,
                "query_cursor": query_cursor,
                "matches": matches,
                "total": total,
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
            && self.session_mut(Some(sid)).is_err()
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

        let state = match self.session_mut(Some(sid)) {
            Ok(v) => v,
            Err(e) => return err_response(request_id, "SessionNotFound", &e.to_string()),
        };
        let cache = state.session.section_cache_stats();
        let cache_hit_rate = {
            let total = cache.hits.saturating_add(cache.misses);
            if total == 0 {
                0.0
            } else {
                cache.hits as f64 / total as f64
            }
        };

        ok_response(
            request_id,
            json!({
                "cache": {
                    "cap": cache.cap,
                    "entries": cache.entries,
                    "hits": cache.hits,
                    "misses": cache.misses,
                    "evictions": cache.evictions,
                    "hit_rate": cache_hit_rate,
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

    #[cfg(target_os = "linux")]
    fn allocate_shm_reply(&self, size: usize) -> Result<ZShmMut, String> {
        if size >= self.shm_config.pool_size_bytes {
            return Err(format!(
                "{} reply is not smaller than the configured {} SHM pool",
                format_bytes(size),
                format_bytes(self.shm_config.pool_size_bytes),
            ));
        }

        let started = Instant::now();
        loop {
            self.shm_provider.garbage_collect();
            self.shm_provider.defragment();
            match zenoh::Wait::wait(self.shm_provider.alloc(size)) {
                Ok(buffer) => return Ok(buffer),
                Err(_) if started.elapsed() < SHM_REPLY_ALLOC_TIMEOUT => {
                    std::thread::sleep(SHM_REPLY_ALLOC_POLL_INTERVAL);
                }
                Err(err) => {
                    return Err(format!(
                        "allocation remained unavailable for {:?}: {err}",
                        SHM_REPLY_ALLOC_TIMEOUT
                    ));
                }
            }
        }
    }

    fn publish_reply(
        &mut self,
        topic: &str,
        response: &DebugRpcResponse,
        codec: WireCodec,
    ) -> CuResult<()> {
        let payload = encode_payload(response, codec, "RemoteDebug: failed to serialize response")?;

        #[cfg(not(target_os = "linux"))]
        {
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

        #[cfg(target_os = "linux")]
        {
            if !self.reply_publishers.contains_key(topic) {
                let publisher = zenoh::Wait::wait(self.session.declare_publisher(keyexpr(topic)?))
                    .map_err(cu_error_map(
                        "RemoteDebug: failed to declare reply publisher",
                    ))?;
                self.reply_publishers.insert(topic.to_owned(), publisher);
            }
            let publisher = self
                .reply_publishers
                .get(topic)
                .ok_or_else(|| CuError::from("RemoteDebug: reply publisher cache miss"))?;
            if payload.len() < self.shm_config.message_size_threshold_bytes {
                return zenoh::Wait::wait(publisher.put(payload).encoding(codec.encoding()))
                    .map_err(cu_error_map("RemoteDebug: failed to publish reply"));
            }

            let mut shm_payload = match self.allocate_shm_reply(payload.len()) {
                Ok(shm_payload) => shm_payload,
                Err(err) => {
                    let allocation_error = err_response(
                        response.request_id.clone(),
                        "SharedMemoryCapacity",
                        &format!(
                            "RemoteDebug refused to copy a {} reply over the socket because its \
                             mandatory {} SHM pool could not allocate it: {err}",
                            format_bytes(payload.len()),
                            format_bytes(self.shm_config.pool_size_bytes),
                        ),
                    );
                    let error_payload = encode_payload(
                        &allocation_error,
                        codec,
                        "RemoteDebug: failed to serialize SHM allocation error",
                    )?;
                    return zenoh::Wait::wait(
                        publisher.put(error_payload).encoding(codec.encoding()),
                    )
                    .map_err(cu_error_map(
                        "RemoteDebug: failed to publish SHM allocation error",
                    ));
                }
            };
            shm_payload[..].copy_from_slice(&payload);
            zenoh::Wait::wait(publisher.put(shm_payload).encoding(codec.encoding()))
                .map_err(cu_error_map("RemoteDebug: failed to publish SHM reply"))
        }
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
        let state = self
            .sessions
            .get_mut(sid)
            .ok_or_else(|| CuError::from(format!("Session '{sid}' not found")))?;
        state.last_touched_at = Instant::now();
        Ok(state)
    }

    fn cleanup_expired_sessions(&mut self) {
        let now = Instant::now();
        let idle_timeout = self.session_lifecycle.idle_timeout;
        self.sessions.retain(|_, state| {
            now.saturating_duration_since(state.last_touched_at) <= idle_timeout
        });
    }

    fn handle_process_describe(
        &mut self,
        request_id: String,
        session_id: Option<&str>,
        _params: &Value,
    ) -> DebugRpcResponse {
        if let Some(sid) = session_id
            && self.session_mut(Some(sid)).is_err()
        {
            return err_response(request_id, "SessionNotFound", "session not found");
        }

        match serde_json::to_value(&self.process_info) {
            Ok(value) => ok_response(request_id, value, None, None),
            Err(err) => err_response(
                request_id,
                "ProcessDescribeFailed",
                &format!("failed encoding process metadata: {err}"),
            ),
        }
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
        attachments: Vec::new(),
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
        attachments: Vec::new(),
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
    #[cfg(target_os = "linux")]
    shm_config: RemoteDebugShmConfig,
}

pub struct RemoteDebugZenohClientBuilder {
    paths: RemoteDebugPaths,
    client_id: String,
    codec: WireCodec,
    zenoh_config: Option<ZenohConfig>,
    shm_config: RemoteDebugShmConfig,
}

impl RemoteDebugZenohClientBuilder {
    pub fn codec(mut self, codec: WireCodec) -> Self {
        self.codec = codec;
        self
    }

    pub fn zenoh_config(mut self, zenoh_config: ZenohConfig) -> Self {
        self.zenoh_config = Some(zenoh_config);
        self
    }

    pub fn shm_config(mut self, shm_config: RemoteDebugShmConfig) -> Self {
        self.shm_config = shm_config;
        self
    }

    pub fn build(self) -> CuResult<RemoteDebugZenohClient> {
        let shm_config = self.shm_config.validate()?;
        let zenoh_config = match self.zenoh_config {
            Some(config) => config,
            None => local_client_zenoh_config(&self.paths, shm_config)?,
        };
        RemoteDebugZenohClient::open(
            zenoh_config,
            self.paths,
            &self.client_id,
            self.codec,
            shm_config,
        )
    }
}

impl RemoteDebugZenohClient {
    pub fn new(paths: RemoteDebugPaths, client_id: &str) -> CuResult<Self> {
        Self::builder(paths, client_id).build()
    }

    pub fn builder(paths: RemoteDebugPaths, client_id: &str) -> RemoteDebugZenohClientBuilder {
        RemoteDebugZenohClientBuilder {
            paths,
            client_id: client_id.to_string(),
            codec: WireCodec::Cbor,
            zenoh_config: None,
            shm_config: RemoteDebugShmConfig::default(),
        }
    }

    fn open(
        zenoh_config: ZenohConfig,
        paths: RemoteDebugPaths,
        client_id: &str,
        codec: WireCodec,
        shm_config: RemoteDebugShmConfig,
    ) -> CuResult<Self> {
        validate_client_zenoh_shm_config(&zenoh_config)?;
        preflight_remote_debug_shm(shm_config, RemoteDebugShmRole::Client)?;
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
            #[cfg(target_os = "linux")]
            shm_config,
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
            #[cfg(target_os = "linux")]
            let payload_len = sample.payload().len();
            #[cfg(target_os = "linux")]
            let payload_is_shm = sample.payload().as_shm().is_some();
            let payload = sample.payload().to_bytes();
            let response = decode_response(payload.as_ref(), self.codec)?;

            if response.request_id == request_id {
                #[cfg(target_os = "linux")]
                if payload_len >= self.shm_config.message_size_threshold_bytes && !payload_is_shm {
                    return Err(CuError::from(format!(
                        "RemoteDebugClient refused a {} reply delivered outside shared memory. \
                         Both endpoints must have SHM enabled and enough RLIMIT_MEMLOCK; \
                         refusing silent Unix-socket fallback.",
                        format_bytes(payload_len),
                    )));
                }
                return Ok(response);
            }
        }
    }
}

fn capabilities_json(session_lifecycle: SessionLifecycleLimits) -> Value {
    json!({
        "version": API_VERSION,
        "wire_codecs": ["cbor", "json"],
        "handle_contents": {
            "deferred_by_default": true,
            "binary_attachments": true,
            "binary_attachment_codec": "cbor",
            "output_index_filtering": true,
        },
        "supports_targets": ["cl", "ts"],
        "max_replay_batch_limit": MAX_REPLAY_BATCH_LIMIT,
        "session_lifecycle": {
            "idle_timeout_ms": session_lifecycle.idle_timeout.as_millis() as u64,
            "max_sessions": session_lifecycle.max_sessions,
            "cleanup_policy": "on_each_request_or_timeout_tick",
        },
        "supports_methods": [
            "admin.stop",
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
            "logs.strings",
            "logs.list",
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
            "health.stats",
            "process.describe"
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
    P: CopperListTuple + 'static,
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
{
    state.last_keyframe = jump.keyframe_culistid;
    state.last_replayed = jump.replayed;
    state.bump_rev();
}

fn ensure_session_interned_strings<'a, App, P, CB, TF, S, L>(
    state: &'a mut SessionState<App, P, CB, TF, S, L>,
    process_info: &RemoteDebugProcessInfo,
) -> CuResult<&'a [String]>
where
    P: CopperListTuple + 'static,
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
{
    if state.interned_strings.is_none() {
        let index_path =
            find_structured_log_index(&state.log_base, process_info).ok_or_else(|| {
                CuError::from("structured log string index not found on debug server")
            })?;
        let strings = cu29_intern_strs::read_interned_strings(&index_path).map_err(|err| {
            CuError::new_with_cause(
                "failed reading structured log string index",
                io::Error::other(err.to_string()),
            )
        })?;
        state.log_index_path = Some(index_path);
        state.interned_strings = Some(strings);
    }
    Ok(state.interned_strings.as_deref().unwrap_or(&[]))
}

fn find_structured_log_index(
    log_base: &Path,
    process_info: &RemoteDebugProcessInfo,
) -> Option<PathBuf> {
    let mut candidates = Vec::<PathBuf>::new();
    if let Ok(path) = std::env::var("LOG_INDEX_DIR") {
        push_log_index_candidate(&mut candidates, PathBuf::from(path));
    }
    if let Some(path) = process_info.restart_env.get("LOG_INDEX_DIR") {
        push_log_index_candidate(&mut candidates, PathBuf::from(path));
    }

    push_log_index_candidates_from_base(&mut candidates, &process_info.working_dir);
    if let Some(parent) = process_info.executable_path.parent() {
        push_log_index_candidates_from_base(&mut candidates, parent);
        for ancestor in parent.ancestors().take(8) {
            push_log_index_candidates_from_base(&mut candidates, ancestor);
        }
    }
    if let Some(parent) = log_base.parent() {
        push_log_index_candidates_from_base(&mut candidates, parent);
        if let Some(grandparent) = parent.parent() {
            push_log_index_candidates_from_base(&mut candidates, grandparent);
        }
    }

    candidates
        .into_iter()
        .find(|path| path.is_file() || path.join("strings.bin").is_file())
}

fn push_log_index_candidates_from_base(candidates: &mut Vec<PathBuf>, base: &Path) {
    for candidate in [
        base.join("cu29_log_index"),
        base.join("target").join("cu29_log_index"),
        base.join("target").join("debug").join("cu29_log_index"),
        base.join("target").join("release").join("cu29_log_index"),
    ] {
        push_log_index_candidate(candidates, candidate);
    }

    if let Ok(entries) = fs::read_dir(base.join("target")) {
        for entry in entries.flatten() {
            push_log_index_candidate(candidates, entry.path().join("cu29_log_index"));
        }
    }
}

fn push_log_index_candidate(candidates: &mut Vec<PathBuf>, candidate: PathBuf) {
    if !candidates.iter().any(|existing| existing == &candidate) {
        candidates.push(candidate);
    }
}

fn open_structured_log_reader(log_base: &Path) -> CuResult<UnifiedLoggerIOReader> {
    let logger = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .map_err(|err| CuError::new_with_cause("failed opening unified log", err))?;
    match logger {
        UnifiedLogger::Read(read_logger) => Ok(UnifiedLoggerIOReader::new(
            read_logger,
            UnifiedLogType::StructuredLogLine,
        )),
        UnifiedLogger::Write(_) => Err(CuError::from(
            "expected read-only unified logger for structured log",
        )),
    }
}

fn read_next_structured_log_entry(
    reader: &mut UnifiedLoggerIOReader,
) -> CuResult<Option<CuLogEntry>> {
    loop {
        return match decode_from_std_read::<CuLogEntry, _, _>(reader, standard()) {
            Ok(entry) if entry.msg_index == 0 => continue,
            Ok(entry) => Ok(Some(entry)),
            Err(DecodeError::UnexpectedEnd { .. }) => Ok(None),
            Err(DecodeError::Io { inner, .. })
                if inner.kind() == std::io::ErrorKind::UnexpectedEof =>
            {
                Ok(None)
            }
            Err(err) => Err(CuError::new_with_cause("error reading structured log", err)),
        };
    }
}

fn structured_log_entry_json(index: usize, strings: &[String], entry: &CuLogEntry) -> Value {
    let params = entry
        .params
        .iter()
        .enumerate()
        .map(|(param_index, value)| {
            let name_index = entry
                .paramname_indexes
                .get(param_index)
                .copied()
                .unwrap_or(0);
            let label = if name_index == 0 {
                format!("${param_index}")
            } else {
                strings
                    .get(name_index as usize)
                    .cloned()
                    .unwrap_or_else(|| format!("param#{name_index}"))
            };
            json!({
                "label": label,
                "name_index": name_index,
                "value": value.to_string(),
                "json_value": serde_json::to_value(value).unwrap_or(Value::Null),
                "numeric_value": numeric_structured_log_value(value),
            })
        })
        .collect::<Vec<_>>();

    json!({
        "index": index,
        "time_ns": entry.time.as_nanos(),
        "level": format!("{:?}", entry.level),
        "msg_index": entry.msg_index,
        "message_template": strings.get(entry.msg_index as usize).cloned(),
        "culistid": entry.origin.culistid,
        "component_id": entry.origin.component_id,
        "task_index": entry.origin.task_index,
        "paramname_indexes": entry.paramname_indexes.iter().copied().collect::<Vec<_>>(),
        "params": params,
    })
}

fn numeric_structured_log_value(value: &cu29_value::Value) -> Option<f64> {
    match value {
        cu29_value::Value::U8(value) => Some(f64::from(*value)),
        cu29_value::Value::U16(value) => Some(f64::from(*value)),
        cu29_value::Value::U32(value) => Some(f64::from(*value)),
        cu29_value::Value::U64(value) => Some(*value as f64),
        cu29_value::Value::U128(value) => Some(*value as f64),
        cu29_value::Value::I8(value) => Some(f64::from(*value)),
        cu29_value::Value::I16(value) => Some(f64::from(*value)),
        cu29_value::Value::I32(value) => Some(f64::from(*value)),
        cu29_value::Value::I64(value) => Some(*value as f64),
        cu29_value::Value::I128(value) => Some(*value as f64),
        cu29_value::Value::F32(value) => Some(f64::from(*value)),
        cu29_value::Value::F64(value) => Some(*value),
        cu29_value::Value::CuTime(value) => Some(value.as_nanos() as f64),
        cu29_value::Value::Newtype(value) | cu29_value::Value::Option(Some(value)) => {
            numeric_structured_log_value(value)
        }
        _ => None,
    }
}

fn seek_to_index<App, P, CB, TF, S, L>(
    session: &mut CuDebugSession<App, P, CB, TF, S, L>,
    idx: usize,
) -> CuResult<JumpOutcome>
where
    App: CuSimApplication<S, L> + CurrentRuntimeCopperList<P>,
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
    session.goto_index(idx)
}

fn replay_current_step<App, P, CB, TF, S, L>(
    session: &mut CuDebugSession<App, P, CB, TF, S, L>,
) -> CuResult<JumpOutcome>
where
    App: CuSimApplication<S, L> + CurrentRuntimeCopperList<P>,
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

fn nav_replay_item_snapshot<App, P, CB, TF, S, L>(
    state: &mut SessionState<App, P, CB, TF, S, L>,
    time_of: &TF,
    params: &NavReplayParams,
) -> CuResult<Value>
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
    let cursor = cursor_snapshot(state, time_of)?;
    let mut item = Map::new();
    item.insert(
        "cursor".to_string(),
        serde_json::to_value(&cursor).unwrap_or(Value::Null),
    );

    if params.include_cl_snapshot
        || params.include_payloads
        || params.include_metadata
        || params.include_raw
    {
        let current_cl = match state.session.current_cl()? {
            Some(cl) => copperlist_snapshot::<P>(
                cl.as_ref(),
                time_of,
                params.include_payloads,
                params.include_metadata,
                params.include_raw,
                params.include_handle_contents,
                params.output_indices.as_deref(),
            )?,
            None => Value::Null,
        };
        item.insert("cl_snapshot".to_string(), current_cl);
    }

    if params.include_replayed_cl {
        let replayed_cl = replayed_copperlist_snapshot::<App, P, CB, TF, S, L>(
            state,
            time_of,
            params.include_payloads,
            params.include_metadata,
            params.include_raw,
            params.include_handle_contents,
            params.output_indices.as_deref(),
        )?;
        item.insert("replayed_cl".to_string(), replayed_cl);
    }

    if params.include_state {
        let tasks = build_tasks_json::<App, P, CB, TF, S, L>(state)?;
        item.insert("tasks".to_string(), tasks);
    }

    Ok(Value::Object(item))
}

fn cursor_snapshot<App, P, CB, TF, S, L>(
    state: &mut SessionState<App, P, CB, TF, S, L>,
    time_of: &TF,
) -> CuResult<CursorSnapshot>
where
    App: CuSimApplication<S, L>,
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

fn query_cursor_snapshot_from_resolved<App, P, CB, TF, S, L>(
    session: &mut CuDebugSession<App, P, CB, TF, S, L>,
    resolved: &ResolvedAt,
    time_of: &TF,
) -> CuResult<QueryCursorSnapshot>
where
    App: CuSimApplication<S, L>,
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
    let cl = session
        .cl_at(resolved.idx)?
        .ok_or_else(|| CuError::from("Resolved cursor points to missing copperlist"))?;
    Ok(QueryCursorSnapshot {
        cl: cl.id,
        idx: resolved.idx,
        ts_ns: time_of(cl.as_ref()).map(|t| t.as_nanos()),
        is_keyframe: session.is_keyframe_culistid(cl.id),
        keyframe_cl: session.nearest_keyframe_culistid(cl.id),
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
    P: CopperListTuple + 'static,
    CB: for<'a> Fn(
        &'a crate::copperlist::CopperList<P>,
        RobotClock,
        RobotClockMock,
    )
        -> Box<dyn for<'z> FnMut(App::Step<'z>) -> crate::simulation::SimOverride + 'a>,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
{
    let indexed_mode = indexed_resolve_mode(mode);
    let idx = match target {
        Target::Cl { cl } => session.resolve_index_for_culistid(*cl, indexed_mode)?,
        Target::Ts { ts_ns } => {
            session.resolve_index_for_time(CuTime::from(*ts_ns), indexed_mode)?
        }
    };
    let entry = session
        .cl_at(idx)?
        .ok_or_else(|| CuError::from("Resolved target points to missing copperlist"))?;

    Ok(ResolvedAt {
        cl: entry.id,
        ts_ns: time_of(entry.as_ref()).map(|t| t.as_nanos()),
        idx,
    })
}

fn indexed_resolve_mode(mode: ResolveMode) -> IndexedResolveMode {
    match mode {
        ResolveMode::Exact => IndexedResolveMode::Exact,
        ResolveMode::AtOrAfter => IndexedResolveMode::AtOrAfter,
        ResolveMode::AtOrBefore => IndexedResolveMode::AtOrBefore,
    }
}

fn copperlist_snapshot<P: CopperListTuple + 'static>(
    cl: &crate::copperlist::CopperList<P>,
    time_of: &impl Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime>,
    include_payloads: bool,
    include_metadata: bool,
    include_raw: bool,
    include_handle_contents: bool,
    output_indices: Option<&[usize]>,
) -> CuResult<Value> {
    let task_ids = P::get_all_task_ids();
    let msgs = cl.cumsgs();
    let mut entries = Vec::with_capacity(msgs.len());

    for (i, msg) in msgs.iter().enumerate() {
        let task_id = task_ids.get(i).copied().unwrap_or("<?>");
        let payload = if include_payloads {
            let include_message_handle_contents = include_handle_contents
                && output_indices.is_none_or(|indices| indices.contains(&i));
            with_debug_handle_contents(include_message_handle_contents, || {
                msg.payload()
                    .map(erased_serialize_to_value)
                    .transpose()
                    .map(|payload| payload.unwrap_or(Value::Null))
            })?
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
    let view = DebugMessageMetadataView {
        tov,
        process_time: metadata.process_time(),
        status_txt: metadata.status_txt().0.to_string(),
        origin: metadata.origin().cloned(),
    };

    cu29_value::to_value(view)
        .map(debug_value_to_json)
        .unwrap_or_else(|_| Value::String("metadata serialization failed".to_string()))
}

fn erased_serialize_to_value(value: &dyn erased_serde::Serialize) -> CuResult<Value> {
    let value = cu29_value::to_value(value)
        .map_err(|e| CuError::from(format!("Failed to serialize erased payload: {e}")))?;
    Ok(debug_value_to_json(value))
}

fn debug_value_to_json(value: cu29_value::Value) -> Value {
    use cu29_value::Value as DebugValue;

    match value {
        DebugValue::Bool(value) => Value::Bool(value),
        DebugValue::U8(value) => json!(value),
        DebugValue::U16(value) => json!(value),
        DebugValue::U32(value) => json!(value),
        DebugValue::U64(value) => json!(value),
        DebugValue::U128(value) => json!({"__cu_u128__": value.to_string()}),
        DebugValue::I8(value) => json!(value),
        DebugValue::I16(value) => json!(value),
        DebugValue::I32(value) => json!(value),
        DebugValue::I64(value) => json!(value),
        DebugValue::I128(value) => json!({"__cu_i128__": value.to_string()}),
        DebugValue::F32(value) => debug_float_to_json(f64::from(value)),
        DebugValue::F64(value) => debug_float_to_json(value),
        DebugValue::Char(value) => Value::String(value.to_string()),
        DebugValue::String(value) => Value::String(value),
        DebugValue::Unit | DebugValue::Option(None) => Value::Null,
        DebugValue::Option(Some(value)) | DebugValue::Newtype(value) => debug_value_to_json(*value),
        DebugValue::Seq(values) => {
            Value::Array(values.into_iter().map(debug_value_to_json).collect())
        }
        DebugValue::Map(values)
            if values
                .keys()
                .all(|key| matches!(key, DebugValue::String(_))) =>
        {
            Value::Object(
                values
                    .into_iter()
                    .filter_map(|(key, value)| match key {
                        DebugValue::String(key) => Some((key, debug_value_to_json(value))),
                        _ => None,
                    })
                    .collect(),
            )
        }
        DebugValue::Map(values) => json!({
            "__cu_map__": values
                .into_iter()
                .map(|(key, value)| {
                    json!({
                        "key": debug_value_to_json(key),
                        "value": debug_value_to_json(value),
                    })
                })
                .collect::<Vec<_>>()
        }),
        DebugValue::Bytes(values) => {
            Value::Array(values.into_iter().map(|value| json!(value)).collect())
        }
        DebugValue::CuTime(value) => json!(value.as_nanos()),
    }
}

fn debug_float_to_json(value: f64) -> Value {
    if value.is_nan() {
        json!({"__cu_float__": "nan"})
    } else if value == f64::INFINITY {
        json!({"__cu_float__": "positive_infinity"})
    } else if value == f64::NEG_INFINITY {
        json!({"__cu_float__": "negative_infinity"})
    } else {
        json!(value)
    }
}

fn build_tasks_json<App, P, CB, TF, S, L>(
    state: &mut SessionState<App, P, CB, TF, S, L>,
) -> CuResult<Value>
where
    App: CuSimApplication<S, L> + ReflectTaskIntrospection,
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
    let mut tasks = serde_json::Map::new();
    let mut registry = TypeRegistry::default();
    populate_debug_type_registry::<App>(&mut registry);

    for task_id in P::get_all_task_ids() {
        if let Ok(task_state) = state
            .session
            .with_debug_state(task_id, |task| reflect_value_to_json(task, &registry))
        {
            tasks.insert(task_id.to_string(), task_state);
        }
    }

    Ok(Value::Object(tasks))
}

fn replayed_copperlist_snapshot<App, P, CB, TF, S, L>(
    state: &mut SessionState<App, P, CB, TF, S, L>,
    time_of: &TF,
    include_payloads: bool,
    include_metadata: bool,
    include_raw: bool,
    include_handle_contents: bool,
    output_indices: Option<&[usize]>,
) -> CuResult<Value>
where
    App: CuSimApplication<S, L> + CurrentRuntimeCopperList<P>,
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
    let snapshot = state.session.with_app(|app| {
        app.current_runtime_copperlist_bytes()
            .map(|bytes| {
                bincode::decode_from_slice::<crate::copperlist::CopperList<P>, _>(
                    bytes,
                    bincode::config::standard(),
                )
                .map(|(cl, _)| cl)
                .map_err(|e| {
                    CuError::new_with_cause("Failed to decode replayed CopperList snapshot", e)
                })
                .and_then(|cl| {
                    copperlist_snapshot::<P>(
                        &cl,
                        time_of,
                        include_payloads,
                        include_metadata,
                        include_raw,
                        include_handle_contents,
                        output_indices,
                    )
                })
            })
            .transpose()
    })?;

    Ok(snapshot.unwrap_or(Value::Null))
}

fn build_state_root_json<App, P, CB, TF, S, L>(
    state: &mut SessionState<App, P, CB, TF, S, L>,
    time_of: &TF,
) -> CuResult<Value>
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
    let tasks = build_tasks_json::<App, P, CB, TF, S, L>(state)?;

    let current_cl = match state.session.current_cl()? {
        Some(cl) => copperlist_snapshot::<P>(cl.as_ref(), time_of, true, true, false, false, None)?,
        None => Value::Null,
    };

    let replayed_cl = replayed_copperlist_snapshot::<App, P, CB, TF, S, L>(
        state, time_of, true, true, false, false, None,
    )?;

    let cursor = cursor_snapshot(state, time_of)
        .map(|c| serde_json::to_value(c).unwrap_or(Value::Null))
        .unwrap_or(Value::Null);

    Ok(json!({
        "tasks": tasks,
        "current_cl": current_cl,
        "replayed_cl": replayed_cl,
        "cursor": cursor,
    }))
}

fn state_root_for_query<App, P, CB, TF, S, L>(
    state: &mut SessionState<App, P, CB, TF, S, L>,
    at: Option<&At>,
    time_of: &TF,
) -> CuResult<(Value, Option<ResolvedAt>)>
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
    if let Some(at) = at {
        let resolved = resolve_target(&mut state.session, &at.target, at.resolve, time_of)?;

        if at.mutate_cursor {
            let jump = seek_to_index(&mut state.session, resolved.idx)?;
            update_after_jump(state, &jump);
            let root = build_state_root_json::<App, P, CB, TF, S, L>(state, time_of)?;
            return Ok((root, Some(resolved)));
        }

        let previous_idx = state.session.current_index().ok_or_else(|| {
            CuError::from("Non-mutating at queries require an initialized cursor")
        })?;
        let saved_rev = state.cursor_rev;
        let saved_last_keyframe = state.last_keyframe;
        let saved_last_replayed = state.last_replayed;

        let did_temp_seek = resolved.idx != previous_idx;
        if did_temp_seek {
            let _ = seek_to_index(&mut state.session, resolved.idx)?;
        }

        let root_result = build_state_root_json::<App, P, CB, TF, S, L>(state, time_of);
        let restore_result = if did_temp_seek {
            seek_to_index(&mut state.session, previous_idx).map(|_| ())
        } else {
            Ok(())
        };

        // Preserve logical cursor revision/metrics for non-mutating queries.
        state.cursor_rev = saved_rev;
        state.last_keyframe = saved_last_keyframe;
        state.last_replayed = saved_last_replayed;

        return match (root_result, restore_result) {
            (Ok(root), Ok(())) => Ok((root, Some(resolved))),
            (Err(e), Ok(())) => Err(e),
            (Ok(_), Err(restore_err)) => Err(CuError::from(format!(
                "Failed to restore cursor after non-mutating query: {restore_err}"
            ))),
            (Err(query_err), Err(restore_err)) => Err(CuError::from(format!(
                "Non-mutating query failed: {query_err}; additionally failed to restore cursor: {restore_err}"
            ))),
        };
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

fn collect_matches_paged(
    value: &Value,
    path: &str,
    needle: &str,
    start: usize,
    end: usize,
    total: &mut usize,
    out: &mut Vec<Value>,
) {
    match value {
        Value::Object(map) => {
            for (k, v) in map {
                let child_path = if path.is_empty() {
                    format!("/{k}")
                } else {
                    format!("{path}/{k}")
                };
                if k.to_ascii_lowercase().contains(needle) {
                    push_paged_match(child_path.clone(), v, start, end, total, out);
                }
                collect_matches_paged(v, &child_path, needle, start, end, total, out);
            }
        }
        Value::Array(list) => {
            for (i, v) in list.iter().enumerate() {
                let child_path = if path.is_empty() {
                    format!("/{i}")
                } else {
                    format!("{path}/{i}")
                };
                collect_matches_paged(v, &child_path, needle, start, end, total, out);
            }
        }
        _ => {
            let text = value.to_string().to_ascii_lowercase();
            if text.contains(needle) {
                push_paged_match(path.to_string(), value, start, end, total, out);
            }
        }
    }
}

fn push_paged_match(
    path: String,
    value: &Value,
    start: usize,
    end: usize,
    total: &mut usize,
    out: &mut Vec<Value>,
) {
    if *total >= start && *total < end {
        out.push(json!({
            "path": path,
            "kind": kind_of(value),
            "preview": preview_json(value),
        }));
    }
    *total = total.saturating_add(1);
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
    let mission_id = <App as CuSimApplication<S, L>>::mission_id();
    let graph = config.get_graph(mission_id)?;

    let mut registry = TypeRegistry::default();
    populate_debug_type_registry::<App>(&mut registry);
    let mut types: Vec<String> = registry
        .iter()
        .map(|registration| registration.type_info().type_path().to_string())
        .collect();
    types.sort();
    types.dedup();

    let mut nodes = Vec::new();
    for (node_id, node) in graph.get_all_nodes() {
        let flavor = match node.get_flavor() {
            Flavor::Task => "task",
            Flavor::Bridge => "bridge",
        };
        let task_type = match crate::curuntime::find_task_type_for_id(graph, node_id)? {
            crate::curuntime::CuTaskType::Source => "source",
            crate::curuntime::CuTaskType::Regular => "regular",
            crate::curuntime::CuTaskType::Sink => "sink",
        };
        let state_type_path =
            <App as ReflectTaskIntrospection>::debug_state_type_path(node.get_id().as_str())
                .map(str::to_owned)
                .or_else(|| {
                    resolve_type_info_by_path(&registry, node.get_type())
                        .map(|info| info.type_path().to_string())
                });
        let resolved_info = state_type_path
            .as_deref()
            .and_then(|type_path| resolve_type_info_by_path(&registry, type_path));
        let state_fields = resolved_info
            .map(|info| build_field_catalog(&registry, info, None))
            .unwrap_or_default();

        nodes.push(json!({
            "node_id": node_id,
            "id": node.get_id(),
            "type": node.get_type(),
            "flavor": flavor,
            "task_type": task_type,
            "state_type_path": state_type_path,
            "state_fields": state_fields,
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
        .filter(|bridge| graph.get_node_id_by_name(bridge.id.as_str()).is_some())
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

    Ok(json!({
        "mission_id": mission_id,
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
        "description": info.type_path().to_string(),
        "reflect": format!("{info:#?}"),
    })
}

#[cfg(test)]
fn build_output_schema_entries<P>(registry: &TypeRegistry) -> CuResult<Vec<Value>>
where
    P: CopperListTuple + 'static,
{
    build_output_schema_entries_page::<P>(registry, 0, usize::MAX)
}

fn build_output_schema_entries_page<P>(
    registry: &TypeRegistry,
    offset: usize,
    limit: usize,
) -> CuResult<Vec<Value>>
where
    P: CopperListTuple + 'static,
{
    let metadata_fields = build_message_metadata_field_descriptors(registry)?;

    P::get_output_specs()
        .iter()
        .enumerate()
        .skip(offset)
        .take(limit)
        .map(|(index, spec)| {
            let payload_type_path = spec.payload_type_path();
            let payload_type =
                resolve_type_info_by_path(registry, payload_type_path).ok_or_else(|| {
                    CuError::from(format!(
                        "Payload type '{}' is not registered",
                        payload_type_path
                    ))
                })?;
            let payload_fields = build_field_catalog(registry, payload_type, Some(spec.task_id));
            Ok(json!({
                "index": index,
                "task_id": spec.task_id,
                "message_type": spec.msg_type,
                "payload_type_path": payload_type.type_path(),
                "payload_fields": payload_fields,
                "metadata_fields": metadata_fields,
            }))
        })
        .collect()
}

fn build_message_metadata_field_descriptors(
    registry: &TypeRegistry,
) -> CuResult<Vec<DebugFieldDescriptor>> {
    let metadata_type_path = core::any::type_name::<DebugMessageMetadataView>();
    let metadata_type =
        resolve_type_info_by_path(registry, metadata_type_path).ok_or_else(|| {
            CuError::from(format!(
                "Message metadata type '{}' is not registered",
                metadata_type_path
            ))
        })?;

    let mut fields = build_field_catalog(registry, metadata_type, None);
    if let Some(status_txt) = fields
        .iter_mut()
        .find(|field| field.display_path == "status_txt")
    {
        status_txt.value_type_path = core::any::type_name::<CuCompactString>().to_owned();
    }

    Ok(fields)
}

fn build_field_catalog(
    registry: &TypeRegistry,
    type_info: &'static TypeInfo,
    binding_root: Option<&str>,
) -> Vec<DebugFieldDescriptor> {
    match type_info {
        TypeInfo::Struct(info) => struct_children(registry, info, "", binding_root, false),
        _ => vec![build_type_node(
            registry,
            type_info,
            type_info.type_path(),
            "",
            binding_root,
            false,
        )],
    }
}

fn debug_scalar_kinds() -> &'static HashMap<&'static str, DebugScalarKind> {
    static SCALAR_KINDS: OnceLock<HashMap<&'static str, DebugScalarKind>> = OnceLock::new();
    SCALAR_KINDS.get_or_init(|| {
        let mut kinds = HashMap::new();
        for registration in cu29_clock::debug_scalar_registrations() {
            kinds.insert(registration.type_path, DebugScalarKind::U64);
        }
        for registration in cu29_units::debug_scalar_registrations() {
            kinds.insert(registration.type_path, registration.scalar_kind);
        }
        kinds.insert(
            core::any::type_name::<CuCompactString>(),
            DebugScalarKind::String,
        );
        kinds
    })
}

fn debug_scalar_kind(value_type_path: &str) -> Option<DebugScalarKind> {
    debug_scalar_kinds().get(value_type_path).copied()
}

fn debug_scalar_semantics(value_type_path: &str) -> Option<DebugFieldSemantics> {
    static FIELD_SEMANTICS: OnceLock<HashMap<&'static str, DebugFieldSemantics>> = OnceLock::new();
    FIELD_SEMANTICS
        .get_or_init(|| {
            let mut semantics = HashMap::new();
            for registration in cu29_clock::debug_scalar_registrations() {
                semantics.insert(
                    registration.type_path,
                    match registration.kind {
                        cu29_clock::ClockDebugScalarKind::Time => DebugFieldSemantics::Time,
                        cu29_clock::ClockDebugScalarKind::OptionalTime => {
                            DebugFieldSemantics::OptionalTime
                        }
                        cu29_clock::ClockDebugScalarKind::Duration => DebugFieldSemantics::Duration,
                    },
                );
            }
            for registration in cu29_units::debug_scalar_registrations() {
                semantics.insert(registration.type_path, registration.semantics);
            }
            semantics
        })
        .get(value_type_path)
        .cloned()
}

fn debug_structured_semantics(value_type_path: &str) -> Option<DebugFieldSemantics> {
    matches!(
        value_type_path,
        "cu_spatial_payloads::GeodeticPosition"
            | "cu_gnss_payloads::GeodeticPosition"
            | "cu_sensor_payloads::GeodeticPosition"
    )
    .then_some(DebugFieldSemantics::GeodeticPosition)
}

fn debug_type_semantics(value_type_path: &str) -> Option<DebugFieldSemantics> {
    debug_scalar_semantics(value_type_path).or_else(|| debug_structured_semantics(value_type_path))
}

fn build_field_node(
    registry: &TypeRegistry,
    type_info: Option<&'static TypeInfo>,
    fallback_type: &Type,
    display_path: &str,
    binding_name: Option<&str>,
    nullable: bool,
) -> DebugFieldDescriptor {
    let Some(type_info) = type_info else {
        let value_type_path = fallback_type.path();
        return debug_field_descriptor(
            display_path,
            binding_name,
            value_type_path,
            DebugFieldShape {
                semantics: None,
                scalar_kind: primitive_scalar_kind(value_type_path),
                nullable,
                kind: DebugFieldKind::Scalar,
            },
            Vec::new(),
        );
    };

    build_type_node(
        registry,
        type_info,
        type_info.type_path(),
        display_path,
        binding_name,
        nullable,
    )
}

fn build_type_node(
    registry: &TypeRegistry,
    type_info: &'static TypeInfo,
    value_type_path: &str,
    display_path: &str,
    binding_name: Option<&str>,
    nullable: bool,
) -> DebugFieldDescriptor {
    if let Some(scalar_kind) = debug_scalar_kind(value_type_path) {
        return debug_field_descriptor(
            display_path,
            binding_name,
            value_type_path,
            DebugFieldShape {
                semantics: debug_type_semantics(value_type_path),
                scalar_kind: Some(scalar_kind),
                nullable,
                kind: DebugFieldKind::Scalar,
            },
            Vec::new(),
        );
    }

    match type_info {
        TypeInfo::Struct(info) => debug_field_descriptor(
            display_path,
            binding_name,
            value_type_path,
            DebugFieldShape {
                semantics: debug_type_semantics(value_type_path),
                scalar_kind: None,
                nullable,
                kind: DebugFieldKind::Struct,
            },
            struct_children(registry, info, display_path, binding_name, nullable),
        ),
        TypeInfo::TupleStruct(info) => debug_field_descriptor(
            display_path,
            binding_name,
            value_type_path,
            DebugFieldShape {
                semantics: debug_type_semantics(value_type_path),
                scalar_kind: None,
                nullable,
                kind: DebugFieldKind::TupleStruct,
            },
            tuple_struct_children(registry, info, display_path, binding_name, nullable),
        ),
        TypeInfo::Tuple(info) => debug_field_descriptor(
            display_path,
            binding_name,
            value_type_path,
            DebugFieldShape {
                semantics: debug_type_semantics(value_type_path),
                scalar_kind: None,
                nullable,
                kind: DebugFieldKind::Tuple,
            },
            tuple_children(registry, info, display_path, binding_name, nullable),
        ),
        TypeInfo::List(info) => debug_field_descriptor(
            display_path,
            binding_name,
            value_type_path,
            DebugFieldShape {
                semantics: debug_type_semantics(value_type_path),
                scalar_kind: None,
                nullable,
                kind: DebugFieldKind::List,
            },
            indexed_children(
                registry,
                info.item_info(),
                &info.item_ty(),
                display_path,
                binding_name,
                nullable,
            ),
        ),
        TypeInfo::Array(info) => debug_field_descriptor(
            display_path,
            binding_name,
            value_type_path,
            DebugFieldShape {
                semantics: debug_type_semantics(value_type_path),
                scalar_kind: None,
                nullable,
                kind: DebugFieldKind::Array,
            },
            indexed_children(
                registry,
                info.item_info(),
                &info.item_ty(),
                display_path,
                binding_name,
                nullable,
            ),
        ),
        TypeInfo::Map(info) => {
            let mut descriptor = debug_field_descriptor(
                display_path,
                binding_name,
                value_type_path,
                DebugFieldShape {
                    semantics: debug_type_semantics(value_type_path),
                    scalar_kind: None,
                    nullable,
                    kind: DebugFieldKind::Map,
                },
                Vec::new(),
            );
            let key_ty = info.key_ty();
            let value_ty = info.value_ty();
            descriptor.map_key = Some(Box::new(build_field_node(
                registry,
                info.key_info(),
                &key_ty,
                &format!("{}{{key}}", path_or_value(display_path)),
                binding_name
                    .map(|name| format!("{}{{key}}", path_or_value(name)))
                    .as_deref(),
                nullable,
            )));
            descriptor.map_value = Some(Box::new(build_field_node(
                registry,
                info.value_info(),
                &value_ty,
                &format!("{}{{value}}", path_or_value(display_path)),
                binding_name
                    .map(|name| format!("{}{{value}}", path_or_value(name)))
                    .as_deref(),
                nullable,
            )));
            descriptor
        }
        TypeInfo::Set(info) => debug_field_descriptor(
            display_path,
            binding_name,
            value_type_path,
            DebugFieldShape {
                semantics: debug_type_semantics(value_type_path),
                scalar_kind: None,
                nullable,
                kind: DebugFieldKind::Set,
            },
            indexed_children(
                registry,
                registry.get_type_info(info.value_ty().id()),
                &info.value_ty(),
                display_path,
                binding_name,
                nullable,
            ),
        ),
        TypeInfo::Enum(info) => {
            if let Some((inner_info, inner_type)) = option_inner_field(info) {
                return build_field_node(
                    registry,
                    inner_info,
                    inner_type,
                    display_path,
                    binding_name,
                    true,
                );
            }

            let mut descriptor = debug_field_descriptor(
                display_path,
                binding_name,
                value_type_path,
                DebugFieldShape {
                    semantics: debug_type_semantics(value_type_path),
                    scalar_kind: None,
                    nullable,
                    kind: DebugFieldKind::Enum,
                },
                Vec::new(),
            );
            descriptor.enum_variants =
                enum_variant_descriptors(registry, info, display_path, binding_name, nullable);
            descriptor
        }
        TypeInfo::Opaque(info) => debug_field_descriptor(
            display_path,
            binding_name,
            value_type_path,
            DebugFieldShape {
                semantics: debug_type_semantics(value_type_path),
                scalar_kind: primitive_scalar_kind(info.type_path()),
                nullable,
                kind: DebugFieldKind::Scalar,
            },
            Vec::new(),
        ),
    }
}

fn struct_children(
    registry: &TypeRegistry,
    info: &StructInfo,
    display_path: &str,
    binding_name: Option<&str>,
    nullable: bool,
) -> Vec<DebugFieldDescriptor> {
    let skipped = skipped_indices(registry, info.type_id());
    info.iter()
        .enumerate()
        .filter(|(index, _)| !skipped.contains(index))
        .map(|(_, field)| {
            let next_display = join_field_path(display_path, field.name());
            let next_binding = binding_name.map(|current| join_field_path(current, field.name()));
            build_field_node(
                registry,
                field.type_info(),
                field.ty(),
                &next_display,
                next_binding.as_deref(),
                nullable,
            )
        })
        .collect()
}

fn tuple_struct_children(
    registry: &TypeRegistry,
    info: &TupleStructInfo,
    display_path: &str,
    binding_name: Option<&str>,
    nullable: bool,
) -> Vec<DebugFieldDescriptor> {
    let skipped = skipped_indices(registry, info.type_id());
    info.iter()
        .enumerate()
        .filter(|(index, _)| !skipped.contains(index))
        .map(|(index, field)| {
            let next_display = indexed_path(display_path, Some(index));
            let next_binding = binding_name.map(|current| indexed_path(current, Some(index)));
            build_field_node(
                registry,
                field.type_info(),
                field.ty(),
                &next_display,
                next_binding.as_deref(),
                nullable,
            )
        })
        .collect()
}

fn tuple_children(
    registry: &TypeRegistry,
    info: &TupleInfo,
    display_path: &str,
    binding_name: Option<&str>,
    nullable: bool,
) -> Vec<DebugFieldDescriptor> {
    info.iter()
        .enumerate()
        .map(|(index, field)| {
            let next_display = indexed_path(display_path, Some(index));
            let next_binding = binding_name.map(|current| indexed_path(current, Some(index)));
            build_field_node(
                registry,
                field.type_info(),
                field.ty(),
                &next_display,
                next_binding.as_deref(),
                nullable,
            )
        })
        .collect()
}

fn indexed_children(
    registry: &TypeRegistry,
    item_info: Option<&'static TypeInfo>,
    item_type: &Type,
    display_path: &str,
    binding_name: Option<&str>,
    nullable: bool,
) -> Vec<DebugFieldDescriptor> {
    let next_display = indexed_path(display_path, None);
    let next_binding = binding_name.map(|current| indexed_path(current, None));
    vec![build_field_node(
        registry,
        item_info,
        item_type,
        &next_display,
        next_binding.as_deref(),
        nullable,
    )]
}

fn enum_variant_descriptors(
    registry: &TypeRegistry,
    info: &EnumInfo,
    display_path: &str,
    binding_name: Option<&str>,
    nullable: bool,
) -> Vec<DebugEnumVariantDescriptor> {
    info.iter()
        .map(|variant| {
            let variant_display =
                join_field_path(path_or_value(display_path).as_str(), variant.name());
            let variant_binding = binding_name
                .map(|name| join_field_path(path_or_value(name).as_str(), variant.name()));
            match variant {
                VariantInfo::Unit(_) => DebugEnumVariantDescriptor {
                    name: variant.name().to_owned(),
                    kind: DebugEnumVariantKind::Unit,
                    fields: Vec::new(),
                },
                VariantInfo::Tuple(tuple) => DebugEnumVariantDescriptor {
                    name: variant.name().to_owned(),
                    kind: DebugEnumVariantKind::Tuple,
                    fields: tuple
                        .iter()
                        .enumerate()
                        .map(|(index, field)| {
                            let field_display = indexed_path(&variant_display, Some(index));
                            let field_binding = variant_binding
                                .as_deref()
                                .map(|name| indexed_path(name, Some(index)));
                            build_field_node(
                                registry,
                                field.type_info(),
                                field.ty(),
                                &field_display,
                                field_binding.as_deref(),
                                nullable,
                            )
                        })
                        .collect(),
                },
                VariantInfo::Struct(struct_variant) => DebugEnumVariantDescriptor {
                    name: variant.name().to_owned(),
                    kind: DebugEnumVariantKind::Struct,
                    fields: struct_variant
                        .iter()
                        .map(|field| {
                            let field_display = join_field_path(&variant_display, field.name());
                            let field_binding = variant_binding
                                .as_deref()
                                .map(|name| join_field_path(name, field.name()));
                            build_field_node(
                                registry,
                                field.type_info(),
                                field.ty(),
                                &field_display,
                                field_binding.as_deref(),
                                nullable,
                            )
                        })
                        .collect(),
                },
            }
        })
        .collect()
}

fn skipped_indices(registry: &TypeRegistry, type_id: core::any::TypeId) -> BTreeSet<usize> {
    registry
        .get_type_data::<SerializationData>(type_id)
        .map(|data| data.iter_skipped().map(|(index, _)| *index).collect())
        .unwrap_or_default()
}

fn resolve_type_info_by_path(
    registry: &TypeRegistry,
    type_path: &str,
) -> Option<&'static TypeInfo> {
    registry
        .get_with_type_path(type_path)
        .map(|registration| registration.type_info())
        .or_else(|| {
            registry.iter().find_map(|registration| {
                let registered = registration.type_info().type_path();
                (registered.ends_with(type_path) || type_path.ends_with(registered))
                    .then_some(registration.type_info())
            })
        })
}

fn option_inner_field(info: &EnumInfo) -> Option<(Option<&'static TypeInfo>, &Type)> {
    let none_variant = info.variant("None")?;
    let some_variant = info.variant("Some")?;

    if !matches!(none_variant, VariantInfo::Unit(_)) {
        return None;
    }

    let VariantInfo::Tuple(tuple_variant) = some_variant else {
        return None;
    };
    if tuple_variant.field_len() != 1 {
        return None;
    }

    let field = tuple_variant.field_at(0)?;
    Some((field.type_info(), field.ty()))
}

struct DebugFieldShape {
    semantics: Option<DebugFieldSemantics>,
    scalar_kind: Option<DebugScalarKind>,
    nullable: bool,
    kind: DebugFieldKind,
}

fn debug_field_descriptor(
    display_path: &str,
    binding_name: Option<&str>,
    value_type_path: &str,
    shape: DebugFieldShape,
    children: Vec<DebugFieldDescriptor>,
) -> DebugFieldDescriptor {
    DebugFieldDescriptor {
        display_path: display_path.to_owned(),
        binding_name: binding_name.map(str::to_owned),
        value_type_path: value_type_path.to_owned(),
        scalar_kind: shape.scalar_kind,
        semantics: shape.semantics,
        nullable: shape.nullable,
        kind: shape.kind,
        children,
        map_key: None,
        map_value: None,
        enum_variants: Vec::new(),
    }
}

fn join_field_path(prefix: &str, field: &str) -> String {
    if prefix.is_empty() {
        field.to_owned()
    } else {
        format!("{prefix}.{field}")
    }
}

fn indexed_path(prefix: &str, index: Option<usize>) -> String {
    let base = path_or_value(prefix);
    match index {
        Some(index) => format!("{base}[{index}]"),
        None => format!("{base}[]"),
    }
}

fn path_or_value(path: &str) -> String {
    if path.is_empty() {
        "value".to_owned()
    } else {
        path.to_owned()
    }
}

fn primitive_scalar_kind(type_path: &str) -> Option<DebugScalarKind> {
    match type_path {
        "bool" | "core::primitive::bool" => Some(DebugScalarKind::Bool),
        "i8" | "core::primitive::i8" => Some(DebugScalarKind::I8),
        "i16" | "core::primitive::i16" => Some(DebugScalarKind::I16),
        "i32" | "core::primitive::i32" => Some(DebugScalarKind::I32),
        "i64" | "core::primitive::i64" => Some(DebugScalarKind::I64),
        "i128" | "core::primitive::i128" => Some(DebugScalarKind::I128),
        "isize" | "core::primitive::isize" => Some(DebugScalarKind::Isize),
        "u8" | "core::primitive::u8" => Some(DebugScalarKind::U8),
        "u16" | "core::primitive::u16" => Some(DebugScalarKind::U16),
        "u32" | "core::primitive::u32" => Some(DebugScalarKind::U32),
        "u64" | "core::primitive::u64" => Some(DebugScalarKind::U64),
        "u128" | "core::primitive::u128" => Some(DebugScalarKind::U128),
        "usize" | "core::primitive::usize" => Some(DebugScalarKind::Usize),
        "f32" | "core::primitive::f32" => Some(DebugScalarKind::F32),
        "f64" | "core::primitive::f64" => Some(DebugScalarKind::F64),
        "char" | "core::primitive::char" => Some(DebugScalarKind::Char),
        "alloc::string::String" | "std::string::String" | "str" | "&str" => {
            Some(DebugScalarKind::String)
        }
        _ => None,
    }
}
#[cfg(feature = "reflect")]
fn reflect_value_to_json(value: &dyn crate::reflect::Reflect, registry: &TypeRegistry) -> Value {
    let serializer = crate::reflect::serde::TypedReflectSerializer::new(value, registry);
    cu29_value::to_value(serializer)
        .map(debug_value_to_json)
        .unwrap_or_else(|_| Value::String("reflect serialization failed".to_string()))
}

#[cfg(not(feature = "reflect"))]
fn reflect_value_to_json(_value: &dyn crate::reflect::Reflect, _registry: &TypeRegistry) -> Value {
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

fn default_nav_replay_limit() -> u32 {
    1
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

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::{
        DebugRpcAttachment, DebugRpcResponse, RemoteDebugShmConfig, RemoteDebugShmRole,
        RemoteDebugShmSystemLimits, WireCodec, build_message_metadata_field_descriptors,
        build_output_schema_entries, build_stack_schema, debug_value_to_json, encode_payload,
        metadata_to_json, reflect_value_to_json, register_debug_support_types,
        validate_remote_debug_shm_limits,
    };
    use crate::app::CuSimApplication;
    use crate::curuntime::KeyFrame;
    use crate::cutask::CuMsgMetadata;
    use crate::pool::{CuSharedMemoryElementType, DebugHandleEncoding};
    use crate::reflect::{Reflect, ReflectTaskIntrospection, TypePath, TypeRegistry};
    use crate::simulation::SimOverride;
    use compact_str::CompactString;
    use cu29_clock::{CuTime, CuTimeRange, OptionCuTime, PartialCuTimeRange, Tov};
    use cu29_traits::{
        CuCompactString, CuMsgOrigin, CuResult, DebugFieldKind, DebugScalarKind,
        ErasedCuStampedData, ErasedCuStampedDataSet, MatchingTasks, TaskOutputSpec,
    };
    use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
    use cu29_units::si::f32::Ratio;
    use std::collections::BTreeMap;

    #[test]
    fn shm_preflight_rejects_production_pool_under_eight_mib_memlock() {
        let error = validate_remote_debug_shm_limits(
            RemoteDebugShmConfig::new(1024 * 1024 * 1024, 256 * 1024),
            RemoteDebugShmRole::Server,
            RemoteDebugShmSystemLimits {
                memlock_soft_bytes: 8 * 1024 * 1024,
                memlock_hard_bytes: 8 * 1024 * 1024,
                shm_available_bytes: 32 * 1024 * 1024 * 1024,
            },
        )
        .expect_err("8 MiB must not satisfy the production SHM contract")
        .to_string();

        assert!(error.contains("preflight failed for the server"));
        assert!(error.contains("8.00 MiB/8.00 MiB"));
        assert!(error.contains("1.00 GiB pool"));
        assert!(error.contains("ulimit -l 2097152"));
        assert!(error.contains("Refusing to start"));
    }

    #[test]
    fn shm_preflight_rejects_insufficient_dev_shm() {
        let error = validate_remote_debug_shm_limits(
            RemoteDebugShmConfig::new(4 * 1024 * 1024, 256 * 1024),
            RemoteDebugShmRole::Server,
            RemoteDebugShmSystemLimits {
                memlock_soft_bytes: 8 * 1024 * 1024,
                memlock_hard_bytes: 8 * 1024 * 1024,
                shm_available_bytes: 2 * 1024 * 1024,
            },
        )
        .expect_err("the pool must fit in /dev/shm")
        .to_string();

        assert!(error.contains("/dev/shm has 2.00 MiB available"));
        assert!(error.contains("requires at least 4.00 MiB"));
    }

    #[test]
    fn shm_preflight_accepts_explicit_small_test_profile() {
        validate_remote_debug_shm_limits(
            RemoteDebugShmConfig::new(4 * 1024 * 1024, 256 * 1024),
            RemoteDebugShmRole::Client,
            RemoteDebugShmSystemLimits {
                memlock_soft_bytes: 8 * 1024 * 1024,
                memlock_hard_bytes: 8 * 1024 * 1024,
                shm_available_bytes: 32 * 1024 * 1024,
            },
        )
        .expect("4 MiB pool plus 2 MiB headroom fits an 8 MiB memlock limit");
    }

    #[test]
    fn shm_config_rejects_impossible_profiles() {
        assert!(RemoteDebugShmConfig::new(0, 1).validate().is_err());
        assert!(RemoteDebugShmConfig::new(1, 0).validate().is_err());
        assert!(RemoteDebugShmConfig::new(1, 2).validate().is_err());
    }

    struct MissionStackApp;

    #[derive(Reflect, bincode::Encode, bincode::Decode, serde::Serialize, Default, Debug)]
    struct CanonicalPayload {
        reading: u16,
    }

    type PayloadAlias = CanonicalPayload;

    #[derive(bincode::Encode, bincode::Decode, serde::Serialize, Default, Debug)]
    struct AliasPayloadCopperList;

    impl ErasedCuStampedDataSet for AliasPayloadCopperList {
        fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
            Vec::new()
        }
    }

    impl MatchingTasks for AliasPayloadCopperList {
        fn get_all_task_ids() -> &'static [&'static str] {
            &["alias_task"]
        }

        fn get_output_specs() -> &'static [TaskOutputSpec] {
            &[TaskOutputSpec {
                task_id: "alias_task",
                msg_type: "alias::payload",
                payload_type_path_fn: <PayloadAlias as TypePath>::type_path,
            }]
        }
    }

    impl ReflectTaskIntrospection for MissionStackApp {
        fn reflect_task(&self, _task_id: &str) -> Option<&dyn Reflect> {
            None
        }

        fn reflect_task_mut(&mut self, _task_id: &str) -> Option<&mut dyn Reflect> {
            None
        }
    }

    #[derive(Reflect)]
    struct RawDebugTask {
        hidden: f32,
    }

    #[derive(Reflect)]
    struct CustomDebugState {
        visible: Ratio,
    }

    #[derive(Reflect, serde::Serialize)]
    enum FidelityEnum {
        Unit,
        Tuple(u16, String),
        Struct { code: u8, valid: bool },
    }

    #[derive(Reflect, serde::Serialize)]
    struct FidelityPayload {
        numeric_map: BTreeMap<u16, String>,
        mode: FidelityEnum,
    }

    struct CustomDebugStateApp;

    impl ReflectTaskIntrospection for CustomDebugStateApp {
        fn reflect_task(&self, _task_id: &str) -> Option<&dyn Reflect> {
            None
        }

        fn reflect_task_mut(&mut self, _task_id: &str) -> Option<&mut dyn Reflect> {
            None
        }

        fn register_reflect_types(registry: &mut TypeRegistry) {
            registry.register::<CustomDebugState>();
        }

        fn debug_state_type_path(task_id: &str) -> Option<&'static str> {
            (task_id == "beta_src").then_some(<CustomDebugState as TypePath>::type_path())
        }
    }

    impl CuSimApplication<MmapSectionStorage, MmapUnifiedLoggerWrite> for CustomDebugStateApp {
        type Step<'z> = ();

        fn get_original_config() -> String {
            include_str!("../tests/remote_debug_missions_config.ron").to_string()
        }

        fn mission_id() -> Option<&'static str> {
            Some("Beta")
        }

        fn start_all_tasks(
            &mut self,
            _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            Ok(())
        }

        fn run_one_iteration(
            &mut self,
            _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            Ok(())
        }

        fn run(
            &mut self,
            _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            Ok(())
        }

        fn stop_all_tasks(
            &mut self,
            _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            Ok(())
        }

        fn restore_keyframe(&mut self, _freezer: &KeyFrame) -> CuResult<()> {
            Ok(())
        }
    }

    impl CuSimApplication<MmapSectionStorage, MmapUnifiedLoggerWrite> for MissionStackApp {
        type Step<'z> = ();

        fn get_original_config() -> String {
            include_str!("../tests/remote_debug_missions_config.ron").to_string()
        }

        fn mission_id() -> Option<&'static str> {
            Some("Beta")
        }

        fn start_all_tasks(
            &mut self,
            _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            Ok(())
        }

        fn run_one_iteration(
            &mut self,
            _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            Ok(())
        }

        fn run(
            &mut self,
            _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            Ok(())
        }

        fn stop_all_tasks(
            &mut self,
            _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
        ) -> CuResult<()> {
            Ok(())
        }

        fn restore_keyframe(&mut self, _freezer: &KeyFrame) -> CuResult<()> {
            Ok(())
        }
    }

    #[test]
    fn cbor_handle_attachments_are_encoded_as_byte_strings() -> CuResult<()> {
        let data = vec![0xde, 0xad, 0xbe, 0xef];
        let response = DebugRpcResponse {
            request_id: "binary-attachment".to_owned(),
            ok: true,
            result: Some(serde_json::json!({
                "handle": {
                    "__cu_handle__": true,
                    "attachment_id": 0,
                    "encoding": "raw_little_endian",
                    "element_type": "u8",
                    "len_elements": 4,
                    "byte_len": 4,
                }
            })),
            error: None,
            cursor_rev: None,
            resolved_at: None,
            attachments: vec![DebugRpcAttachment {
                id: 0,
                encoding: DebugHandleEncoding::RawLittleEndian,
                element_type: Some(CuSharedMemoryElementType::U8),
                len_elements: Some(data.len()),
                data: data.clone(),
            }],
        };

        let encoded = encode_payload(&response, WireCodec::Cbor, "encode test response")?;
        let expected_byte_string = [vec![0x44], data].concat();
        assert!(
            encoded
                .windows(expected_byte_string.len())
                .any(|window| window == expected_byte_string),
            "the attachment must use CBOR major type 2 (byte string), not an integer array"
        );

        Ok(())
    }

    #[test]
    fn stack_schema_uses_generated_mission_id() -> CuResult<()> {
        let mission_id = <MissionStackApp as CuSimApplication<
            MmapSectionStorage,
            MmapUnifiedLoggerWrite,
        >>::mission_id();
        assert_eq!(mission_id, Some("Beta"));

        let schema =
            build_stack_schema::<MissionStackApp, MmapSectionStorage, MmapUnifiedLoggerWrite>()?;
        assert_eq!(schema.get("mission_id"), Some(&serde_json::json!("Beta")));

        let mut task_ids: Vec<&str> = schema["tasks"]
            .as_array()
            .expect("tasks array")
            .iter()
            .filter_map(|task| task["id"].as_str())
            .collect();
        task_ids.sort_unstable();
        assert_eq!(task_ids, vec!["beta_bridge", "beta_sink", "beta_src"]);

        let mut bridge_ids: Vec<&str> = schema["bridges"]
            .as_array()
            .expect("bridges array")
            .iter()
            .filter_map(|bridge| bridge["id"].as_str())
            .collect();
        bridge_ids.sort_unstable();
        assert_eq!(bridge_ids, vec!["beta_bridge"]);

        Ok(())
    }

    #[test]
    fn output_schema_resolves_alias_payload_types_via_canonical_type_path() -> CuResult<()> {
        let mut registry = TypeRegistry::default();
        registry.register::<PayloadAlias>();
        register_debug_support_types(&mut registry);

        let outputs = build_output_schema_entries::<AliasPayloadCopperList>(&registry)?;
        assert_eq!(outputs.len(), 1);
        assert_eq!(
            outputs[0]["payload_type_path"].as_str(),
            Some(<PayloadAlias as TypePath>::type_path())
        );
        assert!(
            outputs[0]["payload_fields"]
                .as_array()
                .expect("payload_fields array")
                .iter()
                .any(|field| field["display_path"].as_str() == Some("reading"))
        );

        Ok(())
    }

    #[derive(Reflect)]
    struct QuantityPayload {
        power: Ratio,
    }

    #[test]
    fn output_schema_collapses_registered_scalar_wrappers() {
        let mut registry = TypeRegistry::default();
        registry.register::<QuantityPayload>();
        registry.register::<Ratio>();
        register_debug_support_types(&mut registry);

        let info = registry
            .get_with_type_path(<QuantityPayload as TypePath>::type_path())
            .expect("quantity payload registered")
            .type_info();
        let fields = super::build_field_catalog(&registry, info, None);

        assert!(fields.iter().any(|field| {
            field.display_path == "power"
                && field.scalar_kind == Some(DebugScalarKind::F32)
                && field.value_type_path == <Ratio as TypePath>::type_path()
        }));
        assert!(
            !fields
                .iter()
                .any(|field| field.display_path == "power.value")
        );
    }

    #[test]
    fn debug_state_serialization_uses_typed_shape_without_type_wrapper() {
        let mut registry = TypeRegistry::default();
        registry.register::<RawDebugTask>();
        register_debug_support_types(&mut registry);

        let value = reflect_value_to_json(&RawDebugTask { hidden: 1.25 }, &registry);
        assert_eq!(value["hidden"], serde_json::json!(1.25));
        assert!(value.get(<RawDebugTask as TypePath>::type_path()).is_none());
    }

    #[test]
    fn stack_schema_uses_custom_debug_state_type() -> CuResult<()> {
        let schema =
            build_stack_schema::<CustomDebugStateApp, MmapSectionStorage, MmapUnifiedLoggerWrite>(
            )?;

        let beta_src = schema["tasks"]
            .as_array()
            .expect("tasks array")
            .iter()
            .find(|task| task["id"].as_str() == Some("beta_src"))
            .expect("beta_src task");
        assert_eq!(
            beta_src["state_type_path"].as_str(),
            Some(<CustomDebugState as TypePath>::type_path())
        );
        assert!(
            beta_src["state_fields"]
                .as_array()
                .expect("state fields")
                .iter()
                .any(|field| {
                    field["display_path"].as_str() == Some("visible")
                        && field["scalar_kind"].as_str() == Some("f32")
                })
        );

        Ok(())
    }

    #[test]
    fn metadata_schema_uses_actual_type_paths() -> CuResult<()> {
        let mut registry = TypeRegistry::default();
        register_debug_support_types(&mut registry);

        let fields = build_message_metadata_field_descriptors(&registry)?;
        let tov = fields
            .iter()
            .find(|field| field.display_path == "tov")
            .expect("tov field");
        assert_eq!(tov.kind, DebugFieldKind::Enum);
        assert_eq!(tov.value_type_path, core::any::type_name::<Tov>());

        let process_time = fields
            .iter()
            .find(|field| field.display_path == "process_time")
            .expect("process_time field");
        assert_eq!(process_time.kind, DebugFieldKind::Struct);
        assert_eq!(
            process_time.value_type_path,
            core::any::type_name::<PartialCuTimeRange>()
        );
        assert!(process_time.children.iter().any(|child| {
            child.display_path == "process_time.start"
                && child.value_type_path == core::any::type_name::<OptionCuTime>()
        }));

        let status_txt = fields
            .iter()
            .find(|field| field.display_path == "status_txt")
            .expect("status_txt field");
        assert_eq!(status_txt.kind, DebugFieldKind::Scalar);
        assert_eq!(
            status_txt.value_type_path,
            core::any::type_name::<CuCompactString>()
        );

        let origin = fields
            .iter()
            .find(|field| field.display_path == "origin")
            .expect("origin field");
        assert_eq!(origin.kind, DebugFieldKind::Struct);
        assert_eq!(
            origin.value_type_path,
            core::any::type_name::<CuMsgOrigin>()
        );
        assert!(origin.nullable);

        Ok(())
    }

    #[test]
    fn metadata_json_uses_actual_rust_shapes() {
        let metadata = CuMsgMetadata {
            process_time: PartialCuTimeRange {
                start: OptionCuTime::from(Some(CuTime::from(10u64))),
                end: OptionCuTime::none(),
            },
            status_txt: CuCompactString(CompactString::from("ready")),
            origin: Some(CuMsgOrigin {
                subsystem_code: 7,
                instance_id: 11,
                cl_id: 42,
            }),
        };

        let value = metadata_to_json(
            &metadata,
            Tov::Range(CuTimeRange {
                start: CuTime::from(100u64),
                end: CuTime::from(200u64),
            }),
        );

        assert_eq!(
            value,
            serde_json::json!({
                "tov": {
                    "Range": {
                        "start": 100u64,
                        "end": 200u64,
                    }
                },
                "process_time": {
                    "start": 10u64,
                    "end": OptionCuTime::NONE_SENTINEL_NANOS,
                },
                "status_txt": "ready",
                "origin": {
                    "subsystem_code": 7,
                    "instance_id": 11,
                    "cl_id": 42,
                }
            })
        );
    }

    #[test]
    fn debug_json_preserves_wide_integers_nonfinite_floats_and_numeric_map_keys() {
        #[derive(serde::Serialize)]
        struct LosslessValues {
            i128_min: i128,
            u128_max: u128,
            nan: f32,
            positive_infinity: f64,
            negative_infinity: f64,
            numeric_map: BTreeMap<u16, String>,
        }

        let encoded = cu29_value::to_value(LosslessValues {
            i128_min: i128::MIN,
            u128_max: u128::MAX,
            nan: f32::NAN,
            positive_infinity: f64::INFINITY,
            negative_infinity: f64::NEG_INFINITY,
            numeric_map: BTreeMap::from([(7, "seven".to_owned())]),
        })
        .map(debug_value_to_json)
        .expect("debug value");

        assert_eq!(
            encoded["i128_min"],
            serde_json::json!({"__cu_i128__": i128::MIN.to_string()})
        );
        assert_eq!(
            encoded["u128_max"],
            serde_json::json!({"__cu_u128__": u128::MAX.to_string()})
        );
        assert_eq!(encoded["nan"], serde_json::json!({"__cu_float__": "nan"}));
        assert_eq!(
            encoded["positive_infinity"],
            serde_json::json!({"__cu_float__": "positive_infinity"})
        );
        assert_eq!(
            encoded["negative_infinity"],
            serde_json::json!({"__cu_float__": "negative_infinity"})
        );
        assert_eq!(
            encoded["numeric_map"],
            serde_json::json!({
                "__cu_map__": [{"key": 7, "value": "seven"}]
            })
        );
    }

    #[test]
    fn field_catalog_describes_map_key_value_and_every_enum_variant_shape() {
        let mut registry = TypeRegistry::default();
        registry.register::<FidelityPayload>();
        register_debug_support_types(&mut registry);
        let info = registry
            .get_with_type_path(<FidelityPayload as TypePath>::type_path())
            .expect("fidelity payload registration")
            .type_info();
        let fields = super::build_field_catalog(&registry, info, None);

        let map = fields
            .iter()
            .find(|field| field.display_path == "numeric_map")
            .expect("numeric map field");
        assert_eq!(map.kind, DebugFieldKind::Map);
        assert_eq!(
            map.map_key.as_deref().and_then(|field| field.scalar_kind),
            Some(DebugScalarKind::U16)
        );
        assert_eq!(
            map.map_value.as_deref().and_then(|field| field.scalar_kind),
            Some(DebugScalarKind::String)
        );

        let mode = fields
            .iter()
            .find(|field| field.display_path == "mode")
            .expect("enum field");
        assert_eq!(mode.kind, DebugFieldKind::Enum);
        assert_eq!(
            mode.enum_variants
                .iter()
                .map(|variant| (variant.name.as_str(), variant.fields.len()))
                .collect::<Vec<_>>(),
            vec![("Unit", 0), ("Tuple", 2), ("Struct", 2)]
        );
    }

    #[test]
    fn builtin_debug_schema_supports_compact_status_strings() {
        let mut paths = Vec::new();
        super::append_builtin_debug_type_paths(&mut paths);
        assert!(
            paths
                .iter()
                .any(|path| path == core::any::type_name::<CuCompactString>())
        );

        let schema = super::builtin_debug_type_schema(
            core::any::type_name::<CuCompactString>(),
            "jsonschema",
        )
        .expect("CuCompactString builtin schema");
        assert_eq!(schema["type"], "string");
    }
}
