use super::{
    DebugRpcAttachment, DebugRpcResponse, MAX_ACTIVE_SESSIONS, RemoteDebugShmConfig,
    RemoteDebugShmRole, RemoteDebugShmSystemLimits, SESSION_HEARTBEAT_INTERVAL,
    SESSION_LEASE_TIMEOUT, SessionLifecycleLimits, WireCodec,
    build_message_metadata_field_descriptors, build_output_schema_entries, build_stack_schema,
    capabilities_json, debug_value_to_json, encode_payload, metadata_to_json,
    reflect_value_to_json, register_debug_support_types, validate_remote_debug_shm_limits,
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
    CuCompactString, CuMsgOrigin, CuResult, DebugFieldKind, DebugScalarKind, ErasedCuStampedData,
    ErasedCuStampedDataSet, MatchingTasks, TaskOutputSpec,
};
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29_units::si::f32::Ratio;
use std::collections::BTreeMap;
use std::time::{Duration, Instant};

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

#[test]
fn copperlist_snapshot_omits_runtime_state() {
    let mut list = crate::copperlist::CopperList::new(7, AliasPayloadCopperList);
    list.change_state(crate::copperlist::CopperListState::Processing);
    let snapshot =
        super::copperlist_snapshot(&list, &|_| None, true, true, true, false, None).unwrap();
    assert!(snapshot.get("state").is_none());
    assert_eq!(snapshot["cl"], 7);
    assert_eq!(snapshot["raw_bincode_hex"], "07");
}

impl MatchingTasks for AliasPayloadCopperList {
    fn get_all_task_ids() -> &'static [&'static str] {
        &["alias_task"]
    }

    fn get_output_specs() -> &'static [TaskOutputSpec] {
        const OUTPUT_SPECS: &[TaskOutputSpec] = &[TaskOutputSpec::new::<PayloadAlias>(
            "alias_task",
            "alias::payload",
        )];
        OUTPUT_SPECS
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
        include_str!("../../tests/remote_debug_missions_config.ron").to_string()
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
        include_str!("../../tests/remote_debug_missions_config.ron").to_string()
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
fn replay_session_lease_allows_only_one_active_session() {
    let lifecycle = SessionLifecycleLimits::default();

    assert_eq!(lifecycle.max_sessions, MAX_ACTIVE_SESSIONS);
    assert_eq!(lifecycle.max_sessions, 1);
    assert!(lifecycle.can_open(0));
    assert!(!lifecycle.can_open(1));
}

#[test]
fn replay_session_lease_expires_three_seconds_after_last_heartbeat() {
    let lifecycle = SessionLifecycleLimits::default();
    let opened_at = Instant::now();
    let heartbeat_at = opened_at + SESSION_HEARTBEAT_INTERVAL;

    assert_eq!(lifecycle.lease_timeout, SESSION_LEASE_TIMEOUT);
    assert!(!lifecycle.is_expired(
        heartbeat_at,
        heartbeat_at + SESSION_LEASE_TIMEOUT - Duration::from_millis(1)
    ));
    assert!(
        lifecycle.is_expired(heartbeat_at, heartbeat_at + SESSION_LEASE_TIMEOUT),
        "the lease must expire at the advertised timeout boundary"
    );
}

#[test]
fn expired_replay_session_allows_immediate_reconnect() {
    let lifecycle = SessionLifecycleLimits::default();
    let opened_at = Instant::now();
    let mut active_sessions = vec![opened_at];

    active_sessions.retain(|last_touched_at| {
        !lifecycle.is_expired(*last_touched_at, opened_at + SESSION_LEASE_TIMEOUT)
    });

    assert!(active_sessions.is_empty());
    assert!(lifecycle.can_open(active_sessions.len()));
}

#[test]
fn capabilities_advertise_single_session_heartbeat_lease() {
    let capabilities = capabilities_json(SessionLifecycleLimits::default());
    let lifecycle = &capabilities["session_lifecycle"];

    assert_eq!(lifecycle["max_sessions"], serde_json::json!(1));
    assert_eq!(lifecycle["lease_timeout_ms"], serde_json::json!(3_000));
    assert_eq!(lifecycle["heartbeat_interval_ms"], serde_json::json!(1_000));
    assert_eq!(lifecycle["heartbeat_required"], serde_json::json!(true));
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
        build_stack_schema::<CustomDebugStateApp, MmapSectionStorage, MmapUnifiedLoggerWrite>()?;

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

    let schema =
        super::builtin_debug_type_schema(core::any::type_name::<CuCompactString>(), "jsonschema")
            .expect("CuCompactString builtin schema");
    assert_eq!(schema["type"], "string");
}
