use crate::CuSafetyMon;
use cu29::prelude::*;

const MONITORED_COMPONENTS: &[MonitorComponentMetadata] = &[
    MonitorComponentMetadata::new("planner", ComponentType::Task, None),
    MonitorComponentMetadata::new("driver", ComponentType::Task, None),
];
const CULIST_COMPONENT_MAPPING: &[ComponentId] = &[ComponentId::new(0)];

fn config_with_single_monitor(extra_config: &str) -> CuConfig {
    let ron = format!(
        r#"
(
    tasks: [],
    cnx: [],
    monitors: [(
        type: "cu_safetymon::CuSafetyMon",
        config: {{
            {extra_config}
        }},
    )],
)
"#
    );
    CuConfig::deserialize_ron(&ron).expect("failed to parse monitor config")
}

fn monitor_metadata(config: &CuConfig) -> (CuMonitoringMetadata, CuMonitoringRuntime) {
    let metadata = CuMonitoringMetadata::new(
        DEFAULT_MISSION_ID.into(),
        MONITORED_COMPONENTS,
        CULIST_COMPONENT_MAPPING,
        CopperListInfo::new(0, 0),
        MonitorTopology::default(),
        config
            .get_monitor_configs()
            .first()
            .and_then(|entry| entry.get_config().cloned()),
    )
    .expect("valid monitor metadata");
    let runtime = CuMonitoringRuntime::default();
    (metadata, runtime)
}

fn new_monitor(extra_config: &str) -> CuResult<CuSafetyMon> {
    let config = config_with_single_monitor(extra_config);
    let (metadata, runtime) = monitor_metadata(&config);
    CuSafetyMon::new(metadata, runtime)
}

#[cfg_attr(test, test)]
#[safety_case("SMON-TEST-001")]
fn safety_monitor_rejects_non_positive_watchdog_timing() {
    let deadline_error = match new_monitor(r#""copperlist_deadline_ms": 0,"#) {
        Ok(_) => panic!("expected invalid copperlist_deadline_ms to fail"),
        Err(error) => error,
    };
    safety_check!(
        "SMON-TEST-001-C1",
        "SMON-REQ-001",
        deadline_error
            .to_string()
            .contains("copperlist_deadline_ms must be > 0"),
    );

    let period_error = match new_monitor(
        r#"
        "copperlist_deadline_ms": 100,
        "watchdog_period_ms": 0,
    "#,
    ) {
        Ok(_) => panic!("expected invalid watchdog_period_ms to fail"),
        Err(error) => error,
    };
    safety_check!(
        "SMON-TEST-001-C2",
        "SMON-REQ-001",
        period_error
            .to_string()
            .contains("watchdog_period_ms must be > 0"),
    );
}

#[cfg_attr(test, test)]
#[safety_case("SMON-TEST-002")]
fn safety_monitor_accepts_default_and_configured_fault_codes() {
    safety_check!("SMON-TEST-002-C1", "SMON-REQ-002", new_monitor("").is_ok(),);

    safety_check!(
        "SMON-TEST-002-C2",
        "SMON-REQ-002",
        new_monitor(
            r#"
            "copperlist_deadline_ms": 10,
            "watchdog_period_ms": 5,
            "exit_code_shutdown": 65,
            "exit_code_lock": 66,
            "exit_code_panic": 67,
        "#
        )
        .is_ok(),
    );
}

#[cfg_attr(test, test)]
#[safety_case("SMON-TEST-003")]
fn safety_monitor_turns_runtime_errors_into_shutdown_decisions() {
    let monitor = new_monitor(
        r#"
        "copperlist_deadline_ms": 1000,
        "watchdog_period_ms": 100,
        "exit_code_shutdown": 65,
    "#,
    )
    .expect("safety monitor");

    let decision = monitor.process_error(
        ComponentId::new(1),
        CuComponentState::Process,
        &CuError::from("fault"),
    );
    safety_check!(
        "SMON-TEST-003-C1",
        "SMON-REQ-003",
        matches!(decision, Decision::Shutdown),
    );
}

#[cfg(feature = "safety-ids")]
pub fn link_safety_ids() {
    let _ = safety_monitor_rejects_non_positive_watchdog_timing as fn();
    let _ = safety_monitor_accepts_default_and_configured_fault_codes as fn();
    let _ = safety_monitor_turns_runtime_errors_into_shutdown_decisions as fn();
}
