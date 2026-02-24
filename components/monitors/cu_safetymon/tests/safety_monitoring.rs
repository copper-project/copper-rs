use cu_logmon::CuLogMon;
use cu_safetymon::CuSafetyMon;
use cu29::prelude::*;
use std::process::{Command, Output};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

const TASK_IDS: &[&str] = &["planner", "driver"];
const CHILD_MODE_ENV: &str = "CU_SAFETYMON_CHILD_MODE";

fn config_with_single_monitor(type_name: &str, extra_config: &str) -> CuConfig {
    let ron = format!(
        r#"
(
    tasks: [],
    cnx: [],
    monitors: [(
        type: "{type_name}",
        config: {{
            {extra_config}
        }},
    )],
)
"#
    );
    CuConfig::deserialize_ron(&ron).expect("failed to parse monitor config")
}

fn safetymon_test_config(exit_lock: i32, exit_panic: i32) -> CuConfig {
    config_with_single_monitor(
        "cu_safetymon::CuSafetyMon",
        &format!(
            r#"
            "copperlist_deadline_ms": 40,
            "watchdog_period_ms": 10,
            "exit_code_shutdown": 65,
            "exit_code_lock": {exit_lock},
            "exit_code_panic": {exit_panic},
        "#
        ),
    )
}

fn spawn_current_test(test_name: &str, mode: &str) -> Output {
    let exe = std::env::current_exe().expect("unable to locate current test executable");
    Command::new(exe)
        .arg("--exact")
        .arg(test_name)
        .arg("--nocapture")
        .arg("--test-threads=1")
        .env(CHILD_MODE_ENV, mode)
        .output()
        .expect("failed to spawn child test process")
}

#[test]
fn cu_logmon_and_cu_safetymon_can_run_together() {
    let log_cfg = config_with_single_monitor("cu_logmon::CuLogMon", "");
    let safe_cfg = safetymon_test_config(79, 80);

    let mut monitors = (
        CuLogMon::new(&log_cfg, TASK_IDS).expect("logmon new"),
        CuSafetyMon::new(&safe_cfg, TASK_IDS).expect("safetymon new"),
    );
    let probe = Arc::new(RuntimeExecutionProbe::default());
    monitors.set_execution_probe(probe.clone());

    let clock = RobotClock::new();
    let meta = CuMsgMetadata::default();
    let msgs: [&CuMsgMetadata; 1] = [&meta];

    monitors.start(&clock).expect("monitors start");
    probe.record(ExecutionMarker {
        component_id: 0,
        step: CuTaskState::Process,
        culistid: Some(1),
    });
    monitors
        .process_copperlist(&msgs)
        .expect("monitors process_copperlist");
    monitors.stop(&clock).expect("monitors stop");
}

#[test]
fn contradictory_decisions_between_logmon_and_safetymon_shutdown() {
    let log_cfg = config_with_single_monitor("cu_logmon::CuLogMon", "");
    let safe_cfg = safetymon_test_config(79, 80);

    let monitors = (
        CuLogMon::new(&log_cfg, TASK_IDS).expect("logmon new"),
        CuSafetyMon::new(&safe_cfg, TASK_IDS).expect("safetymon new"),
    );

    let decision = monitors.process_error(0, CuTaskState::Process, &CuError::from("fault"));
    assert!(
        matches!(decision, Decision::Shutdown),
        "expected Shutdown when monitors disagree"
    );
}

#[test]
fn panic_fault_exits_with_configured_code_and_marker_context() {
    if std::env::var(CHILD_MODE_ENV).ok().as_deref() == Some("panic") {
        let cfg = safetymon_test_config(79, 80);
        let mut monitor = CuSafetyMon::new(&cfg, TASK_IDS).expect("safetymon new");
        let probe = Arc::new(RuntimeExecutionProbe::default());
        monitor.set_execution_probe(probe.clone());
        probe.record(ExecutionMarker {
            component_id: 1,
            step: CuTaskState::Process,
            culistid: Some(33),
        });
        monitor.start(&RobotClock::new()).expect("safetymon start");
        panic!("panic path test");
    }

    let output = spawn_current_test(
        "panic_fault_exits_with_configured_code_and_marker_context",
        "panic",
    );
    assert_eq!(
        output.status.code(),
        Some(80),
        "unexpected panic child exit status: {:?}",
        output.status
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("cu_safetymon panic fault:"));
    assert!(stderr.contains("component=1"));
    assert!(stderr.contains("culist=Some(33)"));
}

#[test]
fn lock_fault_exits_with_configured_code_and_last_marker() {
    if std::env::var(CHILD_MODE_ENV).ok().as_deref() == Some("lock") {
        let cfg = safetymon_test_config(79, 80);
        let mut monitor = CuSafetyMon::new(&cfg, TASK_IDS).expect("safetymon new");
        let probe = Arc::new(RuntimeExecutionProbe::default());
        monitor.set_execution_probe(probe.clone());
        monitor.start(&RobotClock::new()).expect("safetymon start");
        probe.record(ExecutionMarker {
            component_id: 1,
            step: CuTaskState::Process,
            culistid: Some(9),
        });

        thread::sleep(Duration::from_millis(300));
        std::process::exit(254);
    }

    let output = spawn_current_test(
        "lock_fault_exits_with_configured_code_and_last_marker",
        "lock",
    );
    assert_eq!(
        output.status.code(),
        Some(79),
        "unexpected lock child exit status: {:?}",
        output.status
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("cu_safetymon lock fault:"));
    assert!(stderr.contains("component='driver'"));
    assert!(stderr.contains("culist=9"));
}
