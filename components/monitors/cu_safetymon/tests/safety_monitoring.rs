use cu_logmon::CuLogMon;
use cu_safetymon::CuSafetyMon;
use cu29::prelude::*;
use std::process::{Command, Output};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

const MONITORED_COMPONENTS: &[MonitorComponentMetadata] = &[
    MonitorComponentMetadata::new("planner", ComponentType::Task, None),
    MonitorComponentMetadata::new("driver", ComponentType::Task, None),
];
const CULIST_COMPONENT_MAPPING: &[ComponentId] = &[ComponentId::new(0)];
const CHILD_MODE_ENV: &str = "CU_SAFETYMON_CHILD_MODE";

fn monitor_metadata(
    config: &CuConfig,
    probe: Option<Arc<RuntimeExecutionProbe>>,
) -> (CuMonitoringMetadata, CuMonitoringRuntime) {
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
    let runtime = CuMonitoringRuntime::new(MonitorExecutionProbe::from_shared(
        probe.unwrap_or_else(|| Arc::new(RuntimeExecutionProbe::default())),
    ));
    (metadata, runtime)
}

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

fn safetymon_test_config_with_timing(
    copperlist_deadline_ms: u64,
    watchdog_period_ms: u64,
    exit_lock: i32,
    exit_panic: i32,
) -> CuConfig {
    config_with_single_monitor(
        "cu_safetymon::CuSafetyMon",
        &format!(
            r#"
            "copperlist_deadline_ms": {copperlist_deadline_ms},
            "watchdog_period_ms": {watchdog_period_ms},
            "exit_code_shutdown": 65,
            "exit_code_lock": {exit_lock},
            "exit_code_panic": {exit_panic},
        "#
        ),
    )
}

fn safetymon_test_config(exit_lock: i32, exit_panic: i32) -> CuConfig {
    safetymon_test_config_with_timing(40, 10, exit_lock, exit_panic)
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
    let safe_cfg = safetymon_test_config_with_timing(1000, 50, 79, 80);
    let probe = Arc::new(RuntimeExecutionProbe::default());
    let (log_meta, log_runtime) = monitor_metadata(&log_cfg, Some(probe.clone()));
    let (safe_meta, safe_runtime) = monitor_metadata(&safe_cfg, Some(probe.clone()));
    let layout = log_meta.layout();

    let mut monitors = (
        CuLogMon::new(log_meta, log_runtime).expect("logmon new"),
        CuSafetyMon::new(safe_meta, safe_runtime).expect("safetymon new"),
    );

    let (ctx, _clock_control) = CuContext::new_mock_clock();
    let meta = CuMsgMetadata::default();
    let msgs: [&CuMsgMetadata; 1] = [&meta];

    monitors.start(&ctx).expect("monitors start");
    probe.record(ExecutionMarker {
        component_id: ComponentId::new(0),
        step: CuComponentState::Process,
        culistid: Some(1),
    });
    monitors
        .process_copperlist(&ctx, layout.view(&msgs))
        .expect("monitors process_copperlist");
    monitors.stop(&ctx).expect("monitors stop");
}

#[test]
fn contradictory_decisions_between_logmon_and_safetymon_shutdown() {
    let log_cfg = config_with_single_monitor("cu_logmon::CuLogMon", "");
    let safe_cfg = safetymon_test_config(79, 80);
    let (log_meta, log_runtime) = monitor_metadata(&log_cfg, None);
    let (safe_meta, safe_runtime) = monitor_metadata(&safe_cfg, None);

    let monitors = (
        CuLogMon::new(log_meta, log_runtime).expect("logmon new"),
        CuSafetyMon::new(safe_meta, safe_runtime).expect("safetymon new"),
    );

    let decision = monitors.process_error(
        ComponentId::new(0),
        CuComponentState::Process,
        &CuError::from("fault"),
    );
    assert!(
        matches!(decision, Decision::Shutdown),
        "expected Shutdown when monitors disagree"
    );
}

#[test]
fn panic_fault_exits_with_configured_code_and_marker_context() {
    if std::env::var(CHILD_MODE_ENV).ok().as_deref() == Some("panic") {
        let cfg = safetymon_test_config(79, 80);
        let probe = Arc::new(RuntimeExecutionProbe::default());
        let (metadata, runtime) = monitor_metadata(&cfg, Some(probe.clone()));
        let layout = metadata.layout();
        let mut monitor = CuSafetyMon::new(metadata, runtime).expect("safetymon new");
        probe.record(ExecutionMarker {
            component_id: ComponentId::new(1),
            step: CuComponentState::Process,
            culistid: Some(33),
        });
        let (ctx, _clock_control) = CuContext::new_mock_clock();
        monitor.start(&ctx).expect("safetymon start");
        let cl_ctx = CuContext::builder(ctx.clock.clone()).cl_id(33).build();
        let metadata = CuMsgMetadata::default();
        let msgs: [&CuMsgMetadata; 1] = [&metadata];
        monitor
            .process_copperlist(&cl_ctx, layout.view(&msgs))
            .expect("safetymon process_copperlist");
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
    assert!(stderr.contains("last_culist=33"));
}

#[test]
fn lock_fault_exits_with_configured_code_and_last_marker() {
    if std::env::var(CHILD_MODE_ENV).ok().as_deref() == Some("lock") {
        let cfg = safetymon_test_config(79, 80);
        let probe = Arc::new(RuntimeExecutionProbe::default());
        let (metadata, runtime) = monitor_metadata(&cfg, Some(probe.clone()));
        let layout = metadata.layout();
        let mut monitor = CuSafetyMon::new(metadata, runtime).expect("safetymon new");
        let (ctx, _clock_control) = CuContext::new_mock_clock();
        monitor.start(&ctx).expect("safetymon start");
        let cl_ctx = CuContext::builder(ctx.clock.clone()).cl_id(9).build();
        let metadata = CuMsgMetadata::default();
        let msgs: [&CuMsgMetadata; 1] = [&metadata];
        monitor
            .process_copperlist(&cl_ctx, layout.view(&msgs))
            .expect("safetymon process_copperlist");
        probe.record(ExecutionMarker {
            component_id: ComponentId::new(1),
            step: CuComponentState::Process,
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
    assert!(stderr.contains("last_culist=9"));
}

#[test]
fn stop_does_not_trigger_lock_fault_after_shutdown_requested() {
    if std::env::var(CHILD_MODE_ENV).ok().as_deref() == Some("stop-race") {
        for _ in 0..10 {
            let cfg = safetymon_test_config_with_timing(10, 100, 79, 80);
            let probe = Arc::new(RuntimeExecutionProbe::default());
            let (metadata, runtime) = monitor_metadata(&cfg, Some(probe));
            let mut monitor = CuSafetyMon::new(metadata, runtime).expect("safetymon new");
            let (ctx, _clock_control) = CuContext::new_mock_clock();
            monitor.start(&ctx).expect("safetymon start");
            thread::sleep(Duration::from_millis(20));
            monitor.stop(&ctx).expect("safetymon stop");
        }
        return;
    }

    let output = spawn_current_test(
        "stop_does_not_trigger_lock_fault_after_shutdown_requested",
        "stop-race",
    );
    assert_eq!(
        output.status.code(),
        Some(0),
        "unexpected stop-race child exit status: {:?}\nstderr:\n{}",
        output.status,
        String::from_utf8_lossy(&output.stderr)
    );
}
