use cu29::config::{read_configuration_with_features, read_multi_configuration_with_features};
use std::path::{Path, PathBuf};

fn config_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join(name)
}

fn has_default_mcu_task(features: &[&str], task_id: &str) -> bool {
    let config =
        read_configuration_with_features(config_path("mcu_config.ron").to_str().unwrap(), features)
            .unwrap();
    config
        .get_graph(Some("default"))
        .unwrap()
        .get_node_id_by_name(task_id)
        .is_some()
}

fn has_compute_task(features: &[&str], task_id: &str) -> bool {
    let config = read_configuration_with_features(
        config_path("compute_config.ron").to_str().unwrap(),
        features,
    )
    .unwrap();
    config
        .get_graph(None)
        .unwrap()
        .get_node_id_by_name(task_id)
        .is_some()
}

#[test]
fn firmware_monitor_and_recording_are_independent() {
    for (features, monitor, recording) in [
        (vec!["firmware"], false, false),
        (
            vec!["firmware", "textlogs", "log-level-debug"],
            false,
            false,
        ),
        (vec!["firmware", "logmon"], true, false),
        (vec!["firmware", "sdcard-logging"], false, true),
        (vec!["firmware", "logmon", "sdcard-logging"], true, true),
    ] {
        let cfg = read_configuration_with_features(
            config_path("mcu_config.ron").to_str().unwrap(),
            &features,
        )
        .unwrap();
        assert_eq!(cfg.monitors.len(), usize::from(monitor), "{features:?}");
        let logging = cfg.logging.unwrap();
        assert_eq!(logging.enable_task_logging, recording, "{features:?}");
        assert_eq!(logging.enable_keyframe_logging, recording, "{features:?}");
    }
}

#[test]
fn firmware_sd_logging_selects_rate_tuning_payloads() {
    let path = config_path("mcu_config.ron");
    for (features, firmware) in [
        (&["firmware", "sdcard-logging"][..], true),
        (&["sim"][..], false),
    ] {
        let config = read_configuration_with_features(path.to_str().unwrap(), features).unwrap();
        let graph = config.get_graph(Some("default")).unwrap();
        for (task_id, enabled) in [
            ("bmi088", true),
            ("attitude", firmware),
            ("rate", firmware),
            ("mixer0", true),
            ("mixer1", true),
            ("mixer2", true),
            ("mixer3", true),
            ("gnss_ublox", !firmware),
            ("dps310", !firmware),
            ("ist8310", !firmware),
            ("mag_heading", !firmware),
            ("battery_adc", !firmware),
        ] {
            let node = graph
                .get_node(graph.get_node_id_by_name(task_id).unwrap())
                .unwrap();
            assert_eq!(node.is_logging_enabled(), enabled, "{features:?} {task_id}");
        }
    }
}

#[test]
fn mcu_feature_matrix_selects_one_control_graph() {
    for features in [
        &[][..],
        &["firmware"][..],
        &["bevymon"][..],
        &["python-bindings"][..],
    ] {
        assert!(has_default_mcu_task(features, "attitude"));
        assert!(!has_default_mcu_task(features, "mode_supervisor"));
    }

    for features in [
        &["sim"][..],
        &["sim", "sim-debug"][..],
        &["end2end"][..],
        &["logreader"][..],
    ] {
        assert!(has_default_mcu_task(features, "attitude"));
        assert!(has_default_mcu_task(features, "mode_supervisor"));
    }
}

#[test]
fn compute_feature_matrix_selects_one_vision_graph() {
    for features in [&[][..], &["firmware"][..], &["bevymon"][..]] {
        assert!(has_compute_task(features, "vitfly"));
        assert!(!has_compute_task(features, "vitfly_context"));
    }

    for features in [&["sim"][..], &["end2end"][..], &["compute-logreader"][..]] {
        assert!(has_compute_task(features, "vitfly"));
        assert!(has_compute_task(features, "vitfly_context"));
    }
}

#[test]
fn deployment_interconnects_follow_the_autonomy_features() {
    let path = config_path("flight_controller.ron");
    for features in [&[][..], &["firmware"][..], &["bevymon"][..]] {
        let config =
            read_multi_configuration_with_features(path.to_str().unwrap(), features).unwrap();
        assert!(config.interconnects.is_empty());
    }

    for features in [&["sim"][..], &["end2end"][..]] {
        let config =
            read_multi_configuration_with_features(path.to_str().unwrap(), features).unwrap();
        assert_eq!(config.interconnects.len(), 2);
    }
}
