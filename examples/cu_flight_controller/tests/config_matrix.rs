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
fn mcu_feature_matrix_selects_one_control_graph() {
    for features in [
        &[][..],
        &["firmware"][..],
        &["bevymon"][..],
        &["python-bindings"][..],
        &["sim", "sim-debug"][..],
    ] {
        assert!(has_default_mcu_task(features, "attitude"));
        assert!(!has_default_mcu_task(features, "mode_supervisor"));
    }

    for features in [&["sim"][..], &["end2end"][..], &["logreader"][..]] {
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
