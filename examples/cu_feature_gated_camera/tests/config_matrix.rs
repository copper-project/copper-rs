use cu29::config::read_configuration_with_resolved_ron_and_features;
use std::path::PathBuf;

fn config_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("copperconfig.ron")
}

#[test]
fn feature_off_excludes_camera_tasks_connection_and_message() {
    let (config, resolved) =
        read_configuration_with_resolved_ron_and_features(config_path().to_str().unwrap(), &[])
            .unwrap();
    let graph = config.get_graph(None).unwrap();

    assert_eq!(graph.node_count(), 2);
    assert_eq!(graph.edge_count(), 1);
    assert!(!resolved.contains("cu_gstreamer"));
    assert!(!resolved.contains("JetsonFrameSink"));
    assert!(!resolved.contains("jetson_mipi.ron"));
}

#[test]
fn feature_on_includes_camera_tasks_connection_and_message() {
    let (config, resolved) = read_configuration_with_resolved_ron_and_features(
        config_path().to_str().unwrap(),
        &["jetson-mipi"],
    )
    .unwrap();
    let graph = config.get_graph(None).unwrap();

    assert_eq!(graph.node_count(), 4);
    assert_eq!(graph.edge_count(), 2);
    assert!(resolved.contains("cu_gstreamer::CuDefaultGStreamer"));
    assert!(resolved.contains("cu_gstreamer::CuGstBuffer"));
    assert!(resolved.contains("tasks::JetsonFrameSink"));
    assert!(!resolved.contains("jetson_mipi.ron"));
    assert!(!resolved.contains("Feature"));
}
