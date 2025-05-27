use cu29_runtime::config::{read_configuration, Outgoing};
use petgraph::visit::EdgeRef;
use std::fs::{create_dir_all, write};
use tempfile::tempdir;

#[test]
fn test_basic_include() {
    let temp_dir = tempdir().unwrap();
    let base_path = temp_dir.path();

    let config_dir = base_path.join("config");
    create_dir_all(&config_dir).unwrap();

    let included_config = r#"(
        tasks: [
            (
                id: "included_task",
                type: "tasks::IncludedTask",
            ),
        ],
        cnx: [],
    )"#;

    let included_path = config_dir.join("included.ron");
    write(&included_path, included_config).unwrap();

    let main_config = r#"(
            tasks: [
                (
                    id: "main_task",
                    type: "tasks::MainTask",
                ),
            ],
            cnx: [],
            includes: [
                (
                    path: "included.ron",
                    params: {},
                ),
            ],
        )"#
    .to_string();

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    let config = read_configuration(main_path.to_str().unwrap()).unwrap();
    let graph = config.get_graph(None).unwrap();

    let all_nodes = graph.get_all_nodes();
    assert_eq!(all_nodes.len(), 2);

    let task_ids: Vec<_> = all_nodes
        .iter()
        .map(|(_, node)| node.get_id().to_string())
        .collect();
    assert!(task_ids.contains(&"main_task".to_string()));
    assert!(task_ids.contains(&"included_task".to_string()));
}

#[test]
fn test_parameter_substitution() {
    let temp_dir = tempdir().unwrap();
    let base_path = temp_dir.path();

    let config_dir = base_path.join("config");
    create_dir_all(&config_dir).unwrap();

    let included_config = r#"(
        tasks: [
            (
                id: "task_{{instance_id}}",
                type: "tasks::Task{{instance_id}}",
                config: {
                    "param_value": {{param_value}},
                },
            ),
        ],
        cnx: [],
    )"#;

    let included_path = config_dir.join("included.ron");
    write(&included_path, included_config).unwrap();

    let main_config = r#"(
            tasks: [],
            cnx: [],
            includes: [
                (
                    path: "included.ron",
                    params: {
                        "instance_id": "42",
                        "param_value": 100,
                    },
                ),
            ],
        )"#
    .to_string();

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    let config = read_configuration(main_path.to_str().unwrap()).unwrap();
    let graph = config.get_graph(None).unwrap();

    let all_nodes = graph.get_all_nodes();
    assert_eq!(all_nodes.len(), 1);

    let (_, node) = all_nodes[0];
    assert_eq!(node.get_id(), "task_42");
    assert_eq!(node.get_type(), "tasks::Task42");
    assert_eq!(node.get_param::<i32>("param_value").unwrap(), 100);
}

#[test]
fn test_nested_includes() {
    let temp_dir = tempdir().unwrap();
    let base_path = temp_dir.path();

    let config_dir = base_path.join("config");
    create_dir_all(&config_dir).unwrap();

    let nested_config = r#"(
        tasks: [
            (
                id: "nested_task",
                type: "tasks::NestedTask",
            ),
        ],
        cnx: [],
    )"#;

    let nested_path = config_dir.join("nested.ron");
    write(&nested_path, nested_config).unwrap();

    let middle_config = r#"(
            tasks: [
                (
                    id: "middle_task",
                    type: "tasks::MiddleTask",
                ),
            ],
            cnx: [],
            includes: [
                (
                    path: "nested.ron",
                    params: {},
                ),
            ],
        )"#
    .to_string();

    let middle_path = config_dir.join("middle.ron");
    write(&middle_path, middle_config).unwrap();

    let main_config = r#"(
            tasks: [
                (
                    id: "main_task",
                    type: "tasks::MainTask",
                ),
            ],
            cnx: [],
            includes: [
                (
                    path: "middle.ron",
                    params: {},
                ),
            ],
        )"#
    .to_string();

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    let config = read_configuration(main_path.to_str().unwrap()).unwrap();
    let graph = config.get_graph(None).unwrap();

    let all_nodes = graph.get_all_nodes();
    assert_eq!(all_nodes.len(), 3);

    let task_ids: Vec<_> = all_nodes
        .iter()
        .map(|(_, node)| node.get_id().to_string())
        .collect();
    assert!(task_ids.contains(&"main_task".to_string()));
    assert!(task_ids.contains(&"middle_task".to_string()));
    assert!(task_ids.contains(&"nested_task".to_string()));
}

#[test]
fn test_override_behavior() {
    let temp_dir = tempdir().unwrap();
    let base_path = temp_dir.path();

    let config_dir = base_path.join("config");
    create_dir_all(&config_dir).unwrap();

    let included_config = r#"(
        tasks: [
            (
                id: "common_task",
                type: "tasks::IncludedTask",
                config: {
                    "param": 10,
                },
            ),
            (
                id: "only_included_task",
                type: "tasks::OnlyIncludedTask",
            ),
        ],
        cnx: [],
        monitor: (
            type: "tasks::IncludedMonitor",
        ),
    )"#;

    let included_path = config_dir.join("included.ron");
    write(&included_path, included_config).unwrap();

    let main_config = r#"(
            tasks: [
                (
                    id: "common_task",
                    type: "tasks::MainTask",
                    config: {
                        "param": 20,
                    },
                ),
                (
                    id: "only_main_task",
                    type: "tasks::OnlyMainTask",
                ),
            ],
            cnx: [],
            monitor: (
                type: "tasks::MainMonitor",
            ),
            includes: [
                (
                    path: "included.ron",
                    params: {},
                ),
            ],
        )"#
    .to_string();

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    let config = read_configuration(main_path.to_str().unwrap()).unwrap();
    let graph = config.get_graph(None).unwrap();

    let all_nodes = graph.get_all_nodes();
    assert_eq!(all_nodes.len(), 3);

    let task_ids: Vec<_> = all_nodes
        .iter()
        .map(|(_, node)| node.get_id().to_string())
        .collect();
    assert!(task_ids.contains(&"common_task".to_string()));
    assert!(task_ids.contains(&"only_included_task".to_string()));
    assert!(task_ids.contains(&"only_main_task".to_string()));

    let common_task = all_nodes
        .iter()
        .find(|(_, node)| node.get_id() == "common_task")
        .unwrap()
        .1;

    assert_eq!(common_task.get_type(), "tasks::MainTask");
    assert_eq!(common_task.get_param::<i32>("param").unwrap(), 20);

    assert_eq!(
        config.monitor.as_ref().unwrap().get_type(),
        "tasks::MainMonitor"
    );
}

#[test]
fn test_error_handling() {
    let temp_dir = tempdir().unwrap();
    let base_path = temp_dir.path();

    let config_dir = base_path.join("config");
    create_dir_all(&config_dir).unwrap();

    let main_config = r#"(
        tasks: [],
        cnx: [],
        includes: [
            (
                path: "non_existent.ron",
                params: {},
            ),
        ],
    )"#;

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    let result = read_configuration(main_path.to_str().unwrap());
    assert!(result.is_err());

    let invalid_config = r#"(
        tasks: (
            (
                id: "invalid_task",
                type: "tasks::InvalidTask",
            ),
        ),
        cnx: [],
    )"#;

    let invalid_path = config_dir.join("invalid.ron");
    write(&invalid_path, invalid_config).unwrap();

    let main_config = r#"(
            tasks: [],
            cnx: [],
            includes: [
                (
                    path: "invalid.ron",
                    params: {},
                ),
            ],
        )"#
    .to_string();

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    let result = read_configuration(main_path.to_str().unwrap());
    assert!(result.is_err());
}

#[test]
fn test_multiple_parameterized_includes() {
    let temp_dir = tempdir().unwrap();
    let base_path = temp_dir.path();

    let config_dir = base_path.join("config");
    create_dir_all(&config_dir).unwrap();

    // Create sensor-detect.ron template file
    let sensor_detect_config = r#"(
        tasks: [
            (
                id: "camera{{id}}",
                type: "cu_camera::Camera",
                config: {
                    "port": "{{port}}",
                },
            ),
            (
                id: "detect{{id}}",
                type: "cu_detector::Detector",
                config: {
                    "threshold": {{threshold}},
                },
            ),
        ],
        cnx: [
            (src: "camera{{id}}", dst: "detect{{id}}", msg: "cu_camera::CameraPayload"),
        ],
    )"#;

    let sensor_detect_path = config_dir.join("sensor-detect.ron");
    write(&sensor_detect_path, sensor_detect_config).unwrap();

    // Create main.ron file with multiple includes
    let main_config = r#"(
        tasks: [
            (
                id: "octopus",
                type: "cu_octopus::octopus0",
            ),
        ],
        includes: [
            ( path: "sensor-detect.ron", params: { "id": 0, "port": "/dev/video0", "threshold": 0.5, },),
            ( path: "sensor-detect.ron", params: { "id": 1, "port": "/dev/video1", "threshold": 0.1, },),
            ( path: "sensor-detect.ron", params: { "id": 2, "port": "/dev/video2", "threshold": 0.7, },),
        ],
        cnx: [
            (src: "detect0", dst: "octopus", msg: "cu_detect::DetectionPayload"),
            (src: "detect1", dst: "octopus", msg: "cu_detect::DetectionPayload"),
            (src: "detect2", dst: "octopus", msg: "cu_detect::DetectionPayload"),
        ],
        monitor: (
            type: "cu_consolemon::CuConsoleMon",
        )
    )"#;

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    // Parse the configuration and verify
    let config = read_configuration(main_path.to_str().unwrap()).unwrap();
    let graph = config.get_graph(None).unwrap();

    // Verify tasks
    let all_nodes = graph.get_all_nodes();
    assert_eq!(all_nodes.len(), 7); // 1 octopus + 3 cameras + 3 detectors

    // Verify octopus task exists
    let octopus_task = all_nodes
        .iter()
        .find(|(_, node)| node.get_id() == "octopus")
        .map(|(_, node)| node);
    assert!(octopus_task.is_some());
    assert_eq!(octopus_task.unwrap().get_type(), "cu_octopus::octopus0");

    // Verify camera tasks with correct ports
    for id in 0..3 {
        let camera_id = format!("camera{id}");
        let camera_task = all_nodes
            .iter()
            .find(|(_, node)| node.get_id() == camera_id)
            .map(|(_, node)| node);

        assert!(camera_task.is_some());
        let camera = camera_task.unwrap();
        assert_eq!(camera.get_type(), "cu_camera::Camera");

        let expected_port = format!("/dev/video{id}");
        assert_eq!(camera.get_param::<String>("port").unwrap(), expected_port);
    }

    // Verify detector tasks with correct thresholds
    let expected_thresholds = [0.5, 0.1, 0.7];
    #[allow(clippy::needless_range_loop)]
    for id in 0..3 {
        let detect_id = format!("detect{id}");
        let detect_task = all_nodes
            .iter()
            .find(|(_, node)| node.get_id() == detect_id)
            .map(|(_, node)| node);

        assert!(detect_task.is_some());
        let detector = detect_task.unwrap();
        assert_eq!(detector.get_type(), "cu_detector::Detector");

        let threshold = detector.get_param::<f64>("threshold").unwrap();
        assert_eq!(threshold, expected_thresholds[id]);
    }

    // Get the graph and verify connections
    let graph = config.graphs.get_graph(None).unwrap();

    // Find the node IDs for verification
    let mut camera_node_ids = Vec::new();
    let mut detector_node_ids = Vec::new();
    let mut octopus_node_id = None;

    for idx in graph.node_indices() {
        let node = graph.0.node_weight(idx).unwrap();
        let id = node.get_id();

        if id == "octopus" {
            octopus_node_id = Some(idx);
        } else if id.starts_with("camera") {
            camera_node_ids.push((id.clone(), idx));
        } else if id.starts_with("detect") {
            detector_node_ids.push((id.clone(), idx));
        }
    }

    // Verify camera-to-detector connections (internal to sensor-detect)
    for id in 0..3 {
        let camera_id = format!("camera{id}");
        let detect_id = format!("detect{id}");

        let camera_idx = camera_node_ids
            .iter()
            .find(|(id, _)| *id == camera_id)
            .unwrap()
            .1;
        let detector_idx = detector_node_ids
            .iter()
            .find(|(id, _)| *id == detect_id)
            .unwrap()
            .1;

        // Check if there's an edge from camera to detector
        let has_connection = graph.0.edges_directed(camera_idx, Outgoing).any(|edge| {
            let target = edge.target();
            let cnx = edge.weight();
            target == detector_idx && cnx.msg == "cu_camera::CameraPayload"
        });

        assert!(
            has_connection,
            "Connection from {camera_id} to {detect_id} not found"
        );
    }

    // Verify detector-to-octopus connections (defined in main.ron)
    let octopus_idx = octopus_node_id.unwrap();

    for id in 0..3 {
        let detect_id = format!("detect{id}");
        let detector_idx = detector_node_ids
            .iter()
            .find(|(id, _)| *id == detect_id)
            .unwrap()
            .1;

        // Check if there's an edge from detector to octopus
        let has_connection = graph.0.edges_directed(detector_idx, Outgoing).any(|edge| {
            let target = edge.target();
            let cnx = edge.weight();
            target == octopus_idx && cnx.msg == "cu_detect::DetectionPayload"
        });

        assert!(
            has_connection,
            "Connection from {detect_id} to octopus not found"
        );
    }

    // Verify monitor
    assert!(config.monitor.is_some());
    assert_eq!(
        config.monitor.as_ref().unwrap().get_type(),
        "cu_consolemon::CuConsoleMon"
    );
}
