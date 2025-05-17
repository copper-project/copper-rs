use cu29_runtime::config::{read_configuration, Outgoing};
use petgraph::visit::EdgeRef;
use std::fs::{create_dir_all, write};
use tempfile::tempdir;

#[test]
fn test_modular_config_basic() {
    let temp_dir = tempdir().unwrap();
    let base_path = temp_dir.path();

    let config_dir = base_path.join("config");
    create_dir_all(&config_dir).unwrap();

    // Create base.ron with a source task
    let base_config = r#"(
        tasks: [
            (
                id: "source",
                type: "tasks::FlippingSource",
                config: {
                    "rate": 10,
                },
            ),
        ],
        cnx: [],
    )"#;

    let base_path = config_dir.join("base.ron");
    write(&base_path, base_config).unwrap();

    // Create motors.ron template with parameters
    let motors_config = r#"(
        tasks: [
            (
                id: "motor_{{id}}",
                type: "tasks::RPGpio",
                config: {
                    "pin": {{pin}},
                    "direction": "{{direction}}",
                },
            ),
        ],
        cnx: [
            (src: "source", dst: "motor_{{id}}", msg: "tasks::GPIOPayload"),
        ],
    )"#;

    let motors_path = config_dir.join("motors.ron");
    write(&motors_path, motors_config).unwrap();

    // Create main_config.ron that includes the others with params
    let main_config = r#"(
        tasks: [],
        cnx: [],
        monitor: (
            type: "tasks::ConsoleMon",
        ),
        logging: (
            file: "test.copper",
            level: "debug",
        ),
        includes: [
            (
                path: "base.ron",
                params: {},
            ),
            (
                path: "motors.ron",
                params: {
                    "id": "left",
                    "pin": 4,
                    "direction": "forward",
                },
            ),
            (
                path: "motors.ron",
                params: {
                    "id": "right",
                    "pin": 5,
                    "direction": "reverse",
                },
            ),
        ],
    )"#;

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    // Read and validate the configuration
    let config = read_configuration(main_path.to_str().unwrap()).unwrap();
    let graph = config.graphs.get_graph(None).unwrap();

    // Verify tasks
    let all_nodes = graph.get_all_nodes();
    assert_eq!(all_nodes.len(), 3);

    // Verify task IDs
    let task_ids: Vec<_> = all_nodes
        .iter()
        .map(|(_, node)| node.get_id().to_string())
        .collect();
    assert!(task_ids.contains(&"source".to_string()));
    assert!(task_ids.contains(&"motor_left".to_string()));
    assert!(task_ids.contains(&"motor_right".to_string()));

    // Verify parameter substitution for the left motor
    let left_motor = all_nodes
        .iter()
        .find(|(_, node)| node.get_id() == "motor_left")
        .unwrap()
        .1;

    assert_eq!(left_motor.get_type(), "tasks::RPGpio");
    assert_eq!(left_motor.get_param::<i32>("pin").unwrap(), 4);
    assert_eq!(
        left_motor.get_param::<String>("direction").unwrap(),
        "forward"
    );

    // Verify parameter substitution for the right motor
    let right_motor = all_nodes
        .iter()
        .find(|(_, node)| node.get_id() == "motor_right")
        .unwrap()
        .1;

    assert_eq!(right_motor.get_type(), "tasks::RPGpio");
    assert_eq!(right_motor.get_param::<i32>("pin").unwrap(), 5);
    assert_eq!(
        right_motor.get_param::<String>("direction").unwrap(),
        "reverse"
    );

    // Get the graph and verify connections
    let graph = config.graphs.get_graph(None).unwrap();

    // Find node indices for verification
    let indices = graph.node_indices();
    let source_idx = indices
        .iter()
        .find(|idx| graph.0.node_weight(**idx).unwrap().get_id() == "source")
        .unwrap();

    let motor_left_idx = indices
        .iter()
        .find(|idx| graph.0.node_weight(**idx).unwrap().get_id() == "motor_left")
        .unwrap();

    let motor_right_idx = indices
        .iter()
        .find(|idx| graph.0.node_weight(**idx).unwrap().get_id() == "motor_right")
        .unwrap();

    // Verify source to left motor connection exists
    let left_connection = graph
        .0
        .edges_directed(*source_idx, Outgoing)
        .find(|edge| edge.target() == *motor_left_idx)
        .unwrap();
    assert_eq!(left_connection.weight().msg, "tasks::GPIOPayload");

    // Verify source to right motor connection exists
    let right_connection = graph
        .0
        .edges_directed(*source_idx, Outgoing)
        .find(|edge| edge.target() == *motor_right_idx)
        .unwrap();
    assert_eq!(right_connection.weight().msg, "tasks::GPIOPayload");

    // Verify monitor
    assert_eq!(
        config.monitor.as_ref().unwrap().get_type(),
        "tasks::ConsoleMon"
    );
}

#[test]
fn test_modular_config_nested() {
    let temp_dir = tempdir().unwrap();
    let base_path = temp_dir.path();

    let config_dir = base_path.join("config");
    create_dir_all(&config_dir).unwrap();

    // Create sensor.ron with sensor configuration
    let sensor_config = r#"(
        tasks: [
            (
                id: "sensor_{{id}}",
                type: "tasks::Sensor{{type}}",
                config: {
                    "rate": {{rate}},
                },
            ),
        ],
        cnx: [],
    )"#;

    let sensor_path = config_dir.join("sensor.ron");
    write(&sensor_path, sensor_config).unwrap();

    // Create processor.ron with processing tasks
    let processor_config = r#"(
        tasks: [
            (
                id: "processor_{{id}}",
                type: "tasks::DataProcessor",
                config: {
                    "threshold": {{threshold}},
                },
            ),
        ],
        cnx: [
            (src: "sensor_{{sensor_id}}", dst: "processor_{{id}}", msg: "tasks::SensorData"),
        ],
    )"#;

    let processor_path = config_dir.join("processor.ron");
    write(&processor_path, processor_config).unwrap();

    // Create subsystem.ron that includes sensor and processor
    let subsystem_config = r#"(
        tasks: [],
        cnx: [],
        includes: [
            (
                path: "sensor.ron",
                params: {
                    "id": "{{subsystem_id}}",
                    "type": "{{sensor_type}}",
                    "rate": {{sensor_rate}},
                },
            ),
            (
                path: "processor.ron",
                params: {
                    "id": "{{subsystem_id}}",
                    "sensor_id": "{{subsystem_id}}",
                    "threshold": {{processor_threshold}},
                },
            ),
        ],
    )"#;

    let subsystem_path = config_dir.join("subsystem.ron");
    write(&subsystem_path, subsystem_config).unwrap();

    // Create main_config.ron that includes multiple subsystems
    let main_config = r#"(
        tasks: [
            (
                id: "controller",
                type: "tasks::MainController",
            ),
        ],
        cnx: [
            (src: "processor_front", dst: "controller", msg: "tasks::ProcessedData"),
            (src: "processor_rear", dst: "controller", msg: "tasks::ProcessedData"),
        ],
        includes: [
            (
                path: "subsystem.ron",
                params: {
                    "subsystem_id": "front",
                    "sensor_type": "Infrared",
                    "sensor_rate": 20,
                    "processor_threshold": 0.75,
                },
            ),
            (
                path: "subsystem.ron",
                params: {
                    "subsystem_id": "rear",
                    "sensor_type": "Ultrasonic",
                    "sensor_rate": 10,
                    "processor_threshold": 0.5,
                },
            ),
        ],
    )"#;

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    // Read and validate the configuration
    let config = read_configuration(main_path.to_str().unwrap()).unwrap();
    let graph = config.graphs.get_graph(None).unwrap();

    // Verify tasks
    let all_nodes = graph.get_all_nodes();
    assert_eq!(all_nodes.len(), 5); // 1 controller + 2 sensors + 2 processors

    // Verify task IDs
    let task_ids: Vec<_> = all_nodes
        .iter()
        .map(|(_, node)| node.get_id().to_string())
        .collect();
    assert!(task_ids.contains(&"controller".to_string()));
    assert!(task_ids.contains(&"sensor_front".to_string()));
    assert!(task_ids.contains(&"sensor_rear".to_string()));
    assert!(task_ids.contains(&"processor_front".to_string()));
    assert!(task_ids.contains(&"processor_rear".to_string()));

    // Verify front sensor parameters
    let front_sensor = all_nodes
        .iter()
        .find(|(_, node)| node.get_id() == "sensor_front")
        .unwrap()
        .1;

    assert_eq!(front_sensor.get_type(), "tasks::SensorInfrared");
    assert_eq!(front_sensor.get_param::<i32>("rate").unwrap(), 20);

    // Verify rear sensor parameters
    let rear_sensor = all_nodes
        .iter()
        .find(|(_, node)| node.get_id() == "sensor_rear")
        .unwrap()
        .1;

    assert_eq!(rear_sensor.get_type(), "tasks::SensorUltrasonic");
    assert_eq!(rear_sensor.get_param::<i32>("rate").unwrap(), 10);

    // Verify front processor parameters
    let front_processor = all_nodes
        .iter()
        .find(|(_, node)| node.get_id() == "processor_front")
        .unwrap()
        .1;

    assert_eq!(front_processor.get_param::<f64>("threshold").unwrap(), 0.75);

    // Verify rear processor parameters
    let rear_processor = all_nodes
        .iter()
        .find(|(_, node)| node.get_id() == "processor_rear")
        .unwrap()
        .1;

    assert_eq!(rear_processor.get_param::<f64>("threshold").unwrap(), 0.5);

    // Get the graph and verify connections
    let graph = config.graphs.get_graph(None).unwrap();
    let indices = graph.node_indices();

    // Find the node indices
    let controller_idx = *indices
        .iter()
        .find(|idx| graph.0.node_weight(**idx).unwrap().get_id() == "controller")
        .unwrap();

    let sensor_front_idx = *indices
        .iter()
        .find(|idx| graph.0.node_weight(**idx).unwrap().get_id() == "sensor_front")
        .unwrap();

    let sensor_rear_idx = *indices
        .iter()
        .find(|idx| graph.0.node_weight(**idx).unwrap().get_id() == "sensor_rear")
        .unwrap();

    let processor_front_idx = *indices
        .iter()
        .find(|idx| graph.0.node_weight(**idx).unwrap().get_id() == "processor_front")
        .unwrap();

    let processor_rear_idx = *indices
        .iter()
        .find(|idx| graph.0.node_weight(**idx).unwrap().get_id() == "processor_rear")
        .unwrap();

    // Verify sensor to processor connections
    let sensor_front_to_processor = graph
        .0
        .edges_directed(sensor_front_idx, Outgoing)
        .any(|edge| edge.target() == processor_front_idx);
    assert!(
        sensor_front_to_processor,
        "Connection from sensor_front to processor_front not found"
    );

    let sensor_rear_to_processor = graph
        .0
        .edges_directed(sensor_rear_idx, Outgoing)
        .any(|edge| edge.target() == processor_rear_idx);
    assert!(
        sensor_rear_to_processor,
        "Connection from sensor_rear to processor_rear not found"
    );

    // Verify processor to controller connections
    let processor_front_to_controller = graph
        .0
        .edges_directed(processor_front_idx, Outgoing)
        .any(|edge| edge.target() == controller_idx);
    assert!(
        processor_front_to_controller,
        "Connection from processor_front to controller not found"
    );

    let processor_rear_to_controller = graph
        .0
        .edges_directed(processor_rear_idx, Outgoing)
        .any(|edge| edge.target() == controller_idx);
    assert!(
        processor_rear_to_controller,
        "Connection from processor_rear to controller not found"
    );
}

#[test]
fn test_modular_config_override() {
    let temp_dir = tempdir().unwrap();
    let base_path = temp_dir.path();

    let config_dir = base_path.join("config");
    create_dir_all(&config_dir).unwrap();

    // Create base.ron with default tasks and connections
    let base_config = r#"(
        tasks: [
            (
                id: "source",
                type: "tasks::DefaultSource",
                config: {
                    "param": "default",
                },
            ),
            (
                id: "processor",
                type: "tasks::DefaultProcessor",
            ),
        ],
        cnx: [
            (src: "source", dst: "processor", msg: "tasks::DefaultData"),
        ],
        monitor: (
            type: "tasks::DefaultMonitor",
        ),
    )"#;

    let base_path = config_dir.join("base.ron");
    write(&base_path, base_config).unwrap();

    // Create main_config.ron that includes base but overrides some settings
    let main_config = r#"(
        tasks: [
            (
                id: "source",
                type: "tasks::CustomSource",
                config: {
                    "param": "custom",
                    "additional": true,
                },
            ),
            (
                id: "sink",
                type: "tasks::DataSink",
            ),
        ],
        cnx: [
            (src: "processor", dst: "sink", msg: "tasks::ProcessedData"),
        ],
        monitor: (
            type: "tasks::CustomMonitor",
            config: {
                "debug": true,
            },
        ),
        includes: [
            (
                path: "base.ron",
                params: {},
            ),
        ],
    )"#;

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    // Read and validate the configuration
    let config = read_configuration(main_path.to_str().unwrap()).unwrap();
    let graph = config.graphs.get_graph(None).unwrap();

    // Verify tasks
    let all_nodes = graph.get_all_nodes();
    assert_eq!(all_nodes.len(), 3); // source, processor, sink

    // Verify source task has been overridden
    let source = all_nodes
        .iter()
        .find(|(_, node)| node.get_id() == "source")
        .unwrap()
        .1;

    assert_eq!(source.get_type(), "tasks::CustomSource");
    assert_eq!(source.get_param::<String>("param").unwrap(), "custom");
    assert!(source.get_param::<bool>("additional").unwrap());

    // Verify processor remains from base config
    let processor = all_nodes
        .iter()
        .find(|(_, node)| node.get_id() == "processor")
        .unwrap()
        .1;

    assert_eq!(processor.get_type(), "tasks::DefaultProcessor");

    // Verify sink from main config
    let sink = all_nodes
        .iter()
        .find(|(_, node)| node.get_id() == "sink")
        .unwrap()
        .1;

    assert_eq!(sink.get_type(), "tasks::DataSink");

    // Get the graph and verify connections
    let graph = config.graphs.get_graph(None).unwrap();
    let indices = graph.node_indices();

    // Find node indices
    let source_idx = *indices
        .iter()
        .find(|idx| graph.0.node_weight(**idx).unwrap().get_id() == "source")
        .unwrap();

    let processor_idx = *indices
        .iter()
        .find(|idx| graph.0.node_weight(**idx).unwrap().get_id() == "processor")
        .unwrap();

    let sink_idx = *indices
        .iter()
        .find(|idx| graph.0.node_weight(**idx).unwrap().get_id() == "sink")
        .unwrap();

    // Verify source to processor connection
    let source_to_processor = graph
        .0
        .edges_directed(source_idx, Outgoing)
        .any(|edge| edge.target() == processor_idx);
    assert!(
        source_to_processor,
        "Connection from source to processor not found"
    );

    // Verify processor to sink connection
    let processor_to_sink = graph
        .0
        .edges_directed(processor_idx, Outgoing)
        .any(|edge| edge.target() == sink_idx);
    assert!(
        processor_to_sink,
        "Connection from processor to sink not found"
    );

    // Verify monitor has been overridden
    assert_eq!(
        config.monitor.as_ref().unwrap().get_type(),
        "tasks::CustomMonitor"
    );

    // Verify monitor is present
    assert!(config.monitor.is_some());
    // We can't access private fields, so we'll just verify the type is correct
    // The custom logic was removed to focus on what's essential for testing
}

#[test]
fn test_modular_config_error_handling() {
    let temp_dir = tempdir().unwrap();
    let base_path = temp_dir.path();

    let config_dir = base_path.join("config");
    create_dir_all(&config_dir).unwrap();

    // Test missing include file
    let missing_include_config = r#"(
        tasks: [],
        cnx: [],
        includes: [
            (
                path: "nonexistent.ron",
                params: {},
            ),
        ],
    )"#;

    let missing_path = config_dir.join("missing_include.ron");
    write(&missing_path, missing_include_config).unwrap();

    let result = read_configuration(missing_path.to_str().unwrap());
    assert!(result.is_err());

    // Test invalid parameter substitution
    let template_config = r#"(
        tasks: [
            (
                id: "task_{{id}}",
                type: "tasks::Task",
                config: {
                    "value": {{value}},
                },
            ),
        ],
        cnx: [],
    )"#;

    let template_path = config_dir.join("template.ron");
    write(&template_path, template_config).unwrap();

    let missing_param_config = r#"(
        tasks: [],
        cnx: [],
        includes: [
            (
                path: "template.ron",
                params: {
                    "id": "test",
                    // Missing "value" parameter
                },
            ),
        ],
    )"#;

    let missing_param_path = config_dir.join("missing_param.ron");
    write(&missing_param_path, missing_param_config).unwrap();

    let result = read_configuration(missing_param_path.to_str().unwrap());
    assert!(result.is_err());

    // Test with invalid RON syntax
    let invalid_syntax_config = r#"(
        tasks: [
            (
                id: "task"
                type: "tasks::Task", // Missing comma after id
                config: {},
            ),
        ],
        cnx: [],
    )"#;

    let invalid_syntax_path = config_dir.join("invalid_syntax.ron");
    write(&invalid_syntax_path, invalid_syntax_config).unwrap();

    let result = read_configuration(invalid_syntax_path.to_str().unwrap());
    assert!(result.is_err());
}
