use super::*;
#[cfg(not(feature = "std"))]
use alloc::vec;
use serde::Deserialize;
#[cfg(feature = "std")]
use std::path::{Path, PathBuf};

#[test]
fn test_plain_serialize() {
    let mut config = CuConfig::default();
    let graph = config.get_graph_mut(None).unwrap();
    let n1 = graph
        .add_node(Node::new("test1", "package::Plugin1"))
        .unwrap();
    let n2 = graph
        .add_node(Node::new("test2", "package::Plugin2"))
        .unwrap();
    graph.connect(n1, n2, "msgpkg::MsgType").unwrap();
    let serialized = config.serialize_ron().unwrap();
    let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
    let graph = config.graphs.get_graph(None).unwrap();
    let deserialized_graph = deserialized.graphs.get_graph(None).unwrap();
    assert_eq!(graph.node_count(), deserialized_graph.node_count());
    assert_eq!(graph.edge_count(), deserialized_graph.edge_count());
}

#[test]
fn test_planner_config_defaults_and_round_trips() {
    // The default has no planner section and none is serialized.
    let mut config = CuConfig::default();
    config
        .get_graph_mut(None)
        .unwrap()
        .add_node(Node::new("a", "demo::A"))
        .unwrap();
    assert!(!config.serialize_ron().unwrap().contains("planner"));
    assert!(config.planner_config().is_none());

    // A planner selection and its static task order round-trip.
    let txt = r#"( tasks: [], cnx: [],
            runtime: ( planner: ( kind: TaskOrder, config: { "order": ["a", "b"] } ) ) )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let planner = config.planner_config().unwrap();
    assert_eq!(planner.kind(), PlannerKind::TaskOrder);
    let order: Vec<String> = planner
        .get_config()
        .unwrap()
        .get_value("order")
        .unwrap()
        .unwrap();
    assert_eq!(order, ["a", "b"]);
    let reparsed = CuConfig::deserialize_ron(&config.serialize_ron().unwrap()).unwrap();
    assert_eq!(reparsed.planner_kind(), PlannerKind::TaskOrder);
}

#[test]
fn test_pipeline_capacity_defaults_and_validation() {
    let parse = |planner: &str| {
        read_configuration_str(
            format!(
                r#"(
                        tasks: [(id: "src", type: "demo::Src", kind: source)],
                        cnx: [(src: "src", dst: "__nc__", msg: "u8")],
                        logging: (copperlist_count: 4),
                        runtime: (planner: {planner}),
                    )"#
            ),
            None,
        )
    };

    let defaulted = parse("(kind: Pipeline)").unwrap();
    assert_eq!(defaulted.planner_kind(), PlannerKind::Pipeline);
    assert_eq!(
        defaulted
            .planner_config()
            .unwrap()
            .max_in_flight(4)
            .unwrap(),
        4
    );

    let configured = parse(r#"(kind: Pipeline, config: { "max_in_flight": 2 })"#).unwrap();
    assert_eq!(
        configured
            .planner_config()
            .unwrap()
            .max_in_flight(4)
            .unwrap(),
        2
    );

    let zero = parse(r#"(kind: Pipeline, config: { "max_in_flight": 0 })"#)
        .unwrap_err()
        .to_string();
    assert!(zero.contains("cannot be zero"), "{zero}");

    let excess = parse(r#"(kind: Pipeline, config: { "max_in_flight": 5 })"#)
        .unwrap_err()
        .to_string();
    assert!(
        excess.contains("exceeds logging.copperlist_count"),
        "{excess}"
    );
}

#[test]
fn test_serialize_with_params() {
    let mut config = CuConfig::default();
    let graph = config.get_graph_mut(None).unwrap();
    let mut camera = Node::new("copper-camera", "camerapkg::Camera");
    camera.set_param::<Value>("resolution-height", 1080.into());
    graph.add_node(camera).unwrap();
    let serialized = config.serialize_ron().unwrap();
    let config = CuConfig::deserialize_ron(&serialized).unwrap();
    let deserialized = config.get_graph(None).unwrap();
    let resolution = deserialized
        .get_node(0)
        .unwrap()
        .get_param::<i32>("resolution-height")
        .expect("resolution-height lookup failed");
    assert_eq!(resolution, Some(1080));
}

#[derive(Debug, Deserialize, PartialEq)]
struct InnerSettings {
    threshold: u32,
    flags: Option<bool>,
}

#[derive(Debug, Deserialize, PartialEq)]
struct SettingsConfig {
    gain: f32,
    matrix: [[f32; 3]; 3],
    inner: InnerSettings,
    tags: Vec<String>,
}

#[test]
fn test_component_config_get_value_structured() {
    let txt = r#"
            (
                tasks: [
                    (
                        id: "task",
                        type: "pkg::Task",
                        config: {
                            "settings": {
                                "gain": 1.5,
                                "matrix": [
                                    [1.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0],
                                    [0.0, 0.0, 1.0],
                                ],
                                "inner": { "threshold": 42, "flags": Some(true) },
                                "tags": ["alpha", "beta"],
                            },
                        },
                    ),
                ],
                cnx: [],
            )
        "#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.graphs.get_graph(None).unwrap();
    let node = graph.get_node(0).unwrap();
    let component = node.get_instance_config().expect("missing config");
    let settings = component
        .get_value::<SettingsConfig>("settings")
        .expect("settings lookup failed")
        .expect("missing settings");
    let expected = SettingsConfig {
        gain: 1.5,
        matrix: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        inner: InnerSettings {
            threshold: 42,
            flags: Some(true),
        },
        tags: vec!["alpha".to_string(), "beta".to_string()],
    };
    assert_eq!(settings, expected);
}

#[test]
fn test_component_config_get_value_scalar_compatibility() {
    let txt = r#"
            (
                tasks: [
                    (id: "task", type: "pkg::Task", config: { "scalar": 7 }),
                ],
                cnx: [],
            )
        "#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.graphs.get_graph(None).unwrap();
    let node = graph.get_node(0).unwrap();
    let component = node.get_instance_config().expect("missing config");
    let scalar = component
        .get::<u32>("scalar")
        .expect("scalar lookup failed");
    assert_eq!(scalar, Some(7));
}

#[test]
fn test_component_config_get_value_mixed_usage() {
    let txt = r#"
            (
                tasks: [
                    (
                        id: "task",
                        type: "pkg::Task",
                        config: {
                            "scalar": 12,
                            "settings": {
                                "gain": 2.5,
                                "matrix": [
                                    [1.0, 2.0, 3.0],
                                    [4.0, 5.0, 6.0],
                                    [7.0, 8.0, 9.0],
                                ],
                                "inner": { "threshold": 7, "flags": None },
                                "tags": ["gamma"],
                            },
                        },
                    ),
                ],
                cnx: [],
            )
        "#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.graphs.get_graph(None).unwrap();
    let node = graph.get_node(0).unwrap();
    let component = node.get_instance_config().expect("missing config");
    let scalar = component
        .get::<u32>("scalar")
        .expect("scalar lookup failed");
    let settings = component
        .get_value::<SettingsConfig>("settings")
        .expect("settings lookup failed");
    assert_eq!(scalar, Some(12));
    assert!(settings.is_some());
}

#[test]
fn test_component_config_get_value_error_includes_key() {
    let txt = r#"
            (
                tasks: [
                    (
                        id: "task",
                        type: "pkg::Task",
                        config: { "settings": { "gain": 1.0 } },
                    ),
                ],
                cnx: [],
            )
        "#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.graphs.get_graph(None).unwrap();
    let node = graph.get_node(0).unwrap();
    let component = node.get_instance_config().expect("missing config");
    let err = component
        .get_value::<u32>("settings")
        .expect_err("expected type mismatch");
    assert!(err.to_string().contains("settings"));
}

#[test]
fn test_deserialization_error() {
    // Task needs to be an array, but provided tuple wrongfully
    let txt = r#"( tasks: (), cnx: [], monitors: [(type: "ExampleMonitor", )] ) "#;
    let err = CuConfig::deserialize_ron(txt).expect_err("expected deserialization error");
    assert!(
        err.to_string()
            .contains("Syntax Error in config: Expected opening `[` at position 1:9-1:10")
    );
}

#[test]
fn test_compile_time_constant_defaults_and_normalization() {
    let config = read_configuration_str(
        r#"(
                constants: [
                    (id: "COUNT", storage: usize, value: 12),
                    (id: "COUNT", module: "diagnostics", storage: usize, value: 24),
                    (id: "LENGTH_DEFAULT", quantity: length, value: [0.18, 0.0, 0.31]),
                    (id: "LENGTH_EXPLICIT", quantity: length, unit: meter, storage: f32,
                        value: [0.18, 0.0, 0.31]),
                    (id: "LENGTH_MM", quantity: length, unit: millimeter,
                        value: [180.0, 0.0, 310.0]),
                    (id: "ANGLE_DEG", quantity: angle, unit: degree, value: 180.0),
                    (id: "MASS_DEFAULT", quantity: mass, value: 1.0),
                    (id: "TEMPERATURE_C", quantity: thermodynamic_temperature,
                        unit: degree_celsius, storage: f64, value: 20.0),
                    (id: "CONSTRUCTED", module: "geometry", type: "crate::ConstPair",
                        expression: "crate::ConstPair::new(crate::constants::COUNT)"),
                    (id: "CONSTRUCTED_COPY", module: "geometry", type: "crate::ConstPair",
                        expression: "crate::ConstPair::new(crate::constants::COUNT)"),
                    (id: "CONSTRUCTED_REWRITTEN", module: "geometry", type: "crate::ConstPair",
                        expression: "crate::ConstPair::new( crate::constants::COUNT )"),
                ],
                tasks: [],
                cnx: [],
            )"#
        .to_string(),
        None,
    )
    .unwrap();

    assert_eq!(config.constants[0].module_path(), "constants");
    assert_eq!(config.constants[0].qualified_id(), "constants::COUNT");
    assert_eq!(config.constants[0].storage(), ConstantStorage::Usize);
    assert_eq!(config.constants[1].module_path(), "diagnostics");
    assert_eq!(config.constants[1].qualified_id(), "diagnostics::COUNT");
    let (_, default_length) = config.constants[2].normalized_f32().unwrap();
    let (_, explicit_length) = config.constants[3].normalized_f32().unwrap();
    let (_, millimeters) = config.constants[4].normalized_f32().unwrap();
    assert_eq!(default_length[0].to_bits(), explicit_length[0].to_bits());
    assert_eq!(default_length[2].to_bits(), explicit_length[2].to_bits());
    assert_eq!(default_length, millimeters);
    assert_eq!(
        config.constants[2].semantic_fingerprint().unwrap(),
        config.constants[3].semantic_fingerprint().unwrap()
    );
    assert_eq!(
        config.constants[2].semantic_fingerprint().unwrap(),
        config.constants[4].semantic_fingerprint().unwrap()
    );

    let (_, angle) = config.constants[5].normalized_f32().unwrap();
    assert_eq!(angle[0].to_bits(), core::f32::consts::PI.to_bits());

    let mass = &config.constants[6];
    assert_eq!(mass.resolved_unit().unwrap().unwrap().name(), "kilogram");
    assert_eq!(mass.normalized_f32().unwrap().1, vec![1.0]);

    let temperature = config.constants[7].normalized_f64().unwrap().1[0];
    assert!((temperature - 293.15).abs() < f64::EPSILON * 4.0);

    assert_eq!(
        config.constants[8].expression_definition(),
        Some((
            "crate::ConstPair",
            "crate::ConstPair::new(crate::constants::COUNT)"
        ))
    );
    assert_eq!(
        config.constants[8].semantic_fingerprint().unwrap(),
        config.constants[9].semantic_fingerprint().unwrap()
    );
    assert_ne!(
        config.constants[8].semantic_fingerprint().unwrap(),
        config.constants[10].semantic_fingerprint().unwrap()
    );

    let serialized = config.serialize_ron().unwrap();
    let reparsed = CuConfig::deserialize_ron(&serialized).unwrap();
    assert_eq!(
        reparsed.constants[8].expression_definition(),
        config.constants[8].expression_definition()
    );
    assert_eq!(
        reparsed.constants[8].semantic_fingerprint().unwrap(),
        config.constants[8].semantic_fingerprint().unwrap()
    );
}

#[test]
fn test_compile_time_constant_rejects_invalid_definition_shapes() {
    let cases = [
        (
            r#"(id: "BAD", type: "crate::Pair")"#,
            "declares 'type' without 'expression'",
        ),
        (
            r#"(id: "BAD", expression: "crate::Pair::new()")"#,
            "declares 'expression' without 'type'",
        ),
        (
            r#"(id: "BAD", value: 1, type: "u32", expression: "1")"#,
            "cannot combine numeric 'value' with 'type' or 'expression'",
        ),
        (
            r#"(id: "BAD", storage: f32, type: "u32", expression: "1")"#,
            "cannot combine 'type' and 'expression' with numeric 'storage', 'quantity', or 'unit'",
        ),
        (
            r#"(id: "BAD")"#,
            "must declare either numeric 'value' or both 'type' and 'expression'",
        ),
    ];

    for (constant, expected) in cases {
        let source = format!("(constants: [{constant}], tasks: [], cnx: [])");
        let error = read_configuration_str(source, None)
            .expect_err("invalid constant definition shape must fail");
        assert!(
            error.to_string().contains(expected),
            "unexpected error: {error}"
        );
    }
}

#[test]
fn test_compile_time_constant_rejects_duplicate_qualified_id() {
    let error = read_configuration_str(
        r#"(
                constants: [
                    (id: "COUNT", module: "diagnostics", value: 1),
                    (id: "COUNT", module: "diagnostics", value: 2),
                ],
                tasks: [],
                cnx: [],
            )"#
        .to_string(),
        None,
    )
    .expect_err("duplicate qualified constant id must fail");
    assert!(
        error
            .to_string()
            .contains("Duplicate constant 'diagnostics::COUNT'")
    );
}

#[test]
fn test_compile_time_constant_rejects_incompatible_unit() {
    let error = read_configuration_str(
        r#"(
                constants: [(id: "BAD", quantity: length, unit: degree, value: 1.0)],
                tasks: [],
                cnx: [],
            )"#
        .to_string(),
        None,
    )
    .expect_err("length in degrees must fail");
    assert!(
        error
            .to_string()
            .contains("unit 'degree' is not compatible with quantity 'length'")
    );
}

#[test]
fn test_missions() {
    let txt = r#"( missions: [ (id: "data_collection"), (id: "autonomous")])"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.graphs.get_graph(Some("data_collection")).unwrap();
    assert!(graph.node_count() == 0);
    let graph = config.graphs.get_graph(Some("autonomous")).unwrap();
    assert!(graph.node_count() == 0);
}

#[test]
fn test_monitor_plural_syntax() {
    let txt = r#"( tasks: [], cnx: [], monitors: [(type: "ExampleMonitor", )] ) "#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    assert_eq!(config.get_monitor_config().unwrap().type_, "ExampleMonitor");

    let txt =
        r#"( tasks: [], cnx: [], monitors: [(type: "ExampleMonitor", config: { "toto": 4, } )] ) "#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    assert_eq!(
        config
            .get_monitor_config()
            .unwrap()
            .config
            .as_ref()
            .unwrap()
            .0["toto"]
            .0,
        4u8.into()
    );
}

#[test]
fn test_monitor_singular_syntax() {
    let txt =
        r#"( tasks: [], cnx: [], monitor: (type: "ExampleMonitor", config: { "toto": 4, } ) ) "#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    assert_eq!(config.get_monitor_configs().len(), 1);
    assert_eq!(config.get_monitor_config().unwrap().type_, "ExampleMonitor");
    assert_eq!(
        config
            .get_monitor_config()
            .unwrap()
            .config
            .as_ref()
            .unwrap()
            .0["toto"]
            .0,
        4u8.into()
    );
}

#[test]
#[cfg(feature = "std")]
fn test_render_topology_multi_input_ports() {
    let mut config = CuConfig::default();
    let graph = config.get_graph_mut(None).unwrap();
    let src1 = graph.add_node(Node::new("src1", "tasks::Source1")).unwrap();
    let src2 = graph.add_node(Node::new("src2", "tasks::Source2")).unwrap();
    let dst = graph.add_node(Node::new("dst", "tasks::Dst")).unwrap();
    graph.connect(src1, dst, "msg::A").unwrap();
    graph.connect(src2, dst, "msg::B").unwrap();

    let topology = build_render_topology(graph, &[]);
    let dst_node = topology
        .nodes
        .iter()
        .find(|node| node.id == "dst")
        .expect("missing dst node");
    assert_eq!(dst_node.inputs.len(), 2);

    let mut dst_ports: Vec<_> = topology
        .connections
        .iter()
        .filter(|cnx| cnx.dst == "dst")
        .map(|cnx| cnx.dst_port.as_deref().expect("missing dst port"))
        .collect();
    dst_ports.sort();
    assert_eq!(dst_ports, vec!["in.0", "in.1"]);
}

#[test]
fn test_logging_parameters() {
    // Test with `enable_task_logging: false`
    let txt = r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 1024, section_size_mib: 100, enable_task_logging: false ),) "#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    assert!(config.logging.is_some());
    let logging_config = config.logging.unwrap();
    assert_eq!(logging_config.slab_size_mib.unwrap(), 1024);
    assert_eq!(logging_config.section_size_mib.unwrap(), 100);
    assert!(!logging_config.enable_task_logging);

    // Test with `enable_task_logging` not provided
    let txt =
        r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 1024, section_size_mib: 100, ),) "#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    assert!(config.logging.is_some());
    let logging_config = config.logging.unwrap();
    assert_eq!(logging_config.slab_size_mib.unwrap(), 1024);
    assert_eq!(logging_config.section_size_mib.unwrap(), 100);
    assert!(logging_config.enable_task_logging);
}

#[test]
fn test_node_logging_handle_content_round_trips() {
    // RON enum variants use bare identifiers — same convention as `kind: source`.
    let txt = r#"(
            tasks: [
                (id: "cam", type: "pkg::Cam", kind: source, logging: (handle_content: touched_only)),
                (id: "noop", type: "pkg::Noop", kind: sink),
            ],
            cnx: [
                (src: "cam", dst: "noop", msg: "pkg::Frame"),
            ],
        )"#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    let cam = config.find_task_node(None, "cam").unwrap();
    assert_eq!(cam.handle_content_policy(), HandleContent::TouchedOnly);

    // A node without an explicit `logging` block falls back to `All`.
    let noop = config.find_task_node(None, "noop").unwrap();
    assert_eq!(noop.handle_content_policy(), HandleContent::All);

    // Round-trip preserves the policy.
    let reserialized = config.serialize_ron().unwrap();
    let reparsed = CuConfig::deserialize_ron(&reserialized).unwrap();
    let cam2 = reparsed.find_task_node(None, "cam").unwrap();
    assert_eq!(cam2.handle_content_policy(), HandleContent::TouchedOnly);
}

#[test]
fn test_node_logging_handle_content_all_variants_parse() {
    for (value, expected) in [
        ("all", HandleContent::All),
        ("touched_only", HandleContent::TouchedOnly),
        ("none", HandleContent::None),
    ] {
        let txt = format!(
            r#"(
                    tasks: [(id: "s", type: "pkg::T", kind: source, logging: (handle_content: {value}))],
                    cnx: [(src: "s", dst: "__nc__", msg: "pkg::M")],
                )"#
        );
        let config = CuConfig::deserialize_ron(&txt).unwrap();
        assert_eq!(
            config
                .find_task_node(None, "s")
                .unwrap()
                .handle_content_policy(),
            expected,
            "policy mismatch for `{value}`"
        );
    }
}

#[test]
fn test_bridge_parsing() {
    let txt = r#"
        (
            tasks: [
                (id: "dst", type: "tasks::Destination"),
                (id: "src", type: "tasks::Source"),
            ],
            bridges: [
                (
                    id: "radio",
                    type: "tasks::SerialBridge",
                    config: { "path": "/dev/ttyACM0", "baud": 921600 },
                    channels: [
                        Rx ( id: "status", route: "sys/status" ),
                        Tx ( id: "motor", route: "motor/cmd" ),
                    ],
                ),
            ],
            cnx: [
                (src: "radio/status", dst: "dst", msg: "mymsgs::Status"),
                (src: "src", dst: "radio/motor", msg: "mymsgs::MotorCmd"),
            ],
        )
        "#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    assert_eq!(config.bridges.len(), 1);
    let bridge = &config.bridges[0];
    assert_eq!(bridge.id, "radio");
    assert_eq!(bridge.channels.len(), 2);
    match &bridge.channels[0] {
        BridgeChannelConfigRepresentation::Rx { id, route, .. } => {
            assert_eq!(id, "status");
            assert_eq!(route.as_deref(), Some("sys/status"));
        }
        _ => panic!("expected Rx channel"),
    }
    match &bridge.channels[1] {
        BridgeChannelConfigRepresentation::Tx { id, route, .. } => {
            assert_eq!(id, "motor");
            assert_eq!(route.as_deref(), Some("motor/cmd"));
        }
        _ => panic!("expected Tx channel"),
    }
    let graph = config.graphs.get_graph(None).unwrap();
    let bridge_id = graph
        .get_node_id_by_name("radio")
        .expect("bridge node missing");
    let bridge_node = graph.get_node(bridge_id).unwrap();
    assert_eq!(bridge_node.get_flavor(), Flavor::Bridge);

    // Edges should retain channel metadata.
    let mut edges = Vec::new();
    for edge_idx in graph.0.edge_indices() {
        edges.push(graph.0[edge_idx].clone());
    }
    assert_eq!(edges.len(), 2);
    let status_edge = edges
        .iter()
        .find(|e| e.dst == "dst")
        .expect("status edge missing");
    assert_eq!(status_edge.src_channel.as_deref(), Some("status"));
    assert!(status_edge.dst_channel.is_none());
    let motor_edge = edges
        .iter()
        .find(|e| e.dst_channel.is_some())
        .expect("motor edge missing");
    assert_eq!(motor_edge.dst_channel.as_deref(), Some("motor"));
}

#[test]
fn test_bridge_roundtrip() {
    let mut config = CuConfig::default();
    let mut bridge_config = ComponentConfig::default();
    bridge_config.set("port", "/dev/ttyACM0".to_string());
    config.bridges.push(BridgeConfig {
        id: "radio".to_string(),
        type_: "tasks::SerialBridge".to_string(),
        config: Some(bridge_config),
        resources: None,
        missions: None,
        run_in_sim: None,
        channels: vec![
            BridgeChannelConfigRepresentation::Rx {
                id: "status".to_string(),
                route: Some("sys/status".to_string()),
                config: None,
            },
            BridgeChannelConfigRepresentation::Tx {
                id: "motor".to_string(),
                route: Some("motor/cmd".to_string()),
                config: None,
            },
        ],
    });

    let serialized = config.serialize_ron().unwrap();
    assert!(
        serialized.contains("bridges"),
        "bridges section missing from serialized config"
    );
    let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
    assert_eq!(deserialized.bridges.len(), 1);
    let bridge = &deserialized.bridges[0];
    assert!(bridge.is_run_in_sim());
    assert_eq!(bridge.channels.len(), 2);
    assert!(matches!(
        bridge.channels[0],
        BridgeChannelConfigRepresentation::Rx { .. }
    ));
    assert!(matches!(
        bridge.channels[1],
        BridgeChannelConfigRepresentation::Tx { .. }
    ));
}

#[test]
fn test_resource_parsing() {
    let txt = r#"
        (
            resources: [
                (
                    id: "fc",
                    provider: "copper_board_px4::Px4Bundle",
                    config: { "baud": 921600 },
                    missions: ["m1"],
                ),
                (
                    id: "misc",
                    provider: "cu29_runtime::StdClockBundle",
                ),
            ],
        )
        "#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    assert_eq!(config.resources.len(), 2);
    let fc = &config.resources[0];
    assert_eq!(fc.id, "fc");
    assert_eq!(fc.provider, "copper_board_px4::Px4Bundle");
    assert_eq!(fc.missions.as_deref(), Some(&["m1".to_string()][..]));
    let baud: u32 = fc
        .config
        .as_ref()
        .expect("missing config")
        .get::<u32>("baud")
        .expect("baud lookup failed")
        .expect("missing baud");
    assert_eq!(baud, 921_600);
    let misc = &config.resources[1];
    assert_eq!(misc.id, "misc");
    assert_eq!(misc.provider, "cu29_runtime::StdClockBundle");
    assert!(misc.config.is_none());
}

#[test]
fn test_resource_roundtrip() {
    let mut config = CuConfig::default();
    let mut bundle_cfg = ComponentConfig::default();
    bundle_cfg.set("path", "/dev/ttyACM0".to_string());
    config.resources.push(ResourceBundleConfig {
        resources: None,
        id: "fc".to_string(),
        provider: "copper_board_px4::Px4Bundle".to_string(),
        config: Some(bundle_cfg),
        missions: Some(vec!["m1".to_string()]),
    });

    let serialized = config.serialize_ron().unwrap();
    let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
    assert_eq!(deserialized.resources.len(), 1);
    let res = &deserialized.resources[0];
    assert_eq!(res.id, "fc");
    assert_eq!(res.provider, "copper_board_px4::Px4Bundle");
    assert_eq!(res.missions.as_deref(), Some(&["m1".to_string()][..]));
    let path: String = res
        .config
        .as_ref()
        .expect("missing config")
        .get::<String>("path")
        .expect("path lookup failed")
        .expect("missing path");
    assert_eq!(path, "/dev/ttyACM0");
}

#[test]
fn test_bridge_channel_config() {
    let txt = r#"
        (
            tasks: [],
            bridges: [
                (
                    id: "radio",
                    type: "tasks::SerialBridge",
                    channels: [
                        Rx ( id: "status", route: "sys/status", config: { "filter": "fast" } ),
                        Tx ( id: "imu", route: "telemetry/imu", config: { "rate": 100 } ),
                    ],
                ),
            ],
            cnx: [],
        )
        "#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    let bridge = &config.bridges[0];
    match &bridge.channels[0] {
        BridgeChannelConfigRepresentation::Rx {
            config: Some(cfg), ..
        } => {
            let val = cfg
                .get::<String>("filter")
                .expect("filter lookup failed")
                .expect("filter missing");
            assert_eq!(val, "fast");
        }
        _ => panic!("expected Rx channel with config"),
    }
    match &bridge.channels[1] {
        BridgeChannelConfigRepresentation::Tx {
            config: Some(cfg), ..
        } => {
            let rate = cfg
                .get::<i32>("rate")
                .expect("rate lookup failed")
                .expect("rate missing");
            assert_eq!(rate, 100);
        }
        _ => panic!("expected Tx channel with config"),
    }
}

#[test]
fn test_task_resources_roundtrip() {
    let txt = r#"
        (
            tasks: [
                (
                    id: "imu",
                    type: "tasks::ImuDriver",
                    resources: { "bus": "fc.spi_1", "irq": "fc.gpio_imu" },
                ),
            ],
            cnx: [],
        )
        "#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.graphs.get_graph(None).unwrap();
    let node = graph.get_node(0).expect("missing task node");
    let resources = node.get_resources().expect("missing resources map");
    assert_eq!(resources.get("bus").map(String::as_str), Some("fc.spi_1"));
    assert_eq!(
        resources.get("irq").map(String::as_str),
        Some("fc.gpio_imu")
    );

    let serialized = config.serialize_ron().unwrap();
    let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
    let graph = deserialized.graphs.get_graph(None).unwrap();
    let node = graph.get_node(0).expect("missing task node");
    let resources = node
        .get_resources()
        .expect("missing resources map after roundtrip");
    assert_eq!(resources.get("bus").map(String::as_str), Some("fc.spi_1"));
    assert_eq!(
        resources.get("irq").map(String::as_str),
        Some("fc.gpio_imu")
    );
}

#[test]
fn test_bridge_resources_preserved() {
    let mut config = CuConfig::default();
    config.resources.push(ResourceBundleConfig {
        resources: None,
        id: "fc".to_string(),
        provider: "board::Bundle".to_string(),
        config: None,
        missions: None,
    });
    let bridge_resources = HashMap::from([("serial".to_string(), "fc.serial0".to_string())]);
    config.bridges.push(BridgeConfig {
        id: "radio".to_string(),
        type_: "tasks::SerialBridge".to_string(),
        config: None,
        resources: Some(bridge_resources),
        missions: None,
        run_in_sim: None,
        channels: vec![BridgeChannelConfigRepresentation::Tx {
            id: "uplink".to_string(),
            route: None,
            config: None,
        }],
    });

    let serialized = config.serialize_ron().unwrap();
    let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
    let graph = deserialized.graphs.get_graph(None).expect("missing graph");
    let bridge_id = graph
        .get_node_id_by_name("radio")
        .expect("bridge node missing");
    let node = graph.get_node(bridge_id).expect("missing bridge node");
    let resources = node
        .get_resources()
        .expect("bridge resources were not preserved");
    assert_eq!(
        resources.get("serial").map(String::as_str),
        Some("fc.serial0")
    );
}

#[test]
fn test_demo_config_parses() {
    let txt = r#"(
    resources: [
        (
            id: "fc",
            provider: "crate::resources::RadioBundle",
        ),
    ],
    tasks: [
        (id: "thr", type: "tasks::ThrottleControl"),
        (id: "tele0", type: "tasks::TelemetrySink0"),
        (id: "tele1", type: "tasks::TelemetrySink1"),
        (id: "tele2", type: "tasks::TelemetrySink2"),
        (id: "tele3", type: "tasks::TelemetrySink3"),
    ],
    bridges: [
        (  id: "crsf",
           type: "cu_crsf::CrsfBridge<SerialResource, SerialPortError>",
           resources: { "serial": "fc.serial" },
           channels: [
                Rx ( id: "rc_rx" ),  // receiving RC Channels
                Tx ( id: "lq_tx" ),  // Sending LineQuality back
            ],
        ),
        (
            id: "bdshot",
            type: "cu_bdshot::RpBdshotBridge",
            channels: [
                Tx ( id: "esc0_tx" ),
                Tx ( id: "esc1_tx" ),
                Tx ( id: "esc2_tx" ),
                Tx ( id: "esc3_tx" ),
                Rx ( id: "esc0_rx" ),
                Rx ( id: "esc1_rx" ),
                Rx ( id: "esc2_rx" ),
                Rx ( id: "esc3_rx" ),
            ],
        ),
    ],
    cnx: [
        (src: "crsf/rc_rx", dst: "thr", msg: "cu_crsf::messages::RcChannelsPayload"),
        (src: "thr", dst: "bdshot/esc0_tx", msg: "cu_bdshot::EscCommand"),
        (src: "thr", dst: "bdshot/esc1_tx", msg: "cu_bdshot::EscCommand"),
        (src: "thr", dst: "bdshot/esc2_tx", msg: "cu_bdshot::EscCommand"),
        (src: "thr", dst: "bdshot/esc3_tx", msg: "cu_bdshot::EscCommand"),
        (src: "bdshot/esc0_rx", dst: "tele0", msg: "cu_bdshot::EscTelemetry"),
        (src: "bdshot/esc1_rx", dst: "tele1", msg: "cu_bdshot::EscTelemetry"),
        (src: "bdshot/esc2_rx", dst: "tele2", msg: "cu_bdshot::EscTelemetry"),
        (src: "bdshot/esc3_rx", dst: "tele3", msg: "cu_bdshot::EscTelemetry"),
    ],
)"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    assert_eq!(config.resources.len(), 1);
    assert_eq!(config.bridges.len(), 2);
}

#[test]
fn test_bridge_tx_cannot_be_source() {
    let txt = r#"
        (
            tasks: [
                (id: "dst", type: "tasks::Destination"),
            ],
            bridges: [
                (
                    id: "radio",
                    type: "tasks::SerialBridge",
                    channels: [
                        Tx ( id: "motor", route: "motor/cmd" ),
                    ],
                ),
            ],
            cnx: [
                (src: "radio/motor", dst: "dst", msg: "mymsgs::MotorCmd"),
            ],
        )
        "#;

    let err = CuConfig::deserialize_ron(txt).expect_err("expected bridge source error");
    assert!(
        err.to_string()
            .contains("channel 'motor' is Tx and cannot act as a source")
    );
}

#[test]
fn test_bridge_rx_cannot_be_destination() {
    let txt = r#"
        (
            tasks: [
                (id: "src", type: "tasks::Source"),
            ],
            bridges: [
                (
                    id: "radio",
                    type: "tasks::SerialBridge",
                    channels: [
                        Rx ( id: "status", route: "sys/status" ),
                    ],
                ),
            ],
            cnx: [
                (src: "src", dst: "radio/status", msg: "mymsgs::Status"),
            ],
        )
        "#;

    let err = CuConfig::deserialize_ron(txt).expect_err("expected bridge destination error");
    assert!(
        err.to_string()
            .contains("channel 'status' is Rx and cannot act as a destination")
    );
}

#[test]
fn test_validate_logging_config() {
    // Test with valid logging configuration
    let txt = r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 1024, section_size_mib: 100 ) )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    assert!(config.validate_logging_config().is_ok());

    // Test with invalid logging configuration
    let txt = r#"( tasks: [], cnx: [], logging: ( slab_size_mib: 100, section_size_mib: 1024 ) )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    assert!(config.validate_logging_config().is_err());
}

#[test]
fn log_streaming_config_parses_and_round_trips_without_scheme_fields() {
    let txt = r#"
        (
            resources: [
                (
                    id: "telemetry_udp",
                    provider: "cu29_logstream_udp::CuUdpLogStreamResources",
                    config: {
                        "bind_addr": "0.0.0.0:0",
                        "remote_addr": "192.168.10.20:7447",
                        "send_buffer_bytes": 262144,
                        "ttl": 1,
                        "dscp": 46,
                    },
                ),
            ],
            log_streaming: (
                destinations: [
                    (
                        id: "ground",
                        transport: (
                            type: "cu29_logstream_udp::CuUdpLogStreamTx",
                            resource: "telemetry_udp.tx",
                        ),
                        link: (
                            mtu_bytes: 1200,
                            bitrate_bps: 1000000,
                            memory_budget_kib: 512,
                            max_latency_ms: 250,
                            burst_packets: 8,
                        ),
                        fec: (
                            continuous: (
                                field: Gf256,
                                window_symbols: 64,
                                repair_every_source_symbols: 4,
                                repair_density: Full,
                            ),
                            objects: (
                                max_object_bytes: 4194304,
                                repair_symbols_per_block: 8,
                            ),
                        ),
                        recovery_interval: 100,
                        max_record_bytes: 65536,
                    ),
                ],
            ),
        )
        "#;

    let config = read_configuration_str(txt.to_string(), None).unwrap();
    let destination = &config.log_streaming.as_ref().unwrap().destinations[0];
    assert_eq!(destination.id, "ground");
    assert_eq!(destination.transport.resource, "telemetry_udp.tx");
    assert_eq!(destination.fec.continuous.field, LogStreamRlcField::Gf256);
    assert_eq!(
        destination.fec.continuous.repair_density,
        LogStreamRepairDensity::Full
    );

    let serialized = config.serialize_ron().unwrap();
    let reparsed = read_configuration_str(serialized, None).unwrap();
    assert_eq!(reparsed.log_streaming, config.log_streaming);

    let feedback = r#"feedback: (
            transport: (type: "app::FeedbackRx", resource: "telemetry_udp.rx"),
            report_interval_ms: 500, timeout_ms: 2000,
            adaptation: (min_repair_every_source_symbols: 1, max_repair_every_source_symbols: 16),
        ), link:"#;
    let adaptive = txt.replace("link:", feedback);
    let parsed = read_configuration_str(adaptive.clone(), None).unwrap();
    assert!(
        parsed.log_streaming.as_ref().unwrap().destinations[0]
            .feedback
            .is_some()
    );
    let roundtrip = read_configuration_str(parsed.serialize_ron().unwrap(), None).unwrap();
    assert_eq!(roundtrip.log_streaming, parsed.log_streaming);
    for invalid in [
        adaptive.replace("report_interval_ms: 500", "report_interval_ms: 0"),
        adaptive.replace("timeout_ms: 2000", "timeout_ms: 500"),
        adaptive.replace(
            "min_repair_every_source_symbols: 1",
            "min_repair_every_source_symbols: 5",
        ),
        adaptive.replace(
            "max_repair_every_source_symbols: 16",
            "max_repair_every_source_symbols: 3",
        ),
        adaptive.replace("telemetry_udp.rx", "telemetry_udp.tx"),
        adaptive.replace("telemetry_udp.rx", "missing.rx"),
    ] {
        assert!(read_configuration_str(invalid, None).is_err());
    }
}

#[test]
fn log_streaming_rejects_a_pluggable_fec_scheme() {
    let txt = r#"
        (
            resources: [(id: "network", provider: "app::Network")],
            log_streaming: (
                destinations: [(
                    id: "ground",
                    transport: (type: "app::Tx", resource: "network.tx"),
                    link: (
                        mtu_bytes: 1200,
                        bitrate_bps: 1000000,
                        memory_budget_kib: 512,
                        max_latency_ms: 250,
                        burst_packets: 8,
                    ),
                    fec: (
                        continuous: (
                            scheme: Rlc,
                            field: Gf256,
                            window_symbols: 64,
                            repair_every_source_symbols: 4,
                            repair_density: Full,
                        ),
                        objects: (
                            max_object_bytes: 4194304,
                            repair_symbols_per_block: 8,
                        ),
                    ),
                    recovery_interval: 100,
                    max_record_bytes: 65536,
                )],
            ),
        )
        "#;

    let error = read_configuration_str(txt.to_string(), None).unwrap_err();
    assert!(error.to_string().contains("scheme"), "{error}");
}

// this test makes sure the edge id is suitable to be used to sort the inputs of a task
#[test]
fn test_deserialization_edge_id_assignment() {
    // note here that the src1 task is added before src2 in the tasks array,
    // however, src1 connection is added AFTER src2 in the cnx array
    let txt = r#"(
            tasks: [(id: "src1", type: "a"), (id: "src2", type: "b"), (id: "sink", type: "c")],
            cnx: [(src: "src2", dst: "sink", msg: "msg1"), (src: "src1", dst: "sink", msg: "msg2")]
        )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.graphs.get_graph(None).unwrap();
    assert!(config.validate_logging_config().is_ok());

    // the node id depends on the order in which the tasks are added
    let src1_id = 0;
    assert_eq!(graph.get_node(src1_id).unwrap().id, "src1");
    let src2_id = 1;
    assert_eq!(graph.get_node(src2_id).unwrap().id, "src2");

    // the edge id depends on the order the connection is created
    // the src2 was added second in the tasks, but the connection was added first
    let src1_edge_id = *graph.get_src_edges(src1_id).unwrap().first().unwrap();
    assert_eq!(src1_edge_id, 1);
    let src2_edge_id = *graph.get_src_edges(src2_id).unwrap().first().unwrap();
    assert_eq!(src2_edge_id, 0);
}

#[test]
fn test_simple_missions() {
    // A simple config that selection a source depending on the mission it is in.
    let txt = r#"(
                    missions: [ (id: "m1"),
                                (id: "m2"),
                                ],
                    tasks: [(id: "src1", type: "a", missions: ["m1"]),
                            (id: "src2", type: "b", missions: ["m2"]),
                            (id: "sink", type: "c")],

                    cnx: [
                            (src: "src1", dst: "sink", msg: "u32", missions: ["m1"]),
                            (src: "src2", dst: "sink", msg: "u32", missions: ["m2"]),
                         ],
              )
              "#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    let m1_graph = config.graphs.get_graph(Some("m1")).unwrap();
    assert_eq!(m1_graph.edge_count(), 1);
    assert_eq!(m1_graph.node_count(), 2);
    let index = 0;
    let cnx = m1_graph.get_edge_weight(index).unwrap();

    assert_eq!(cnx.src, "src1");
    assert_eq!(cnx.dst, "sink");
    assert_eq!(cnx.msg, "u32");
    assert_eq!(cnx.missions, Some(vec!["m1".to_string()]));

    let m2_graph = config.graphs.get_graph(Some("m2")).unwrap();
    assert_eq!(m2_graph.edge_count(), 1);
    assert_eq!(m2_graph.node_count(), 2);
    let index = 0;
    let cnx = m2_graph.get_edge_weight(index).unwrap();
    assert_eq!(cnx.src, "src2");
    assert_eq!(cnx.dst, "sink");
    assert_eq!(cnx.msg, "u32");
    assert_eq!(cnx.missions, Some(vec!["m2".to_string()]));
}
#[test]
fn test_mission_serde() {
    // A simple config that selection a source depending on the mission it is in.
    let txt = r#"(
                    missions: [ (id: "m1"),
                                (id: "m2"),
                                ],
                    tasks: [(id: "src1", type: "a", missions: ["m1"]),
                            (id: "src2", type: "b", missions: ["m2"]),
                            (id: "sink", type: "c")],

                    cnx: [
                            (src: "src1", dst: "sink", msg: "u32", missions: ["m1"]),
                            (src: "src2", dst: "sink", msg: "u32", missions: ["m2"]),
                         ],
              )
              "#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    let serialized = config.serialize_ron().unwrap();
    let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
    let m1_graph = deserialized.graphs.get_graph(Some("m1")).unwrap();
    assert_eq!(m1_graph.edge_count(), 1);
    assert_eq!(m1_graph.node_count(), 2);
    let index = 0;
    let cnx = m1_graph.get_edge_weight(index).unwrap();
    assert_eq!(cnx.src, "src1");
    assert_eq!(cnx.dst, "sink");
    assert_eq!(cnx.msg, "u32");
    assert_eq!(cnx.missions, Some(vec!["m1".to_string()]));
}

#[test]
fn test_mission_scoped_nc_connection_survives_serialize_roundtrip() {
    let txt = r#"(
            missions: [(id: "m1"), (id: "m2")],
            tasks: [
                (id: "src_m1", type: "a", missions: ["m1"]),
                (id: "src_m2", type: "b", missions: ["m2"]),
            ],
            cnx: [
                (src: "src_m1", dst: "__nc__", msg: "msg::A", missions: ["m1"]),
                (src: "src_m2", dst: "__nc__", msg: "msg::B", missions: ["m2"]),
            ]
        )"#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    let serialized = config.serialize_ron().unwrap();
    let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();

    let m1_graph = deserialized.graphs.get_graph(Some("m1")).unwrap();
    let src_m1_id = m1_graph.get_node_id_by_name("src_m1").unwrap();
    let src_m1 = m1_graph.get_node(src_m1_id).unwrap();
    assert_eq!(src_m1.nc_outputs(), &["msg::A".to_string()]);

    let m2_graph = deserialized.graphs.get_graph(Some("m2")).unwrap();
    let src_m2_id = m2_graph.get_node_id_by_name("src_m2").unwrap();
    let src_m2 = m2_graph.get_node(src_m2_id).unwrap();
    assert_eq!(src_m2.nc_outputs(), &["msg::B".to_string()]);
}

#[test]
fn test_keyframe_interval() {
    // note here that the src1 task is added before src2 in the tasks array,
    // however, src1 connection is added AFTER src2 in the cnx array
    let txt = r#"(
            tasks: [(id: "src1", type: "a"), (id: "src2", type: "b"), (id: "sink", type: "c")],
            cnx: [(src: "src2", dst: "sink", msg: "msg1"), (src: "src1", dst: "sink", msg: "msg2")],
            logging: ( keyframe_interval: 314 )
        )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let logging_config = config.logging.unwrap();
    assert_eq!(logging_config.keyframe_interval.unwrap(), 314);
    assert!(logging_config.enable_keyframe_logging);
}

#[test]
fn test_keyframe_logging_can_be_disabled_independently() {
    let txt = r#"(
            tasks: [],
            cnx: [],
            logging: (enable_task_logging: true, enable_keyframe_logging: false),
        )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let logging = config.logging.unwrap();
    assert!(logging.enable_task_logging);
    assert!(!logging.enable_keyframe_logging);
}

#[test]
fn test_default_keyframe_interval() {
    // note here that the src1 task is added before src2 in the tasks array,
    // however, src1 connection is added AFTER src2 in the cnx array
    let txt = r#"(
            tasks: [(id: "src1", type: "a"), (id: "src2", type: "b"), (id: "sink", type: "c")],
            cnx: [(src: "src2", dst: "sink", msg: "msg1"), (src: "src1", dst: "sink", msg: "msg2")],
            logging: ( slab_size_mib: 200, section_size_mib: 1024, )
        )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let logging_config = config.logging.unwrap();
    assert_eq!(logging_config.keyframe_interval.unwrap(), 100);
}

#[test]
fn test_task_kind_roundtrip_and_alias() {
    let txt = r#"(
            tasks: [
                (id: "src", type: "a", kind: source),
                (id: "regular", type: "b", kind: regular),
                (id: "stateless", type: "c", kind: stateless_task),
                (id: "sink", type: "d", kind: sink),
            ],
            cnx: [
                (src: "src", dst: "regular", msg: "msg::A"),
                (src: "regular", dst: "stateless", msg: "msg::B"),
                (src: "stateless", dst: "sink", msg: "msg::C"),
            ]
        )"#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.get_graph(None).unwrap();

    assert_eq!(
        graph
            .get_node(graph.get_node_id_by_name("src").unwrap())
            .unwrap()
            .get_declared_task_kind(),
        Some(TaskKind::Source)
    );
    assert_eq!(
        graph
            .get_node(graph.get_node_id_by_name("regular").unwrap())
            .unwrap()
            .get_declared_task_kind(),
        Some(TaskKind::Regular)
    );
    assert_eq!(
        graph
            .get_node(graph.get_node_id_by_name("stateless").unwrap())
            .unwrap()
            .get_declared_task_kind(),
        Some(TaskKind::Stateless)
    );
    assert_eq!(
        graph
            .get_node(graph.get_node_id_by_name("sink").unwrap())
            .unwrap()
            .get_declared_task_kind(),
        Some(TaskKind::Sink)
    );

    let serialized = config.serialize_ron().unwrap();
    assert!(serialized.contains("kind: source"));
    assert!(serialized.contains("kind: task"));
    assert!(serialized.contains("kind: stateless_task"));
    assert!(serialized.contains("kind: sink"));
}

#[test]
fn test_stateless_task_rejects_background_and_anytime_modes() {
    for (attribute, expected) in [
        ("background: true", "cannot be backgrounded"),
        ("anytime: (max_refines: 1)", "cannot use an anytime policy"),
    ] {
        let txt = format!(
            r#"(
                    tasks: [
                        (id: "src", type: "a"),
                        (id: "transform", type: "b", kind: stateless_task, {attribute}),
                        (id: "sink", type: "c"),
                    ],
                    cnx: [
                        (src: "src", dst: "transform", msg: "msg::A"),
                        (src: "transform", dst: "sink", msg: "msg::B"),
                    ],
                )"#
        );
        let err = read_configuration_str(txt, None).expect_err("config should fail");
        assert!(
            err.to_string().contains(expected),
            "unexpected error for {attribute}: {err}"
        );
    }
}

#[test]
fn test_resolve_task_kind_uses_nc_outputs_for_regular_tasks() {
    let txt = r#"(
            tasks: [
                (id: "src", type: "a"),
                (id: "regular", type: "b"),
            ],
            cnx: [
                (src: "src", dst: "regular", msg: "msg::A"),
                (src: "regular", dst: "__nc__", msg: "msg::B"),
            ]
        )"#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.get_graph(None).unwrap();
    let regular_id = graph.get_node_id_by_name("regular").unwrap();

    assert_eq!(
        resolve_task_kind_for_id(graph, regular_id).unwrap(),
        TaskKind::Regular
    );
}

#[test]
fn test_resolve_task_kind_rejects_isolated_task_without_kind() {
    let txt = r#"(
            tasks: [
                (id: "lonely", type: "a"),
            ],
            cnx: []
        )"#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.get_graph(None).unwrap();
    let lonely_id = graph.get_node_id_by_name("lonely").unwrap();

    let err = resolve_task_kind_for_id(graph, lonely_id).expect_err("expected task kind error");
    assert!(
        err.to_string()
            .contains("cannot infer whether it is a source, task, or sink"),
        "unexpected error: {err}"
    );
}

#[test]
fn test_resolve_explicit_source_kind_allows_missing_declared_outputs() {
    let txt = r#"(
            tasks: [
                (id: "src", type: "a", kind: source),
            ],
            cnx: []
        )"#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.get_graph(None).unwrap();
    let src_id = graph.get_node_id_by_name("src").unwrap();

    assert_eq!(
        resolve_task_kind_for_id(graph, src_id).unwrap(),
        TaskKind::Source
    );
}

#[test]
fn test_resolve_explicit_regular_kind_allows_missing_declared_outputs() {
    let txt = r#"(
            tasks: [
                (id: "src", type: "a"),
                (id: "regular", type: "b", kind: task),
            ],
            cnx: [
                (src: "src", dst: "regular", msg: "msg::A"),
            ]
        )"#;

    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.get_graph(None).unwrap();
    let regular_id = graph.get_node_id_by_name("regular").unwrap();

    assert_eq!(
        resolve_task_kind_for_id(graph, regular_id).unwrap(),
        TaskKind::Regular
    );
}

#[test]
fn test_runtime_rate_target_rejects_zero() {
    let txt = r#"(
            tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
            cnx: [(src: "src", dst: "sink", msg: "msg::A")],
            runtime: (rate_target_hz: 0)
        )"#;

    let err =
        read_configuration_str(txt.to_string(), None).expect_err("runtime config should fail");
    assert!(
        err.to_string()
            .contains("Runtime rate target cannot be zero"),
        "unexpected error: {err}"
    );
}

#[test]
fn test_runtime_rate_target_rejects_above_nanosecond_resolution() {
    let txt = format!(
        r#"(
                tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
                cnx: [(src: "src", dst: "sink", msg: "msg::A")],
                runtime: (rate_target_hz: {})
            )"#,
        MAX_RATE_TARGET_HZ + 1
    );

    let err = read_configuration_str(txt, None).expect_err("runtime config should fail");
    assert!(
        err.to_string().contains("exceeds the supported maximum"),
        "unexpected error: {err}"
    );
}

/// Builds a src -> any -> sink config with the given `anytime:` policy body,
/// extra node attributes (e.g. `, background: true`) and top-level extras
/// (e.g. `runtime: (rate_target_hz: 100),`).
fn anytime_config_txt(policy: &str, node_attrs: &str, top_level: &str) -> String {
    format!(
        r#"(
            tasks: [
                (id: "src", type: "a"),
                (id: "any", type: "b", anytime: ({policy}){node_attrs}),
                (id: "sink", type: "c"),
            ],
            cnx: [
                (src: "src", dst: "any", msg: "msg::A"),
                (src: "any", dst: "sink", msg: "msg::B"),
            ],
            {top_level}
        )"#
    )
}

fn expect_anytime_error(txt: String, expected: &str) {
    let err = read_configuration_str(txt, None).expect_err("anytime config should fail");
    assert!(
        err.to_string().contains(expected),
        "unexpected error: {err}"
    );
}

#[test]
fn test_anytime_node_parses_and_exposes_policy() {
    let txt = anytime_config_txt(
        r#"
                time_budget_ms: 8.0,
                max_age_ms: 100.0,
                quality_target: 0.95,
                quality_floor: 0.30,
                max_refines: 64,
                max_stall: 4,
            "#,
        ", background: true",
        "",
    );
    let config = read_configuration_str(txt, None).unwrap();
    let graph = config.get_graph(None).unwrap();
    let node = graph
        .get_node(graph.get_node_id_by_name("any").unwrap())
        .unwrap();
    assert!(node.is_anytime());
    assert!(node.is_background());
    assert_eq!(
        node.anytime().unwrap(),
        &AnytimeConfig {
            time_budget_ms: Some(8.0),
            max_age_ms: Some(100.0),
            quality_target: Some(0.95),
            quality_floor: Some(0.30),
            max_refines: Some(64),
            max_stall: Some(4),
        }
    );
    let src = graph
        .get_node(graph.get_node_id_by_name("src").unwrap())
        .unwrap();
    assert!(!src.is_anytime());
    assert!(src.anytime().is_none());
}

#[test]
fn test_anytime_typical_perception_config_is_accepted() {
    // The doc's typical perception config; foreground placement compiles to
    // a static plan, so max_refines is part of the minimum foreground set.
    let txt = anytime_config_txt(
        "max_age_ms: 100.0, quality_target: 0.9, max_refines: 22",
        "",
        "",
    );
    let config = read_configuration_str(txt, None).unwrap();
    let graph = config.get_graph(None).unwrap();
    let node = graph
        .get_node(graph.get_node_id_by_name("any").unwrap())
        .unwrap();
    let anytime = node.anytime().unwrap();
    assert_eq!(anytime.max_age_ms, Some(100.0));
    assert_eq!(anytime.quality_target, Some(0.9));
    assert_eq!(anytime.max_refines, Some(22));
    assert_eq!(anytime.time_budget_ms, None);
}

#[test]
fn test_anytime_arity_is_one_input_one_output() {
    // Two inputs: the runner cannot pick a Tov anchor.
    let two_inputs = r#"(
            tasks: [
                (id: "src_a", type: "a"),
                (id: "src_b", type: "a"),
                (id: "any", type: "b", anytime: (max_refines: 2)),
                (id: "sink", type: "c"),
            ],
            cnx: [
                (src: "src_a", dst: "any", msg: "msg::A"),
                (src: "src_b", dst: "any", msg: "msg::A"),
                (src: "any", dst: "sink", msg: "msg::B"),
            ],
        )"#;
    expect_anytime_error(
        two_inputs.to_string(),
        "exactly one input connection (found 2)",
    );

    // Two output message types: refine() has no single slot to rewrite.
    let two_outputs = r#"(
            tasks: [
                (id: "src", type: "a"),
                (id: "any", type: "b", anytime: (max_refines: 2)),
                (id: "sink_a", type: "c"),
                (id: "sink_b", type: "c"),
            ],
            cnx: [
                (src: "src", dst: "any", msg: "msg::A"),
                (src: "any", dst: "sink_a", msg: "msg::B"),
                (src: "any", dst: "sink_b", msg: "msg::C"),
            ],
        )"#;
    expect_anytime_error(
        two_outputs.to_string(),
        "exactly one output message type (found 2)",
    );

    // Fan-out of ONE output type to two consumers stays legal.
    let fan_out = r#"(
            tasks: [
                (id: "src", type: "a"),
                (id: "any", type: "b", anytime: (max_refines: 2)),
                (id: "sink_a", type: "c"),
                (id: "sink_b", type: "c"),
            ],
            cnx: [
                (src: "src", dst: "any", msg: "msg::A"),
                (src: "any", dst: "sink_a", msg: "msg::B"),
                (src: "any", dst: "sink_b", msg: "msg::B"),
            ],
        )"#;
    read_configuration_str(fan_out.to_string(), None).unwrap();
}

#[test]
fn test_anytime_foreground_needs_max_refines() {
    // A time-only hard bound cannot produce a static plan in the foreground.
    expect_anytime_error(
        anytime_config_txt("max_age_ms: 100.0, quality_target: 0.9", "", ""),
        "needs anytime.max_refines",
    );
    // Background placement has no static refine schedule to emit.
    let background = anytime_config_txt("max_age_ms: 100.0", ", background: true", "");
    read_configuration_str(background, None).unwrap();
}

#[test]
fn test_anytime_survives_serialize_roundtrip() {
    let txt = anytime_config_txt("time_budget_ms: 8.0, max_refines: 64", "", "");
    let config = CuConfig::deserialize_ron(&txt).unwrap();
    let serialized = config.serialize_ron().unwrap();
    let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
    let graph = deserialized.get_graph(None).unwrap();
    let node = graph
        .get_node(graph.get_node_id_by_name("any").unwrap())
        .unwrap();
    assert_eq!(
        node.anytime().unwrap(),
        &AnytimeConfig {
            time_budget_ms: Some(8.0),
            max_age_ms: None,
            quality_target: None,
            quality_floor: None,
            max_refines: Some(64),
            max_stall: None,
        }
    );
}

#[test]
fn test_anytime_rejects_missing_hard_bound() {
    expect_anytime_error(
        anytime_config_txt("quality_target: 0.9, max_stall: 4", "", ""),
        "needs at least one hard bound",
    );
}

#[test]
fn test_anytime_rejects_nan_quality_target() {
    expect_anytime_error(
        anytime_config_txt("time_budget_ms: 8.0, quality_target: NaN", "", ""),
        "anytime.quality_target must be within (0.0, 1.0]",
    );
}

#[test]
fn test_anytime_rejects_non_positive_times() {
    expect_anytime_error(
        anytime_config_txt("time_budget_ms: 0.0", "", ""),
        "anytime.time_budget_ms must be a positive",
    );
    expect_anytime_error(
        anytime_config_txt("max_age_ms: -5.0", "", ""),
        "anytime.max_age_ms must be a positive",
    );
    expect_anytime_error(
        anytime_config_txt("time_budget_ms: inf", "", ""),
        "anytime.time_budget_ms must be a positive",
    );
}

#[test]
fn test_anytime_rejects_zero_counts() {
    expect_anytime_error(
        anytime_config_txt("max_refines: 0", "", ""),
        "anytime.max_refines must be at least 1",
    );
    expect_anytime_error(
        anytime_config_txt("max_refines: 4, max_stall: 0", "", ""),
        "anytime.max_stall must be at least 1",
    );
}

#[test]
fn test_anytime_quality_ranges() {
    // target is (0.0, 1.0]: exactly 1.0 is fine, 0.0 is not.
    let ok = anytime_config_txt(
        "time_budget_ms: 8.0, quality_target: 1.0, max_refines: 4",
        "",
        "",
    );
    read_configuration_str(ok, None).unwrap();
    expect_anytime_error(
        anytime_config_txt("time_budget_ms: 8.0, quality_target: 0.0", "", ""),
        "anytime.quality_target must be within (0.0, 1.0]",
    );
    // floor is (0.0, 1.0): exactly 1.0 is rejected.
    expect_anytime_error(
        anytime_config_txt("time_budget_ms: 8.0, quality_floor: 1.0", "", ""),
        "anytime.quality_floor must be within (0.0, 1.0)",
    );
}

#[test]
fn test_anytime_rejects_floor_above_target() {
    expect_anytime_error(
        anytime_config_txt(
            "time_budget_ms: 8.0, quality_target: 0.5, quality_floor: 0.8",
            "",
            "",
        ),
        "must not exceed anytime.quality_target",
    );
}

#[test]
fn test_anytime_rejects_sources_and_sinks() {
    let on_source = r#"(
            tasks: [
                (id: "src", type: "a", anytime: (max_refines: 4)),
                (id: "sink", type: "b"),
            ],
            cnx: [(src: "src", dst: "sink", msg: "msg::A")],
        )"#;
    expect_anytime_error(on_source.to_string(), "only supported on regular tasks");

    let on_sink = r#"(
            tasks: [
                (id: "src", type: "a"),
                (id: "sink", type: "b", anytime: (max_refines: 4)),
            ],
            cnx: [(src: "src", dst: "sink", msg: "msg::A")],
        )"#;
    expect_anytime_error(on_sink.to_string(), "only supported on regular tasks");
}

#[test]
fn test_anytime_foreground_rate_limited_needs_time_bound() {
    expect_anytime_error(
        anytime_config_txt("max_refines: 64", "", "runtime: (rate_target_hz: 100),"),
        "needs a time bound",
    );
}

#[test]
fn test_anytime_foreground_window_must_fit_period() {
    expect_anytime_error(
        anytime_config_txt(
            "time_budget_ms: 12.0, max_refines: 8",
            "",
            "runtime: (rate_target_hz: 100),",
        ),
        "does not fit within",
    );
    // The worst-case window is min(time_budget_ms, max_age_ms).
    let ok = anytime_config_txt(
        "time_budget_ms: 20.0, max_age_ms: 5.0, max_refines: 8",
        "",
        "runtime: (rate_target_hz: 100),",
    );
    read_configuration_str(ok, None).unwrap();
}

#[test]
fn test_anytime_background_exempt_from_fit_check() {
    let txt = anytime_config_txt(
        "max_refines: 64",
        ", background: true",
        "runtime: (rate_target_hz: 100),",
    );
    read_configuration_str(txt, None).unwrap();
}

#[test]
fn test_anytime_no_rate_target_accepts_refines_only_foreground() {
    let txt = anytime_config_txt("max_refines: 64", "", "");
    read_configuration_str(txt, None).unwrap();
}

#[test]
fn test_anytime_validated_per_mission_graph() {
    let txt = r#"(
            missions: [(id: "A"), (id: "B")],
            tasks: [
                (id: "src", type: "a"),
                (id: "any", type: "b", missions: ["B"], anytime: (quality_target: 0.9)),
                (id: "sink", type: "c"),
            ],
            cnx: [
                (src: "src", dst: "any", msg: "msg::A", missions: ["B"]),
                (src: "any", dst: "sink", msg: "msg::B", missions: ["B"]),
                (src: "src", dst: "sink", msg: "msg::A", missions: ["A"]),
            ],
        )"#;
    expect_anytime_error(txt.to_string(), "needs at least one hard bound");
}

#[test]
fn test_nc_connection_marks_source_output_without_creating_edge() {
    let txt = r#"(
            tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
            cnx: [
                (src: "src", dst: "sink", msg: "msg::A"),
                (src: "src", dst: "__nc__", msg: "msg::B"),
            ]
        )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.get_graph(None).unwrap();
    let src_id = graph.get_node_id_by_name("src").unwrap();
    let src_node = graph.get_node(src_id).unwrap();

    assert_eq!(graph.edge_count(), 1);
    assert_eq!(src_node.nc_outputs(), &["msg::B".to_string()]);
}

#[test]
fn test_nc_connection_survives_serialize_roundtrip() {
    let txt = r#"(
            tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
            cnx: [
                (src: "src", dst: "sink", msg: "msg::A"),
                (src: "src", dst: "__nc__", msg: "msg::B"),
            ]
        )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let serialized = config.serialize_ron().unwrap();
    let deserialized = CuConfig::deserialize_ron(&serialized).unwrap();
    let graph = deserialized.get_graph(None).unwrap();
    let src_id = graph.get_node_id_by_name("src").unwrap();
    let src_node = graph.get_node(src_id).unwrap();

    assert_eq!(graph.edge_count(), 1);
    assert_eq!(src_node.nc_outputs(), &["msg::B".to_string()]);
}

#[test]
fn test_nc_connection_preserves_original_connection_order() {
    let txt = r#"(
            tasks: [(id: "src", type: "a"), (id: "sink", type: "b")],
            cnx: [
                (src: "src", dst: "__nc__", msg: "msg::A"),
                (src: "src", dst: "sink", msg: "msg::B"),
            ]
        )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.get_graph(None).unwrap();
    let src_id = graph.get_node_id_by_name("src").unwrap();
    let src_node = graph.get_node(src_id).unwrap();
    let edge_id = graph.get_src_edges(src_id).unwrap()[0];
    let edge = graph.edge(edge_id).unwrap();

    assert_eq!(edge.msg, "msg::B");
    assert_eq!(edge.order, 1);
    assert_eq!(
        src_node
            .nc_outputs_with_order()
            .map(|(msg, order)| (msg.as_str(), order))
            .collect::<Vec<_>>(),
        vec![("msg::A", 0)]
    );
}

#[cfg(feature = "std")]
fn multi_config_test_dir(name: &str) -> PathBuf {
    let unique = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system time before unix epoch")
        .as_nanos();
    let dir = std::env::temp_dir().join(format!("cu29_multi_config_{name}_{unique}"));
    std::fs::create_dir_all(&dir).expect("create temp test dir");
    dir
}

#[cfg(feature = "std")]
fn write_multi_config_file(dir: &Path, name: &str, contents: &str) -> PathBuf {
    let path = dir.join(name);
    std::fs::write(&path, contents).expect("write temp config file");
    path
}

#[cfg(feature = "std")]
fn alpha_subsystem_config() -> &'static str {
    r#"(
            tasks: [
                (id: "src", type: "demo::Src"),
                (id: "sink", type: "demo::Sink"),
            ],
            bridges: [
                (
                    id: "zenoh",
                    type: "demo::ZenohBridge",
                    channels: [
                        Tx(id: "ping"),
                        Rx(id: "pong"),
                    ],
                ),
            ],
            cnx: [
                (src: "src", dst: "zenoh/ping", msg: "demo::Ping"),
                (src: "zenoh/pong", dst: "sink", msg: "demo::Pong"),
            ],
        )"#
}

#[cfg(feature = "std")]
fn beta_subsystem_config() -> &'static str {
    r#"(
            tasks: [
                (id: "responder", type: "demo::Responder"),
            ],
            bridges: [
                (
                    id: "zenoh",
                    type: "demo::ZenohBridge",
                    channels: [
                        Rx(id: "ping"),
                        Tx(id: "pong"),
                    ],
                ),
            ],
            cnx: [
                (src: "zenoh/ping", dst: "responder", msg: "demo::Ping"),
                (src: "responder", dst: "zenoh/pong", msg: "demo::Pong"),
            ],
        )"#
}

#[cfg(feature = "std")]
fn instance_override_subsystem_config() -> &'static str {
    r#"(
            tasks: [
                (
                    id: "imu",
                    type: "demo::ImuTask",
                    config: {
                        "sample_hz": 200,
                    },
                ),
            ],
            resources: [
                (
                    id: "board",
                    provider: "demo::BoardBundle",
                    config: {
                        "bus": "i2c-1",
                    },
                ),
            ],
            bridges: [
                (
                    id: "radio",
                    type: "demo::RadioBridge",
                    config: {
                        "mtu": 32,
                    },
                    channels: [
                        Tx(id: "tx"),
                        Rx(id: "rx"),
                    ],
                ),
            ],
            cnx: [
                (src: "imu", dst: "radio/tx", msg: "demo::Packet"),
                (src: "radio/rx", dst: "imu", msg: "demo::Packet"),
            ],
        )"#
}

#[cfg(feature = "std")]
#[test]
fn test_read_multi_configuration_assigns_stable_subsystem_codes() {
    let dir = multi_config_test_dir("stable_ids");
    write_multi_config_file(&dir, "alpha.ron", alpha_subsystem_config());
    write_multi_config_file(&dir, "beta.ron", beta_subsystem_config());
    let network_path = write_multi_config_file(
        &dir,
        "network.ron",
        r#"(
                subsystems: [
                    (id: "beta", config: "beta.ron"),
                    (id: "alpha", config: "alpha.ron"),
                ],
                interconnects: [
                    (from: "alpha/zenoh/ping", to: "beta/zenoh/ping", msg: "demo::Ping"),
                    (from: "beta/zenoh/pong", to: "alpha/zenoh/pong", msg: "demo::Pong"),
                ],
            )"#,
    );

    let config =
        read_multi_configuration(network_path.to_str().expect("network path utf8")).unwrap();

    let alpha = config.subsystem("alpha").expect("alpha subsystem missing");
    let beta = config.subsystem("beta").expect("beta subsystem missing");
    assert_eq!(alpha.subsystem_code, 0);
    assert_eq!(beta.subsystem_code, 1);
    assert_eq!(config.interconnects.len(), 2);
    assert_eq!(config.interconnects[0].bridge_type, "demo::ZenohBridge");
}

#[cfg(feature = "std")]
#[test]
fn test_multi_configuration_filters_interconnects_by_feature() {
    let dir = multi_config_test_dir("feature_interconnects");
    write_multi_config_file(&dir, "alpha.ron", alpha_subsystem_config());
    write_multi_config_file(&dir, "beta.ron", beta_subsystem_config());
    let network_path = write_multi_config_file(
        &dir,
        "network.ron",
        r#"(
                subsystems: [
                    (id: "alpha", config: "alpha.ron"),
                    (id: "beta", config: "beta.ron"),
                ],
                interconnects: [
                    (
                        from: "alpha/zenoh/ping",
                        to: "beta/zenoh/ping",
                        msg: "demo::Ping",
                        when: Feature("networked"),
                    ),
                    (
                        from: "beta/zenoh/pong",
                        to: "alpha/zenoh/pong",
                        msg: "demo::Pong",
                        when: Feature("networked"),
                    ),
                ],
            )"#,
    );

    let disconnected = read_multi_configuration_with_features(
        network_path.to_str().expect("network path utf8"),
        &[],
    )
    .unwrap();
    assert!(disconnected.interconnects.is_empty());

    let networked = read_multi_configuration_with_features(
        network_path.to_str().expect("network path utf8"),
        &["networked"],
    )
    .unwrap();
    assert_eq!(networked.interconnects.len(), 2);
}

#[cfg(feature = "std")]
#[test]
fn test_multi_configuration_uses_default_mission_contracts() {
    let dir = multi_config_test_dir("default_mission");
    write_multi_config_file(
        &dir,
        "alpha.ron",
        r#"(
                missions: [(id: "default"), (id: "diagnostics")],
                tasks: [
                    (id: "src", type: "demo::Src"),
                    (
                        id: "diagnostic",
                        type: "demo::Diagnostic",
                        missions: ["diagnostics"],
                    ),
                ],
                bridges: [
                    (
                        id: "zenoh",
                        type: "demo::ZenohBridge",
                        channels: [Tx(id: "ping")],
                    ),
                ],
                cnx: [
                    (src: "src", dst: "zenoh/ping", msg: "demo::Ping"),
                    (
                        src: "diagnostic",
                        dst: "__nc__",
                        msg: "demo::DiagnosticMessage",
                        missions: ["diagnostics"],
                    ),
                ],
            )"#,
    );
    write_multi_config_file(&dir, "beta.ron", beta_subsystem_config());
    let network_path = write_multi_config_file(
        &dir,
        "network.ron",
        r#"(
                subsystems: [
                    (id: "alpha", config: "alpha.ron"),
                    (id: "beta", config: "beta.ron"),
                ],
                interconnects: [
                    (from: "alpha/zenoh/ping", to: "beta/zenoh/ping", msg: "demo::Ping"),
                ],
            )"#,
    );

    let config =
        read_multi_configuration(network_path.to_str().expect("network path utf8")).unwrap();
    assert_eq!(config.interconnects.len(), 1);
}

#[cfg(feature = "std")]
#[test]
fn test_read_multi_configuration_rejects_wrong_direction() {
    let dir = multi_config_test_dir("wrong_direction");
    write_multi_config_file(&dir, "alpha.ron", alpha_subsystem_config());
    write_multi_config_file(&dir, "beta.ron", beta_subsystem_config());
    let network_path = write_multi_config_file(
        &dir,
        "network.ron",
        r#"(
                subsystems: [
                    (id: "alpha", config: "alpha.ron"),
                    (id: "beta", config: "beta.ron"),
                ],
                interconnects: [
                    (from: "alpha/zenoh/pong", to: "beta/zenoh/ping", msg: "demo::Pong"),
                ],
            )"#,
    );

    let err = read_multi_configuration(network_path.to_str().expect("network path utf8"))
        .expect_err("direction mismatch should fail");

    assert!(
        err.to_string()
            .contains("must reference a Tx bridge channel"),
        "unexpected error: {err}"
    );
}

#[cfg(feature = "std")]
#[test]
fn test_read_multi_configuration_rejects_declared_message_mismatch() {
    let dir = multi_config_test_dir("msg_mismatch");
    write_multi_config_file(&dir, "alpha.ron", alpha_subsystem_config());
    write_multi_config_file(&dir, "beta.ron", beta_subsystem_config());
    let network_path = write_multi_config_file(
        &dir,
        "network.ron",
        r#"(
                subsystems: [
                    (id: "alpha", config: "alpha.ron"),
                    (id: "beta", config: "beta.ron"),
                ],
                interconnects: [
                    (from: "alpha/zenoh/ping", to: "beta/zenoh/ping", msg: "demo::Wrong"),
                ],
            )"#,
    );

    let err = read_multi_configuration(network_path.to_str().expect("network path utf8"))
        .expect_err("message mismatch should fail");

    assert!(
        err.to_string()
            .contains("declares message type 'demo::Wrong'"),
        "unexpected error: {err}"
    );
}

#[cfg(feature = "std")]
#[test]
fn test_read_multi_configuration_resolves_instance_override_root() {
    let dir = multi_config_test_dir("instance_root");
    write_multi_config_file(&dir, "robot.ron", instance_override_subsystem_config());
    let network_path = write_multi_config_file(
        &dir,
        "multi_copper.ron",
        r#"(
                subsystems: [
                    (id: "robot", config: "robot.ron"),
                ],
                interconnects: [],
                instance_overrides_root: "instances",
            )"#,
    );

    let config =
        read_multi_configuration(network_path.to_str().expect("network path utf8")).unwrap();

    assert_eq!(
        config.instance_overrides_root.as_deref().map(Path::new),
        Some(dir.join("instances").as_path())
    );
}

#[cfg(feature = "std")]
#[test]
fn test_resolve_subsystem_config_for_instance_applies_overrides() {
    let dir = multi_config_test_dir("instance_apply");
    write_multi_config_file(&dir, "robot.ron", instance_override_subsystem_config());
    let instances_dir = dir.join("instances").join("17");
    std::fs::create_dir_all(&instances_dir).expect("create instance dir");
    write_multi_config_file(
        &instances_dir,
        "robot.ron",
        r#"(
                set: [
                    (
                        path: "tasks/imu/config",
                        value: {
                            "gyro_bias": [0.1, -0.2, 0.3],
                        },
                    ),
                    (
                        path: "resources/board/config",
                        value: {
                            "bus": "robot17-imu",
                        },
                    ),
                    (
                        path: "bridges/radio/config",
                        value: {
                            "mtu": 64,
                        },
                    ),
                ],
            )"#,
    );
    let network_path = write_multi_config_file(
        &dir,
        "multi_copper.ron",
        r#"(
                subsystems: [
                    (id: "robot", config: "robot.ron"),
                ],
                interconnects: [],
                instance_overrides_root: "instances",
            )"#,
    );

    let multi =
        read_multi_configuration(network_path.to_str().expect("network path utf8")).unwrap();
    let effective = multi
        .resolve_subsystem_config_for_instance("robot", 17)
        .expect("effective config");

    let graph = effective.get_graph(None).expect("graph");
    let imu_id = graph.get_node_id_by_name("imu").expect("imu node");
    let imu = graph.get_node(imu_id).expect("imu weight");
    let imu_cfg = imu.get_instance_config().expect("imu config");
    assert_eq!(imu_cfg.get::<u64>("sample_hz").unwrap(), Some(200));
    let gyro_bias: Vec<f64> = imu_cfg
        .get_value("gyro_bias")
        .expect("gyro_bias deserialize")
        .expect("gyro_bias value");
    assert_eq!(gyro_bias, vec![0.1, -0.2, 0.3]);

    let board = effective
        .resources
        .iter()
        .find(|resource| resource.id == "board")
        .expect("board resource");
    assert_eq!(
        board.config.as_ref().unwrap().get::<String>("bus").unwrap(),
        Some("robot17-imu".to_string())
    );

    let radio = effective
        .bridges
        .iter()
        .find(|bridge| bridge.id == "radio")
        .expect("radio bridge");
    assert_eq!(
        radio.config.as_ref().unwrap().get::<u64>("mtu").unwrap(),
        Some(64)
    );

    let radio_id = graph.get_node_id_by_name("radio").expect("radio node");
    let radio_node = graph.get_node(radio_id).expect("radio weight");
    assert_eq!(
        radio_node
            .get_instance_config()
            .unwrap()
            .get::<u64>("mtu")
            .unwrap(),
        Some(64)
    );
}

#[cfg(feature = "std")]
#[test]
fn test_resolve_subsystem_config_for_instance_rejects_unknown_path() {
    let dir = multi_config_test_dir("instance_unknown");
    write_multi_config_file(&dir, "robot.ron", instance_override_subsystem_config());
    let instances_dir = dir.join("instances").join("17");
    std::fs::create_dir_all(&instances_dir).expect("create instance dir");
    write_multi_config_file(
        &instances_dir,
        "robot.ron",
        r#"(
                set: [
                    (
                        path: "tasks/missing/config",
                        value: {
                            "gyro_bias": [1.0, 2.0, 3.0],
                        },
                    ),
                ],
            )"#,
    );
    let network_path = write_multi_config_file(
        &dir,
        "multi_copper.ron",
        r#"(
                subsystems: [
                    (id: "robot", config: "robot.ron"),
                ],
                interconnects: [],
                instance_overrides_root: "instances",
            )"#,
    );

    let multi =
        read_multi_configuration(network_path.to_str().expect("network path utf8")).unwrap();
    let err = multi
        .resolve_subsystem_config_for_instance("robot", 17)
        .expect_err("unknown task override should fail");

    assert!(
        err.to_string().contains("targets unknown task 'missing'"),
        "unexpected error: {err}"
    );
}

#[test]
fn test_thread_pools_parse_and_round_trip() {
    let txt = r#"(
            runtime: (
                rate_target_hz: 1000,
                thread_pools: [
                    ( id: "rt",         threads: 4, affinity: [2, 3, 4, 5], policy: Fifo(priority: 80) ),
                    ( id: "background", threads: 2, affinity: [0, 1] ),
                    ( id: "vision",     threads: 2, policy: Nice(10), on_error: Strict ),
                ],
            ),
            tasks: [ ( id: "t", type: "tasks::Foo" ) ],
        )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let runtime = config.runtime.as_ref().expect("runtime config");
    assert_eq!(runtime.thread_pools.len(), 3);

    let rt = &runtime.thread_pools[0];
    assert_eq!(rt.id, "rt");
    assert_eq!(rt.threads, 4);
    assert_eq!(rt.affinity.as_deref(), Some([2, 3, 4, 5].as_slice()));
    assert_eq!(rt.policy, SchedulingPolicy::Fifo { priority: 80 });
    assert_eq!(rt.on_error, OnError::Warn);

    let bg = &runtime.thread_pools[1];
    assert_eq!(bg.id, "background");
    assert_eq!(bg.policy, SchedulingPolicy::Fair);

    let vision = &runtime.thread_pools[2];
    assert_eq!(vision.policy, SchedulingPolicy::Nice(10));
    assert_eq!(vision.affinity, None);
    assert_eq!(vision.on_error, OnError::Strict);

    // Round-trips through serialization.
    let serialized = config.serialize_ron().unwrap();
    let reparsed = CuConfig::deserialize_ron(&serialized).unwrap();
    assert_eq!(
        reparsed.runtime.as_ref().unwrap().thread_pools,
        runtime.thread_pools
    );
}

#[test]
fn test_background_flag_and_pool_forms() {
    let txt = r#"(
            tasks: [
                ( id: "a", type: "tasks::Foo", background: true ),
                ( id: "b", type: "tasks::Foo", background: (pool: "vision") ),
                ( id: "c", type: "tasks::Foo" ),
            ],
            cnx: [],
        )"#;
    let config = CuConfig::deserialize_ron(txt).unwrap();
    let graph = config.get_graph(None).unwrap();

    let a = graph.get_node(0).unwrap();
    assert!(a.is_background());
    assert_eq!(a.background_pool(), DEFAULT_BACKGROUND_POOL);

    let b = graph.get_node(1).unwrap();
    assert!(b.is_background());
    assert_eq!(b.background_pool(), "vision");

    let c = graph.get_node(2).unwrap();
    assert!(!c.is_background());
    assert_eq!(c.background_pool(), DEFAULT_BACKGROUND_POOL);
}

#[test]
fn test_thread_pool_validation_rejects_bad_configs() {
    let cases = [
        (
            r#"( runtime: ( thread_pools: [ ( id: "rt", threads: 0 ) ] ), tasks: [] )"#,
            "at least 1 thread",
        ),
        (
            r#"( runtime: ( thread_pools: [ ( id: "a", threads: 1 ), ( id: "a", threads: 1 ) ] ), tasks: [] )"#,
            "Duplicate thread pool id",
        ),
        (
            r#"( runtime: ( thread_pools: [ ( id: "rt", threads: 1, policy: Fifo(priority: 200) ) ] ), tasks: [] )"#,
            "out of range",
        ),
        (
            r#"( runtime: ( thread_pools: [ ( id: "rt", threads: 1, affinity: [] ) ] ), tasks: [] )"#,
            "empty affinity",
        ),
    ];

    for (txt, expected) in cases {
        let err =
            CuConfig::deserialize_ron(txt).expect_err("expected thread pool validation to fail");
        assert!(
            err.to_string().contains(expected),
            "error '{err}' did not contain '{expected}'"
        );
    }
}

#[cfg(feature = "std")]
#[test]
fn test_default_background_pool_injected_for_background_tasks() {
    let txt = r#"(
            tasks: [
                ( id: "src", type: "tasks::Src" ),
                ( id: "bg",  type: "tasks::Task", background: true ),
            ],
            cnx: [
                ( src: "src", dst: "bg", msg: "i32" ),
                ( src: "bg", dst: "__nc__", msg: "i32" ),
            ],
        )"#;
    let config = read_configuration_str(txt.to_string(), None).unwrap();
    let pools = &config.runtime.as_ref().unwrap().thread_pools;
    let background: Vec<_> = pools
        .iter()
        .filter(|p| p.id == DEFAULT_BACKGROUND_POOL)
        .collect();
    assert_eq!(background.len(), 1);
    assert_eq!(background[0].threads, 2);
    // Thread pools are owned by the runtime, not the resource manager — no
    // synthetic "threadpool" bundle should be injected.
    assert!(!config.resources.iter().any(|b| b.id == "threadpool"));
}
