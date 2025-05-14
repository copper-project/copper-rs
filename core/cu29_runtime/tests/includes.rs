use cu29_runtime::config::read_configuration;
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

    let main_config = format!(
        r#"(
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
                    params: {{}},
                ),
            ],
        )"#
    );

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    let config = read_configuration(main_path.to_str().unwrap()).unwrap();

    let all_nodes = config.get_all_nodes(None);
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

    let main_config = format!(
        r#"(
            tasks: [],
            cnx: [],
            includes: [
                (
                    path: "included.ron",
                    params: {{
                        "instance_id": "42",
                        "param_value": 100,
                    }},
                ),
            ],
        )"#
    );

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    let config = read_configuration(main_path.to_str().unwrap()).unwrap();

    let all_nodes = config.get_all_nodes(None);
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

    let middle_config = format!(
        r#"(
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
                    params: {{}},
                ),
            ],
        )"#
    );

    let middle_path = config_dir.join("middle.ron");
    write(&middle_path, middle_config).unwrap();

    let main_config = format!(
        r#"(
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
                    params: {{}},
                ),
            ],
        )"#
    );

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    let config = read_configuration(main_path.to_str().unwrap()).unwrap();

    let all_nodes = config.get_all_nodes(None);
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

    let main_config = format!(
        r#"(
            tasks: [
                (
                    id: "common_task",
                    type: "tasks::MainTask",
                    config: {{
                        "param": 20,
                    }},
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
                    params: {{}},
                ),
            ],
        )"#
    );

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    let config = read_configuration(main_path.to_str().unwrap()).unwrap();

    let all_nodes = config.get_all_nodes(None);
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

    let main_config = format!(
        r#"(
            tasks: [],
            cnx: [],
            includes: [
                (
                    path: "invalid.ron",
                    params: {{}},
                ),
            ],
        )"#
    );

    let main_path = config_dir.join("main.ron");
    write(&main_path, main_config).unwrap();

    let result = read_configuration(main_path.to_str().unwrap());
    assert!(result.is_err());
}
