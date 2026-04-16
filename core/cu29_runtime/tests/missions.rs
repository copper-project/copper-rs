#[cfg(all(test, feature = "std"))]
mod tests {
    use cu29_runtime::config::read_configuration;
    use cu29_runtime::curuntime::{CuExecutionUnit, compute_runtime_plan};
    use std::fs::{create_dir_all, write};
    use tempfile::tempdir;

    #[test]
    fn mission_scoped_inputs_change_sink_arity() {
        let temp_dir = tempdir().expect("temp dir");
        let config_dir = temp_dir.path().join("config");
        create_dir_all(&config_dir).expect("create config dir");

        let config = r#"(
            missions: [(id: "A"), (id: "B")],
            tasks: [
                (
                    id: "src_int",
                    type: "tasks::IntSource",
                ),
                (
                    id: "src_bool",
                    type: "tasks::BoolSource",
                    missions: ["B"],
                ),
                (
                    id: "sink",
                    type: "tasks::CombinedSink",
                ),
            ],
            cnx: [
                (
                    src: "src_int",
                    dst: "sink",
                    msg: "i32",
                ),
                (
                    src: "src_bool",
                    dst: "sink",
                    msg: "bool",
                    missions: ["B"],
                ),
            ],
        )"#;

        let config_path = config_dir.join("main.ron");
        write(&config_path, config).expect("write config");

        let config = read_configuration(config_path.to_str().expect("utf8 path"))
            .expect("configuration should parse");
        let mission_a = config.get_graph(Some("A")).expect("mission A graph");
        let mission_b = config.get_graph(Some("B")).expect("mission B graph");

        let runtime_a = compute_runtime_plan(mission_a).expect("mission A runtime plan");
        let runtime_b = compute_runtime_plan(mission_b).expect("mission B runtime plan");

        let sink_a = runtime_a
            .steps
            .iter()
            .find_map(|unit| match unit {
                CuExecutionUnit::Step(step) if step.node.get_id() == "sink" => Some(step),
                _ => None,
            })
            .expect("sink step for mission A");
        let sink_b = runtime_b
            .steps
            .iter()
            .find_map(|unit| match unit {
                CuExecutionUnit::Step(step) if step.node.get_id() == "sink" => Some(step),
                _ => None,
            })
            .expect("sink step for mission B");

        assert_eq!(sink_a.input_msg_indices_types.len(), 1);
        assert_eq!(sink_a.input_msg_indices_types[0].msg_type, "i32");

        assert_eq!(sink_b.input_msg_indices_types.len(), 2);
        assert_eq!(sink_b.input_msg_indices_types[0].msg_type, "i32");
        assert_eq!(sink_b.input_msg_indices_types[1].msg_type, "bool");
        assert_eq!(sink_b.input_msg_indices_types[0].connection_order, 0);
        assert_eq!(sink_b.input_msg_indices_types[1].connection_order, 1);
    }
}
