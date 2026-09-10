use cu29_runtime::config::read_configuration_str;

fn vision_graph(camera_logging: bool, detect_replay: &str, detect_logging: bool) -> String {
    format!(
        r#"(
            tasks: [
                (id: "camera", type: "Camera", logging: (enabled: {camera_logging})),
                (id: "detect", type: "Detect", logging: (enabled: {detect_logging}), streaming: (replay: {detect_replay})),
                (id: "track", type: "Track", streaming: (replay: reconstruct)),
                (id: "plan", type: "Plan", streaming: (replay: reconstruct)),
            ],
            cnx: [
                (src: "camera", dst: "detect", msg: "Image"),
                (src: "detect", dst: "track", msg: "Detections"),
                (src: "track", dst: "plan", msg: "Tracks"),
                (src: "plan", dst: "__nc__", msg: "PlanOutput"),
            ],
        )"#
    )
}

#[test]
fn reconstruction_rejects_unlogged_inputs() {
    for (camera_logging, detect_replay, detect_logging, source, consumer) in [
        (false, "reconstruct", true, "camera", "detect"),
        (true, "capture", false, "detect", "track"),
        (true, "reconstruct", false, "detect", "track"),
    ] {
        let config = vision_graph(camera_logging, detect_replay, detect_logging);
        let error = read_configuration_str(config, None)
            .unwrap_err()
            .to_string();
        assert!(
            error.contains(&format!(
                "Task '{consumer}' uses streaming.replay: reconstruct"
            )),
            "{error}"
        );
        assert!(
            error.contains(&format!("from '{source}' has logging.enabled: false")),
            "{error}"
        );
    }
}

#[test]
fn captured_detections_allow_reconstruction_without_images() {
    read_configuration_str(vision_graph(false, "capture", true), None).unwrap();
}

#[test]
fn captured_images_allow_the_entire_downstream_chain_to_reconstruct() {
    read_configuration_str(vision_graph(true, "reconstruct", true), None).unwrap();
}

#[test]
fn reconstruction_checks_every_fan_in_edge() {
    let config = vision_graph(false, "capture", true).replace(
        "cnx: [",
        "cnx: [(src: \"camera\", dst: \"track\", msg: \"Image\"),",
    );
    let error = read_configuration_str(config, None)
        .unwrap_err()
        .to_string();
    assert!(
        error.contains("Task 'track' uses streaming.replay: reconstruct"),
        "{error}"
    );
    assert!(error.contains("input 'Image' from 'camera'"), "{error}");
}

#[test]
fn reconstruction_is_validated_in_each_mission() {
    let config = vision_graph(false, "reconstruct", true)
        .replacen(
            "tasks: [",
            "missions: [(id: \"safe\"), (id: \"broken\")], tasks: [",
            1,
        )
        .replace("msg: \"Image\"", "msg: \"Image\", missions: [\"broken\"]");
    let error = read_configuration_str(config, None)
        .unwrap_err()
        .to_string();
    assert!(error.contains("Mission 'broken'"), "{error}");
    assert!(error.contains("input 'Image' from 'camera'"), "{error}");
}
