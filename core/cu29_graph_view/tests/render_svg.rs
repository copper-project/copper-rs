use cu29_graph_view::render_config_svg;
use cu29_runtime::config::CuConfig;
use std::process::Command;

const CONFIG: &str = r#"(
    tasks: [(id: "source", type: "Source"), (id: "sink", type: "Sink")],
    cnx: [(src: "source", dst: "sink", msg: "u32")],
)"#;

#[test]
fn public_renderer_matches_cli_output() {
    let config = CuConfig::deserialize_ron(CONFIG).unwrap();
    let svg = render_config_svg(&config, None).unwrap();
    assert!(svg.contains("<svg"));
    assert!(svg.contains("source"));
    assert!(svg.contains("sink"));
    assert_eq!(svg, render_config_svg(&config, Some("default")).unwrap());

    let artifact_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target");
    std::fs::create_dir_all(&artifact_dir).unwrap();
    let dir = tempfile::tempdir_in(artifact_dir).unwrap();
    let config_path = dir.path().join("graph.ron");
    let svg_path = dir.path().join("graph.svg");
    std::fs::write(&config_path, CONFIG).unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_cu29-graph-view"))
        .arg(&config_path)
        .arg("--output")
        .arg(&svg_path)
        .output()
        .unwrap();
    assert!(output.status.success(), "{output:?}");
    assert_eq!(std::fs::read_to_string(svg_path).unwrap(), svg);
}

#[test]
fn mission_selection_and_errors_are_available_to_library_callers() {
    let config = CuConfig::deserialize_ron(include_str!(
        "../../cu29_runtime/tests/render_resource_tables.ron"
    ))
    .unwrap();
    let all = render_config_svg(&config, None).unwrap();
    let alpha = render_config_svg(&config, Some("Alpha")).unwrap();
    let beta = render_config_svg(&config, Some("Beta")).unwrap();
    assert!(all.contains("Mission: Alpha"));
    assert!(all.contains("Mission: Beta"));
    assert!(alpha.contains("Mission: Alpha"));
    assert!(!alpha.contains("Mission: Beta"));
    assert!(beta.contains("Mission: Beta"));
    assert!(!beta.contains("Mission: Alpha"));
    let error = render_config_svg(&config, Some("missing")).unwrap_err();
    assert!(
        error
            .to_string()
            .contains("Available missions: Alpha, Beta")
    );

    let simple = CuConfig::deserialize_ron(CONFIG).unwrap();
    assert!(render_config_svg(&simple, Some("missing")).is_err());
}
