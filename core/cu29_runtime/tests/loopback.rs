#[cfg(all(test, feature = "std"))]
mod tests {
    use cu29_runtime::config::read_configuration;
    use cu29_runtime::curuntime::compute_runtime_plan;
    use std::path::PathBuf;

    #[test]
    fn test_loopback_config_rejected() {
        let config_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests")
            .join("loopback_config.ron");
        let config =
            read_configuration(config_path.to_str().unwrap()).expect("config should parse");
        let graph = config.get_graph(None).expect("graph should load");
        let err = compute_runtime_plan(graph).expect_err("loopback should fail");
        let msg = err.to_string();
        assert!(msg.contains("loopback"), "unexpected error: {msg}");
        assert!(msg.contains("Missing"), "unexpected error: {msg}");
    }
}
