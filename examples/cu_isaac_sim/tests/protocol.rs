//! Python/Rust layout, lease, graph, and simulation-clock integration checks without a GPU.

#[test]
fn test_python_shared_memory_protocol() {
    let output = std::process::Command::new("python3")
        .arg(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/protocol.py"))
        .arg(env!("CARGO_BIN_EXE_cu-isaac-sim"))
        .output()
        .expect("python3 is required for the Isaac adapter protocol tests");
    assert!(
        output.status.success(),
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}
