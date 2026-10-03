//! Exercise the installed CLI contract and process exit status against V1 fixtures.
#![cfg(feature = "self-describing-logs")]

use bincode::Encode;
use bincode::enc::write::Writer;
use cu29::prelude::*;
use std::path::Path;
use std::process::{Command, Output};
use std::sync::{Arc, Mutex};

struct WireBytes(Vec<u8>);
impl Encode for WireBytes {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        encoder.writer().write(&self.0)
    }
}
fn fixture(path: &Path, catalog: bool, corrupt: bool, duplicate: bool) {
    let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
        .file_base_name(path)
        .write(true)
        .create(true)
        .preallocated_size(64 * 1024)
        .build()
        .unwrap()
    else {
        panic!("writer")
    };
    let logger = Arc::new(Mutex::new(logger));
    if catalog {
        let blob = include_bytes!("../../cu29_value/tests/fixtures/catalog_v1.bin");
        write_value_decode_catalog(logger.clone(), blob).unwrap();
        if duplicate {
            write_value_decode_catalog(logger.clone(), blob).unwrap();
        }
    }
    let mut cls = stream_write::<WireBytes, _>(logger, UnifiedLogType::CopperList, 1024).unwrap();
    cls.log(&WireBytes(vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0]))
        .unwrap();
    if corrupt {
        cls.log(&WireBytes(vec![1, 0])).unwrap();
    }
}
fn run(path: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cu29-logextract"))
        .arg(path)
        .args(args)
        .output()
        .unwrap()
}
#[test]
fn test_machine_output_and_process_failure_contract() {
    let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
    let good = dir.path().join("good.copper");
    fixture(&good, true, false, false);
    for format in ["json", "jsonl"] {
        let output = run(&good, &["extract-copperlists", "--export-format", format]);
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        assert!(output.stderr.is_empty());
        let value: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
        if format == "json" {
            assert_eq!(value[0]["id"], 0);
        } else {
            assert_eq!(value["id"], 0);
        }
    }
    let output = run(
        &good,
        &["catalog", "--export-format", "ron", "--color", "always"],
    );
    assert!(output.status.success());
    assert!(!output.stdout.contains(&0x1b));
    let basic = run(&good, &["fsck"]);
    assert!(basic.status.success());
    let basic = String::from_utf8(basic.stdout).unwrap();
    assert!(basic.contains("# of Catalogs"));
    let catalog_bytes = include_bytes!("../../cu29_value/tests/fixtures/catalog_v1.bin").len();
    assert!(basic.contains(&format!("Catalog total size    -> {catalog_bytes} bytes")));
    let deep = run(&good, &["fsck", "--deep"]);
    assert!(deep.status.success());
    let deep = String::from_utf8(deep.stdout).unwrap();
    assert!(deep.contains("\n\n  Deep validation"));
    assert!(deep.contains("-> passed"));
    assert!(deep.contains("# of captured payloads -> 0"));
    assert!(deep.contains("Payload total size     -> 0 bytes"));
    assert!(!deep.contains("frozen task-state"));
    for (name, catalog, corrupt, duplicate) in [
        ("legacy", false, false, false),
        ("corrupt", true, true, false),
        ("duplicate", true, false, true),
    ] {
        let path = dir.path().join(format!("{name}.copper"));
        fixture(&path, catalog, corrupt, duplicate);
        let output = run(&path, &["fsck", "--deep"]);
        assert!(!output.status.success(), "{name}");
        assert!(!output.stderr.is_empty(), "{name}");
        if corrupt {
            let output = run(&path, &["extract-copperlists", "--export-format", "jsonl"]);
            assert!(!output.status.success());
            assert!(String::from_utf8_lossy(&output.stderr).contains("CopperList #1"));
        }
    }
}
