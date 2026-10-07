//! Exercise the installed CLI contract and process exit status against generated catalogs.
#![cfg(feature = "self-describing-logs")]

use bincode::Encode;
use bincode::enc::write::Writer;
use cu29::prelude::*;
use num_format::{Locale, ToFormattedString};
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
use bincode::value_decode::ValueDecodeRef;
use cu29_value::catalog_stream::{CatalogDescription, CatalogMission, CatalogSlot, write_catalog};
static EMPTY: CatalogDescription = CatalogDescription {
    layout: ValueDecodeCatalogLayout::Compact,
    missions: &[CatalogMission { slots: &[] }],
};
struct StartupCatalog<'a>(&'a CatalogDescription);
impl Encode for StartupCatalog<'_> {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        write_catalog(encoder.writer(), self.0)
    }
}
fn catalog_blob() -> Vec<u8> {
    let mut bytes = [0; 4096];
    let mut writer = bincode::enc::write::SliceWriter::new(&mut bytes);
    write_catalog(&mut writer, &EMPTY).unwrap();
    let size = writer.bytes_written();
    bytes[..size].to_vec()
}
fn metadata() -> ApplicationMetadata {
    ApplicationMetadata {
        app_type: "App".into(),
        app_name: "test".into(),
        app_version: "1".into(),
        git_commit: None,
        git_dirty: None,
        subsystem_id: None,
        subsystem_code: 0,
        effective_config_ron: "()".into(),
        missions: vec!["default".into()],
        catalog_offset: 0,
    }
}
fn fixture(path: &Path, catalog: bool, corrupt: bool, duplicate: bool) {
    let UnifiedLogger::Write(mut logger) = UnifiedLoggerBuilder::new()
        .file_base_name(path)
        .write(true)
        .create(true)
        .preallocated_size(64 * 1024)
        .build()
        .unwrap()
    else {
        panic!("writer")
    };
    if catalog {
        logger
            .seal_metadata(&metadata(), Some(&StartupCatalog(&EMPTY)))
            .unwrap();
    } else {
        logger.seal_metadata::<()>(&metadata(), None).unwrap();
    }
    let logger = Arc::new(Mutex::new(logger));
    if duplicate {
        let mut duplicate =
            LogStream::new(UnifiedLogType::ValueDecodeCatalog, logger.clone(), 1024).unwrap();
        duplicate.log(&WireBytes(catalog_blob())).unwrap();
    }
    let mut cls = LogStream::new(UnifiedLogType::CopperList, logger, 1024).unwrap();
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
    let catalog_bytes = catalog_blob().len();
    assert!(basic.contains(&format!("Catalog compressed    -> {catalog_bytes} bytes")));
    let catalog = cu29_export::catalog::read_value_decode_catalog(&good, None).unwrap();
    let decompressed_bytes = bincode::encode_to_vec(&catalog, bincode::config::standard())
        .unwrap()
        .len();
    let decompressed_line = format!(
        "Catalog decompressed  -> {} bytes",
        decompressed_bytes.to_formatted_string(&Locale::en)
    );
    assert!(basic.contains(&decompressed_line));
    let deep = run(&good, &["fsck", "--deep"]);
    assert!(deep.status.success());
    let deep = String::from_utf8(deep.stdout).unwrap();
    assert!(deep.contains(&decompressed_line));
    assert!(deep.contains("\n\n  Deep validation"));
    assert!(deep.contains("-> passed"));
    assert!(deep.contains("# of captured payloads -> 0"));
    assert!(deep.contains("Payload total size     -> 0 bytes"));
    assert!(!deep.contains("frozen task-state"));
    for (name, catalog, corrupt, duplicate) in [
        ("missing-catalog", false, false, false),
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

#[test]
fn static_catalog_spans_backing_files_as_one_section() {
    let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
    let path = dir.path().join("spanning.copper");
    let mut state = 7u32;
    let noise: String = (0..128 * 1024)
        .map(|_| {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            char::from(b'a' + (state % 26) as u8)
        })
        .collect();
    let task_id: &'static str = Box::leak(noise.into_boxed_str());
    let slots = Box::leak(
        vec![CatalogSlot {
            task_id,
            msg_type: "u32",
            payload: Some(ValueDecodeRef::of::<u32>()),
        }]
        .into_boxed_slice(),
    );
    let missions = Box::leak(vec![CatalogMission { slots }].into_boxed_slice());
    let description = CatalogDescription {
        layout: ValueDecodeCatalogLayout::Compact,
        missions,
    };
    let UnifiedLogger::Write(mut logger) = UnifiedLoggerBuilder::new()
        .file_base_name(&path)
        .write(true)
        .create(true)
        .preallocated_size(16 * 1024)
        .build()
        .unwrap()
    else {
        panic!("writer")
    };
    logger
        .seal_metadata(&metadata(), Some(&StartupCatalog(&description)))
        .unwrap();
    drop(logger);
    let catalog = cu29_export::catalog::read_value_decode_catalog(&path, None).unwrap();
    assert_eq!(catalog.missions[0].slots[0].task_id, task_id);
    let output = run(&path, &["fsck", "--deep"]);
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let mut reader = UnifiedLoggerRead::new(&path).unwrap();
    let bytes = reader
        .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
        .unwrap()
        .unwrap();
    assert!(bytes.len() > 16 * 1024);
    assert!(
        reader
            .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
            .unwrap()
            .is_none()
    );
    let output = String::from_utf8(output.stdout).unwrap();
    let compressed = bytes.len().to_formatted_string(&Locale::en);
    let decompressed = bincode::encode_to_vec(&catalog, bincode::config::standard())
        .unwrap()
        .len()
        .to_formatted_string(&Locale::en);
    assert!(output.contains(&format!("Catalog compressed    -> {compressed} bytes")));
    assert!(output.contains(&format!("Catalog decompressed  -> {decompressed} bytes")));
    assert!(ValueDecodeCatalog::from_blob(&bytes[..bytes.len() - 1]).is_err());
    assert!(ValueDecodeCatalog::from_blob(&[bytes.as_slice(), bytes.as_slice()].concat()).is_err());
}
