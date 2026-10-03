//! Catalog fixtures and integration tests for the offline reader and CLI.

use crate::catalog::copperlist_values_reader;
use bincode::Encode;
use bincode::enc::write::Writer;
use cu29::prelude::*;
use std::io::Write;
use std::path::Path;
use std::sync::{Arc, Mutex};

pub(crate) struct WireBytes(pub Vec<u8>);
impl Encode for WireBytes {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        encoder.writer().write(&self.0)
    }
}

pub(crate) fn fixture(path: &Path, corrupt: bool, with_catalog: bool) {
    let logger = writer(path, false);
    write_run(&logger, 300, "drive", corrupt, with_catalog);
}
fn writer(path: &Path, append: bool) -> Arc<Mutex<UnifiedLoggerWrite>> {
    let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
        .file_base_name(path)
        .write(true)
        .create(true)
        .append(append)
        .preallocated_size(16 * 1024)
        .build()
        .unwrap()
    else {
        panic!("writer")
    };
    Arc::new(Mutex::new(logger))
}
fn write_run(
    logger: &Arc<Mutex<UnifiedLoggerWrite>>,
    value: u32,
    mission: &str,
    corrupt: bool,
    with_catalog: bool,
) {
    if with_catalog {
        let mut builder = ValueDecodeCatalogBuilder::default();
        builder.add::<u32>(mission, "u32");
        let catalog = builder
            .finish("()", mission, ValueDecodeCatalogLayout::Compact)
            .unwrap();
        write_value_decode_catalog(logger.clone(), &catalog_blob(&catalog)).unwrap();
    }
    let _text =
        stream_write::<CuLogEntry, _>(logger.clone(), UnifiedLogType::StructuredLogLine, 1024)
            .unwrap();
    let mut cls =
        stream_write::<WireBytes, _>(logger.clone(), UnifiedLogType::CopperList, 1024).unwrap();
    let mut lifecycle = stream_write::<RuntimeLifecycleRecord, _>(
        logger.clone(),
        UnifiedLogType::RuntimeLifecycle,
        1024,
    )
    .unwrap();
    lifecycle
        .log(&RuntimeLifecycleRecord {
            timestamp: CuTime(0),
            event: RuntimeLifecycleEvent::Instantiated {
                config_source: RuntimeLifecycleConfigSource::BundledDefault,
                effective_config_ron: "()".into(),
                stack: RuntimeLifecycleStackInfo {
                    app_name: "catalog-test".into(),
                    app_version: "1".into(),
                    git_commit: None,
                    git_dirty: None,
                    subsystem_id: None,
                    subsystem_code: 0,
                    instance_id: 0,
                },
            },
        })
        .unwrap();
    lifecycle
        .log(&RuntimeLifecycleRecord {
            timestamp: CuTime(1),
            event: RuntimeLifecycleEvent::MissionStarted {
                mission: mission.into(),
            },
        })
        .unwrap();
    for id in [0u8, 1] {
        let mut bytes = vec![id, 0, 0, 0, 0, 0, 0, 1, 1, 0];
        bytes.extend(bincode::encode_to_vec(value, bincode::config::standard()).unwrap());
        if corrupt && id == 1 {
            bytes.pop();
        }
        cls.log(&WireBytes(bytes)).unwrap();
    }
    lifecycle
        .log(&RuntimeLifecycleRecord {
            timestamp: CuTime(2),
            event: RuntimeLifecycleEvent::ShutdownCompleted,
        })
        .unwrap();
}

#[test]
fn test_run_scoped_catalog_selection_and_append() {
    let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
    let path = dir.path().join("appended.copper");
    fixture(&path, false, true);
    let logger = writer(&path, true);
    write_run(&logger, 42, "park", false, true);
    drop(logger);
    assert!(copperlist_values_reader(&path, None).is_err());
    for (run, expected, task) in [(0, 300, "drive"), (1, 42, "park")] {
        let entries = copperlist_values_reader(&path, Some(run))
            .unwrap()
            .collect::<CuResult<Vec<_>>>()
            .unwrap();
        assert_eq!(entries.len(), 2);
        assert_eq!(entries[0].id, 0);
        assert_eq!(entries[0].msgs[0].task_id, task);
        assert_eq!(entries[0].msgs[0].payload, Some(Value::U32(expected)));
    }
}

#[test]
fn test_corrupt_record_returns_context_and_ends_iterator() {
    let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
    let path = dir.path().join("corrupt.copper");
    fixture(&path, true, true);
    let mut reader = copperlist_values_reader(&path, None).unwrap();
    assert!(reader.next().unwrap().is_ok());
    let error = reader.next().unwrap().unwrap_err().to_string();
    for context in [
        "Run 0",
        "slab",
        "record byte",
        "CopperList #1",
        "slot 0 (drive)",
    ] {
        assert!(error.contains(context), "{error}");
    }
    assert!(reader.next().is_none());
}

#[test]
fn test_missing_catalog_is_explicit() {
    let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
    let path = dir.path().join("legacy.copper");
    fixture(&path, false, false);
    assert!(
        crate::catalog::read_value_decode_catalog(&path, None)
            .unwrap_err()
            .to_string()
            .contains("require a catalog")
    );
}

fn catalog_blob(catalog: &ValueDecodeCatalog) -> Vec<u8> {
    let raw = bincode::encode_to_vec(catalog, bincode::config::standard()).unwrap();
    let mut blob = b"CUVDCAT\0\x01\x00".to_vec();
    {
        let mut compressed = brotli::CompressorWriter::new(&mut blob, 4096, 11, 24);
        compressed.write_all(&raw).unwrap();
    }
    blob
}

#[test]
fn test_oversized_catalog_is_rejected_before_loading_its_body() {
    let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
    let path = dir.path().join("oversized.copper");
    let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
        .file_base_name(&path)
        .write(true)
        .create(true)
        .preallocated_size(32 * 1024 * 1024)
        .build()
        .unwrap()
    else {
        panic!("writer")
    };
    let logger = Arc::new(Mutex::new(logger));
    let blob = vec![0; 16 * 1024 * 1024 + 11];
    write_value_decode_catalog(logger.clone(), &blob).unwrap();
    drop(logger);
    let error = crate::catalog::read_value_decode_catalog(&path, None)
        .unwrap_err()
        .to_string();
    assert!(error.contains("catalog discovery"), "{error}");
    assert!(error.contains("exceeds 16 MiB"), "{error}");
}
