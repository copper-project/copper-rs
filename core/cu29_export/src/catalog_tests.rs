//! Catalog fixtures and integration tests for the offline reader and CLI.

use crate::catalog::copperlist_values_reader;
use bincode::Encode;
use bincode::enc::write::Writer;
use cu29::prelude::*;
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
use bincode::value_decode::ValueDecodeRef;
use cu29_value::catalog_stream::{CatalogDescription, CatalogMission, CatalogSlot, write_catalog};

pub(crate) struct StartupCatalog<'a>(pub &'a CatalogDescription);
impl Encode for StartupCatalog<'_> {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        write_catalog(encoder.writer(), self.0)
    }
}
static DESCRIPTION: CatalogDescription = CatalogDescription {
    layout: ValueDecodeCatalogLayout::Compact,
    missions: &[
        CatalogMission {
            slots: &[CatalogSlot {
                task_id: "drive",
                msg_type: "u32",
                payload: Some(ValueDecodeRef::of::<u32>()),
            }],
        },
        CatalogMission {
            slots: &[CatalogSlot {
                task_id: "park",
                msg_type: "u32",
                payload: Some(ValueDecodeRef::of::<u32>()),
            }],
        },
    ],
};
fn metadata() -> ApplicationMetadata {
    ApplicationMetadata {
        app_type: "App".into(),
        app_name: "catalog-test".into(),
        app_version: "1".into(),
        git_commit: None,
        git_dirty: None,
        subsystem_id: None,
        subsystem_code: 0,
        effective_config_ron: "()".into(),
        missions: vec!["drive".into(), "park".into()],
        catalog_offset: 0,
    }
}
fn write_run(
    logger: &Arc<Mutex<UnifiedLoggerWrite>>,
    value: u32,
    mission: &str,
    corrupt: bool,
    with_catalog: bool,
) {
    let context = {
        let mut logger = logger.lock().unwrap();
        if with_catalog {
            logger
                .seal_metadata(&metadata(), Some(&StartupCatalog(&DESCRIPTION)))
                .unwrap();
        } else {
            logger.seal_metadata::<()>(&metadata(), None).unwrap();
        }
        logger
            .construction_context(0, u32::from(mission == "park"))
            .unwrap()
    };
    let _text = LogStream::with_context(
        UnifiedLogType::StructuredLogLine,
        logger.clone(),
        1024,
        context,
    )
    .unwrap();
    let mut cls =
        LogStream::with_context(UnifiedLogType::CopperList, logger.clone(), 1024, context).unwrap();
    let mut lifecycle = LogStream::with_context(
        UnifiedLogType::RuntimeLifecycle,
        logger.clone(),
        1024,
        context,
    )
    .unwrap();
    lifecycle
        .log(&RuntimeLifecycleRecord {
            timestamp: CuTime(0),
            event: RuntimeLifecycleEvent::Instantiated {
                config_source: RuntimeLifecycleConfigSource::BundledDefault,
            },
        })
        .unwrap();
    lifecycle
        .log(&RuntimeLifecycleRecord {
            timestamp: CuTime(1),
            event: RuntimeLifecycleEvent::MissionStarted,
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
    drop(cls);
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
        "section byte",
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
    let path = dir.path().join("missing-catalog.copper");
    fixture(&path, false, false);
    assert!(
        crate::catalog::read_value_decode_catalog(&path)
            .unwrap_err()
            .to_string()
            .contains("require a catalog")
    );
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
    let mut logger = logger;
    let blob = vec![0; 16 * 1024 * 1024 * 9 / 8 + 9];
    logger
        .seal_metadata(&metadata(), Some(&WireBytes(blob)))
        .unwrap();
    drop(logger);
    let error = crate::catalog::read_value_decode_catalog(&path)
        .unwrap_err()
        .to_string();
    assert!(error.contains("exceeds offline size limit"), "{error}");
}

#[test]
fn appended_runs_reuse_one_catalog_with_both_mission_maps() {
    let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
    let path = dir.path().join("shared.copper");
    for (index, (mission, value)) in [("drive", 300), ("park", 42)].into_iter().enumerate() {
        let logger = writer(&path, index != 0);
        write_run(&logger, value, mission, false, true);
        drop(logger);
    }
    let catalog = crate::catalog::read_value_decode_catalog(&path).unwrap();
    assert_eq!(catalog.missions.len(), 2);
    for (index, (task, value)) in [("drive", 300), ("park", 42)].into_iter().enumerate() {
        assert_eq!(catalog.missions[index].slots[0].task_id, task);
        let entries = copperlist_values_reader(&path, Some(index))
            .unwrap()
            .collect::<CuResult<Vec<_>>>()
            .unwrap();
        assert_eq!(entries.len(), 2);
        assert_eq!(entries[0].msgs[0].payload, Some(Value::U32(value)));
    }
    let mut reader = UnifiedLoggerRead::new(&path).unwrap();
    assert!(
        reader
            .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
            .unwrap()
            .is_some()
    );
    assert!(
        reader
            .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
            .unwrap()
            .is_none()
    );
}

#[test]
fn cropped_startup_uses_retained_mission_identity_and_shared_catalog() {
    let dir = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
    let path = dir.path().join("cropped.copper");
    let UnifiedLogger::Write(mut logger) = UnifiedLoggerBuilder::new()
        .file_base_name(&path)
        .preallocated_size(16 * 1024)
        .capacity(8192, CapacityPolicy::OverwriteOldest)
        .write(true)
        .create(true)
        .build()
        .unwrap()
    else {
        panic!("writer")
    };
    logger
        .seal_metadata(&metadata(), Some(&StartupCatalog(&DESCRIPTION)))
        .unwrap();
    let context = logger.construction_context(7, 1).unwrap();
    let logger = Arc::new(Mutex::new(logger));
    {
        let mut lifecycle = LogStream::with_context(
            UnifiedLogType::RuntimeLifecycle,
            logger.clone(),
            1024,
            context,
        )
        .unwrap();
        lifecycle
            .log(&RuntimeLifecycleRecord {
                timestamp: CuTime(0),
                event: RuntimeLifecycleEvent::Instantiated {
                    config_source: RuntimeLifecycleConfigSource::BundledDefault,
                },
            })
            .unwrap();
    }
    for id in 0..50u8 {
        let mut stream =
            LogStream::with_context(UnifiedLogType::CopperList, logger.clone(), 1024, context)
                .unwrap();
        stream
            .log(&WireBytes(vec![id, 0, 0, 0, 0, 0, 0, 1, 1, 0, 42]))
            .unwrap();
    }
    drop(logger);
    let runs = crate::runs::discover(&path).unwrap();
    assert_eq!(runs.len(), 1);
    assert!(runs[0].started_at.is_none());
    assert_eq!(runs[0].mission_index, 1);
    let entries = copperlist_values_reader(&path, None)
        .unwrap()
        .collect::<CuResult<Vec<_>>>()
        .unwrap();
    assert!(!entries.is_empty() && entries.len() < 50);
    assert_eq!(entries.last().unwrap().id, 49);
    assert!(
        entries.iter().all(|entry| entry.msgs[0].task_id == "park"
            && entry.msgs[0].payload == Some(Value::U32(42)))
    );
}
