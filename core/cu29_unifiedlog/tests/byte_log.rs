#![cfg(feature = "std")]
use bincode::{config::standard, decode_from_slice, encode_into_slice};
use cu29_traits::{UnifiedLogType, WriteStream};
use cu29_unifiedlog::*;
use std::sync::{Arc, Mutex};
use tempfile::TempDir;

fn test_dir() -> TempDir {
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/unifiedlog-tests");
    std::fs::create_dir_all(&path).unwrap();
    TempDir::new_in(path).unwrap()
}
fn metadata() -> ApplicationMetadata {
    ApplicationMetadata {
        app_type: "Robot".into(),
        app_name: "test".into(),
        app_version: "1".into(),
        git_commit: None,
        git_dirty: None,
        subsystem_id: Some("robot".into()),
        subsystem_code: 7,
        effective_config_ron: "(tasks:[])".into(),
        missions: vec!["alpha".into(), "beta".into()],
        catalog_offset: 0,
    }
}
fn writer(
    path: &std::path::Path,
    slab: usize,
    capacity: usize,
    append: bool,
    policy: CapacityPolicy,
) -> UnifiedLoggerWrite {
    match UnifiedLoggerBuilder::new()
        .file_base_name(path)
        .preallocated_size(slab)
        .capacity(capacity as u64, policy)
        .write(true)
        .create(true)
        .append(append)
        .build()
        .unwrap()
    {
        UnifiedLogger::Write(logger) => logger,
        _ => unreachable!(),
    }
}
fn values(path: &std::path::Path) -> Vec<(SectionContext, u32)> {
    let mut reader = UnifiedLoggerRead::new(path).unwrap();
    let mut values = Vec::new();
    loop {
        let (header, data) = reader.raw_read_section().unwrap();
        if header.entry_type == UnifiedLogType::LastEntry {
            break;
        }
        if header.entry_type == UnifiedLogType::CopperList {
            let mut data = data.as_slice();
            while !data.is_empty() {
                let (value, used) = decode_from_slice::<u32, _>(data, standard()).unwrap();
                values.push((header.context, value));
                data = &data[used..];
            }
        }
    }
    values
}
#[test]
fn maximum_header_fits_existing_reservation() {
    let header = SectionHeader {
        allocated: u64::MAX,
        next_section: u64::MAX,
        used: u32::MAX,
        context: SectionContext {
            run_id: u64::MAX,
            instance_id: u32::MAX,
            mission_index: u32::MAX,
        },
        ..SectionHeader::default()
    };
    let mut bytes = [0; 512];
    assert!(encode_into_slice(&header, &mut bytes, standard()).unwrap() < 512);
}
#[test]
fn sections_cross_backing_files_with_direct_native_encoding() {
    let dir = test_dir();
    let path = dir.path().join("cross.copper");
    let mut logger = writer(&path, 1024, 8192, false, CapacityPolicy::Grow);
    logger
        .seal_metadata(&metadata(), Some(&[9u8; 1800]))
        .unwrap();
    let context = logger.construction_context(3, 1).unwrap();
    let logger = Arc::new(Mutex::new(logger));
    let mut stream =
        LogStream::with_context(UnifiedLogType::CopperList, logger.clone(), 4096, context).unwrap();
    for value in 0..1300u32 {
        stream.log(&value).unwrap();
    }
    drop(stream);
    drop(logger);
    assert_eq!(
        values(&path),
        (0..1300).map(|value| (context, value)).collect::<Vec<_>>()
    );
    let mut reader = UnifiedLoggerRead::new(&path).unwrap();
    assert_eq!(
        reader.application_metadata().unwrap().unwrap().missions,
        ["alpha", "beta"]
    );
    assert!(
        reader
            .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
            .unwrap()
            .is_some()
    );
}
#[test]
fn repeated_wrap_rebinds_sparse_streams_and_preserves_metadata() {
    let dir = test_dir();
    let path = dir.path().join("ring.copper");
    let mut logger = writer(&path, 2048, 8192, false, CapacityPolicy::OverwriteOldest);
    logger
        .seal_metadata(&metadata(), Some(&[1u8; 100]))
        .unwrap();
    let context = logger.construction_context(5, 0).unwrap();
    let logger = Arc::new(Mutex::new(logger));
    let mut sparse = LogStream::with_context(
        UnifiedLogType::StructuredLogLine,
        logger.clone(),
        1024,
        context,
    )
    .unwrap();
    sparse.log(&100u32).unwrap();
    for value in 0..80u32 {
        let mut stream =
            LogStream::with_context(UnifiedLogType::CopperList, logger.clone(), 1024, context)
                .unwrap();
        stream.log(&value).unwrap();
        if value.is_multiple_of(13) {
            sparse.log(&value).unwrap();
        }
    }
    sparse.log(&999u32).unwrap();
    drop(sparse);
    let reserved = logger.lock().unwrap().status().total_used_space;
    drop(logger);
    let retained = values(&path);
    assert!(retained.len() < 80 && !retained.is_empty());
    assert_eq!(retained.last().unwrap().1, 79);
    assert!(retained.windows(2).all(|pair| pair[0].1 < pair[1].1));
    assert!(retained.iter().all(|(ctx, _)| *ctx == context));
    let mut reader = UnifiedLoggerRead::new(&path).unwrap();
    let saved = reader.application_metadata().unwrap().unwrap();
    assert_eq!(saved.effective_config_ron, metadata().effective_config_ron);
    assert!(saved.catalog_offset != 0);
    let bytes = reader
        .read_next_section_type(UnifiedLogType::ValueDecodeCatalog)
        .unwrap()
        .unwrap();
    assert_eq!(
        decode_from_slice::<[u8; 100], _>(&bytes, standard())
            .unwrap()
            .0,
        [1; 100]
    );
    let mut reader = UnifiedLoggerRead::new(&path).unwrap();
    let mut allocated = reader.raw_main_header().page_size as usize;
    loop {
        let section = reader.raw_skip_section().unwrap();
        if section.entry_type == UnifiedLogType::LastEntry {
            break;
        }
        allocated += section.allocated as usize;
    }
    assert_eq!(reserved, allocated);
    assert!(reserved <= 8192);
    let reopened = writer(&path, 2048, 8192, true, CapacityPolicy::OverwriteOldest);
    assert_eq!(reopened.status().total_used_space, reserved);
}
#[test]
fn matching_append_changes_policy_and_allocates_new_run_ids() {
    let dir = test_dir();
    let path = dir.path().join("append.copper");
    let mut contexts = Vec::new();
    for (append, policy) in [
        (false, CapacityPolicy::OverwriteOldest),
        (true, CapacityPolicy::Grow),
        (true, CapacityPolicy::OverwriteOldest),
    ] {
        let mut logger = writer(&path, 2048, 8192, append, policy);
        logger
            .seal_metadata(&metadata(), Some(&[1u8; 100]))
            .unwrap();
        let context = logger.construction_context(5, 1).unwrap();
        contexts.push(context);
        let mut handle = logger
            .add_section_with_context(UnifiedLogType::CopperList, 1024, context)
            .unwrap();
        handle.append(42u32).unwrap();
        logger.flush_section(&mut handle);
    }
    assert_eq!(
        contexts.iter().map(|ctx| ctx.run_id).collect::<Vec<_>>(),
        [1, 2, 3]
    );
    assert_eq!(values(&path).len(), 3);
}
#[test]
fn mismatched_append_makes_no_header_or_payload_changes() {
    let dir = test_dir();
    let path = dir.path().join("mismatch.copper");
    {
        let mut logger = writer(&path, 8192, 8192, false, CapacityPolicy::Grow);
        logger.seal_metadata(&metadata(), Some(&42u32)).unwrap();
    }
    let before = std::fs::read(&path).unwrap();
    for catalog in [None, Some(43u32)] {
        let mut logger = writer(&path, 8192, 8192, true, CapacityPolicy::Grow);
        assert!(logger.seal_metadata(&metadata(), catalog.as_ref()).is_err());
        drop(logger);
        assert_eq!(std::fs::read(&path).unwrap(), before);
    }
    let mut changed = metadata();
    changed.app_version = "different".into();
    let mut logger = writer(&path, 8192, 8192, true, CapacityPolicy::Grow);
    assert!(logger.seal_metadata(&changed, Some(&42u32)).is_err());
    drop(logger);
    assert_eq!(std::fs::read(&path).unwrap(), before);
}
#[test]
fn oversized_entry_rolls_back_committed_bytes_and_cursor() {
    let dir = test_dir();
    let path = dir.path().join("rollback.copper");
    let logger = Arc::new(Mutex::new(writer(
        &path,
        8192,
        8192,
        false,
        CapacityPolicy::Grow,
    )));
    let mut stream = LogStream::new(UnifiedLogType::CopperList, logger.clone(), 1024).unwrap();
    assert!(stream.log(&[1u8; 1024]).is_err());
    stream.log(&11u32).unwrap();
    stream.log(&12u32).unwrap();
    drop(stream);
    drop(logger);
    assert_eq!(
        values(&path)
            .iter()
            .map(|(_, value)| *value)
            .collect::<Vec<_>>(),
        [11, 12]
    );
}
#[test]
fn incomplete_log_is_inspectable_but_append_is_read_only_failure() {
    let dir = test_dir();
    let path = dir.path().join("incomplete.copper");
    let mut logger = writer(&path, 8192, 8192, false, CapacityPolicy::Grow);
    let mut section = logger
        .add_section(UnifiedLogType::CopperList, 1024)
        .unwrap();
    section.append(7u32).unwrap();
    assert_eq!(values(&path).last().unwrap().1, 7);
    let before = std::fs::read(&path).unwrap();
    assert!(
        UnifiedLoggerBuilder::new()
            .file_base_name(&path)
            .preallocated_size(8192)
            .write(true)
            .create(true)
            .append(true)
            .build()
            .is_err()
    );
    assert_eq!(std::fs::read(&path).unwrap(), before);
    logger.flush_section(&mut section);
}

#[test]
fn allocation_during_an_encode_is_rejected_without_mutation() {
    struct Paused {
        entered: std::sync::mpsc::Sender<()>,
        resume: std::sync::mpsc::Receiver<()>,
    }
    impl bincode::Encode for Paused {
        fn encode<E: bincode::enc::Encoder>(
            &self,
            encoder: &mut E,
        ) -> Result<(), bincode::error::EncodeError> {
            self.entered.send(()).unwrap();
            self.resume.recv().unwrap();
            42u32.encode(encoder)
        }
    }
    let dir = test_dir();
    let path = dir.path().join("busy.copper");
    let mut logger = writer(&path, 4096, 4096, false, CapacityPolicy::OverwriteOldest);
    logger.seal_metadata::<()>(&metadata(), None).unwrap();
    let mut handle = logger
        .add_section(UnifiedLogType::CopperList, 2048)
        .unwrap();
    let before = std::fs::read(&path).unwrap();
    let (entered, entering) = std::sync::mpsc::channel();
    let (resume, resuming) = std::sync::mpsc::channel();
    let worker = std::thread::spawn(move || {
        handle
            .append(Paused {
                entered,
                resume: resuming,
            })
            .unwrap();
        handle
    });
    entering.recv().unwrap();
    assert!(
        logger
            .add_section(UnifiedLogType::CopperList, 2048)
            .is_err()
    );
    assert_eq!(std::fs::read(&path).unwrap(), before);
    resume.send(()).unwrap();
    let mut handle = worker.join().unwrap();
    logger.try_flush_section(&mut handle).unwrap();
    drop(logger);
    assert_eq!(values(&path)[0].1, 42);
}

#[test]
fn invalid_links_are_rejected_before_append_mutates_the_log() {
    let dir = test_dir();
    for corrupt in [0, 1, 2] {
        let path = dir.path().join(format!("invalid{corrupt}.copper"));
        {
            let mut logger = writer(&path, 4096, 4096, false, CapacityPolicy::Grow);
            logger.seal_metadata::<()>(&metadata(), None).unwrap();
            let mut handle = logger
                .add_section(UnifiedLogType::CopperList, 1024)
                .unwrap();
            handle.append(42u32).unwrap();
            logger.try_flush_section(&mut handle).unwrap();
        }
        let reader = UnifiedLoggerRead::new(&path).unwrap();
        let offset = reader.raw_main_header().head_section;
        drop(reader);
        let mut bytes = std::fs::read(&path).unwrap();
        let (mut header, _) =
            decode_from_slice::<SectionHeader, _>(&bytes[offset as usize..], standard()).unwrap();
        match corrupt {
            0 => header.next_section = offset,
            1 => header.next_section = u64::MAX,
            _ => header.used = u32::MAX,
        }
        encode_into_slice(
            &header,
            &mut bytes[offset as usize..offset as usize + 512],
            standard(),
        )
        .unwrap();
        std::fs::write(&path, &bytes).unwrap();
        assert!(
            UnifiedLoggerBuilder::new()
                .file_base_name(&path)
                .preallocated_size(4096)
                .write(true)
                .create(true)
                .append(true)
                .build()
                .is_err()
        );
        assert_eq!(std::fs::read(&path).unwrap(), bytes);
    }
}
