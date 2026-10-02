//! Recorded run discovery and bounded access to a selected run's sections.

use crate::build_read_logger;
use bincode::config::standard;
use bincode::decode_from_slice;
use cu29::prelude::CuError;
use cu29::prelude::CuResult;
use cu29::prelude::CuTime;
use cu29::prelude::MainHeader;
use cu29::prelude::RuntimeLifecycleEvent;
use cu29::prelude::RuntimeLifecycleRecord;
use cu29::prelude::RuntimeLifecycleStackInfo;
use cu29::prelude::SectionHeader;
use cu29::prelude::UnifiedLogRead;
use cu29::prelude::UnifiedLogType;
use cu29::prelude::UnifiedLoggerRead;
use cu29::prelude::memmap::LogPosition;
use std::io::{self, Read};
use std::path::Path;

#[derive(Debug)]
pub(crate) struct RecordedRun {
    pub index: usize,
    pub run_id: u64,
    pub started_at: Option<CuTime>,
    pub stack: Option<RuntimeLifecycleStackInfo>,
    pub config: Option<String>,
    pub missions: Vec<String>,
    pub shutdown_completed: bool,
    start: LogPosition,
    pub copperlist_bytes: u64,
}

impl RecordedRun {
    fn new(index: usize, start: LogPosition) -> Self {
        Self {
            index,
            run_id: 0,
            started_at: None,
            stack: None,
            config: None,
            missions: Vec::new(),
            shutdown_completed: false,
            start,
            copperlist_bytes: 0,
        }
    }

    pub fn reader(&self, path: &Path) -> CuResult<RunReader> {
        let mut inner = build_read_logger(path)?;
        inner.seek(self.start)?;
        Ok(RunReader {
            inner,
            run_id: self.run_id,
        })
    }
}

/// Discover constructions from section identities, including runs whose startup rolled out.
pub(crate) fn discover(path: &Path) -> CuResult<Vec<RecordedRun>> {
    let mut reader = build_read_logger(path)?;
    let metadata = reader.application_metadata()?;
    let beginning = reader.position();
    let mut runs = Vec::<RecordedRun>::new();
    loop {
        let position = reader.position();
        let (header, content) = reader.raw_read_section()?;
        if header.entry_type == UnifiedLogType::LastEntry {
            break;
        }
        if matches!(
            header.entry_type,
            UnifiedLogType::ApplicationMetadata | UnifiedLogType::ValueDecodeCatalog
        ) {
            continue;
        }
        let run_id = header.context.run_id;
        let index = if let Some(index) = runs.iter().position(|run| run.run_id == run_id) {
            index
        } else {
            let mut run = RecordedRun::new(runs.len(), beginning);
            run.run_id = run_id;
            if let Some(metadata) = &metadata {
                run.config = Some(metadata.effective_config_ron.clone());
                run.stack = Some(RuntimeLifecycleStackInfo {
                    app_name: metadata.app_name.clone(),
                    app_version: metadata.app_version.clone(),
                    git_commit: metadata.git_commit.clone(),
                    git_dirty: metadata.git_dirty,
                    subsystem_id: metadata.subsystem_id.clone(),
                    subsystem_code: metadata.subsystem_code,
                    instance_id: header.context.instance_id,
                });
            }
            runs.push(run);
            runs.len() - 1
        };
        let run = &mut runs[index];
        if let Some(metadata) = &metadata {
            let mission = metadata
                .missions
                .get(header.context.mission_index as usize)
                .ok_or(CuError::from(
                    "Section mission index is outside static metadata",
                ))?;
            if !run.missions.contains(mission) {
                run.missions.push(mission.clone());
            }
            if run
                .stack
                .as_ref()
                .is_some_and(|stack| stack.instance_id != header.context.instance_id)
            {
                return Err(CuError::from(
                    "Run contains inconsistent instance identities",
                ));
            }
        }
        if header.entry_type == UnifiedLogType::CopperList {
            run.copperlist_bytes += header.used as u64;
        }
        if header.entry_type == UnifiedLogType::RuntimeLifecycle {
            let mut remaining = content.as_slice();
            while !remaining.is_empty() {
                let (record, used) =
                    decode_from_slice::<RuntimeLifecycleRecord, _>(remaining, standard()).map_err(
                        |e| CuError::new_with_cause("Invalid runtime lifecycle record", e),
                    )?;
                match record.event {
                    RuntimeLifecycleEvent::Instantiated { .. } => {
                        run.started_at = Some(record.timestamp);
                    }
                    RuntimeLifecycleEvent::ShutdownCompleted => {
                        run.shutdown_completed = true;
                    }
                    _ => {}
                }
                remaining = &remaining[used..];
            }
        }
        if reader.position() == position {
            return Err(CuError::from("Section traversal did not advance"));
        }
    }
    if runs.is_empty() {
        runs.push(RecordedRun::new(0, beginning));
    }
    Ok(runs)
}

pub(crate) fn select(runs: &[RecordedRun], requested: Option<usize>) -> CuResult<&RecordedRun> {
    let index = match requested {
        Some(index) => index,
        None if runs.len() == 1 => 0,
        None => {
            return Err(CuError::from(format!(
                "Log contains {} recorded runs; use list-runs and select --run <index>",
                runs.len()
            )));
        }
    };
    let run = runs.get(index).ok_or_else(|| {
        CuError::from(format!(
            "Recorded run {index} does not exist; use list-runs to see available runs"
        ))
    })?;
    Ok(run)
}

pub(crate) struct RunReader {
    inner: UnifiedLoggerRead,
    run_id: u64,
}

impl RunReader {
    #[cfg(feature = "self-describing-logs")]
    pub(crate) fn position(&self) -> LogPosition {
        self.inner.position()
    }
    pub fn raw_main_header(&self) -> &MainHeader {
        self.inner.raw_main_header()
    }

    pub fn raw_read_section(&mut self) -> CuResult<(SectionHeader, Vec<u8>)> {
        loop {
            let (header, content) = self.inner.raw_read_section()?;
            if header.entry_type == UnifiedLogType::LastEntry
                || header.context.run_id == self.run_id
                || matches!(
                    header.entry_type,
                    UnifiedLogType::ApplicationMetadata | UnifiedLogType::ValueDecodeCatalog
                )
            {
                return Ok((header, content));
            }
        }
    }

    fn read_next_section_type(&mut self, kind: UnifiedLogType) -> CuResult<Option<Vec<u8>>> {
        loop {
            let (header, content) = self.raw_read_section()?;
            if header.entry_type == UnifiedLogType::LastEntry {
                return Ok(None);
            }
            if header.entry_type == kind {
                return Ok(Some(content));
            }
        }
    }

    pub fn stream(self, kind: UnifiedLogType) -> RunStream {
        RunStream {
            reader: self,
            kind,
            buffer: Vec::new(),
            offset: 0,
            finished: false,
        }
    }
}

pub(crate) struct RunStream {
    reader: RunReader,
    kind: UnifiedLogType,
    buffer: Vec<u8>,
    offset: usize,
    finished: bool,
}

impl Read for RunStream {
    fn read(&mut self, destination: &mut [u8]) -> io::Result<usize> {
        if destination.is_empty() || self.finished {
            return Ok(0);
        }
        while self.offset == self.buffer.len() {
            let content = self
                .reader
                .read_next_section_type(self.kind)
                .map_err(|e| io::Error::other(e.to_string()))?;
            let Some(content) = content else {
                self.finished = true;
                return Ok(0);
            };
            self.buffer = content;
            self.offset = 0;
        }
        let count = destination.len().min(self.buffer.len() - self.offset);
        destination[..count].copy_from_slice(&self.buffer[self.offset..self.offset + count]);
        self.offset += count;
        Ok(count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::prelude::*;
    use std::sync::{Arc, Mutex};
    fn metadata() -> ApplicationMetadata {
        ApplicationMetadata {
            app_type: "App".into(),
            app_name: "test".into(),
            app_version: "1".into(),
            git_commit: None,
            git_dirty: None,
            subsystem_id: None,
            subsystem_code: 0,
            effective_config_ron: "(tasks:[])".into(),
            missions: vec!["alpha".into(), "beta".into()],
            catalog_offset: 0,
        }
    }
    fn directory() -> tempfile::TempDir {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/export-tests");
        std::fs::create_dir_all(&path).unwrap();
        tempfile::TempDir::new_in(path).unwrap()
    }
    #[test]
    fn interleaved_and_cropped_runs_use_section_context_and_shared_metadata() {
        let dir = directory();
        let path = dir.path().join("runs.copper");
        let UnifiedLogger::Write(mut logger) = UnifiedLoggerBuilder::new()
            .file_base_name(&path)
            .preallocated_size(4096)
            .rollover(16384)
            .write(true)
            .create(true)
            .build()
            .unwrap()
        else {
            unreachable!()
        };
        logger
            .seal_metadata(&metadata(), Some(&[1u8; 100]))
            .unwrap();
        let first = logger.construction_context(7, 0).unwrap();
        let second = logger.construction_context(9, 1).unwrap();
        let logger = Arc::new(Mutex::new(logger));
        for context in [first, second] {
            let mut stream =
                stream_write_context::<RuntimeLifecycleRecord, memmap::MmapSectionStorage>(
                    logger.clone(),
                    UnifiedLogType::RuntimeLifecycle,
                    1024,
                    context,
                )
                .unwrap();
            stream
                .log(&RuntimeLifecycleRecord {
                    timestamp: CuTime::from_nanos(100),
                    event: RuntimeLifecycleEvent::Instantiated {
                        config_source: RuntimeLifecycleConfigSource::ExternalFile,
                    },
                })
                .unwrap();
        }
        for value in 0..60u32 {
            let context = if value.is_multiple_of(2) {
                first
            } else {
                second
            };
            let mut stream = stream_write_context::<u32, memmap::MmapSectionStorage>(
                logger.clone(),
                UnifiedLogType::CopperList,
                1024,
                context,
            )
            .unwrap();
            stream.log(&value).unwrap();
        }
        drop(logger);
        let runs = discover(&path).unwrap();
        assert_eq!(runs.len(), 2);
        assert!(select(&runs, None).is_err());
        for run in &runs {
            assert!(run.started_at.is_none());
            assert!(!run.shutdown_completed);
            assert_eq!(run.config.as_deref(), Some("(tasks:[])"));
            let context = if run.run_id == first.run_id {
                first
            } else {
                second
            };
            assert_eq!(run.stack.as_ref().unwrap().instance_id, context.instance_id);
            assert_eq!(
                run.missions,
                [if context.mission_index == 0 {
                    "alpha"
                } else {
                    "beta"
                }]
            );
            let mut bytes = Vec::new();
            run.reader(&path)
                .unwrap()
                .stream(UnifiedLogType::CopperList)
                .read_to_end(&mut bytes)
                .unwrap();
            assert!(
                bytes.iter().all(
                    |value| u32::from(*value).is_multiple_of(2) == (context.mission_index == 0)
                )
            );
            assert!(!bytes.is_empty());
            let mut catalog = Vec::new();
            run.reader(&path)
                .unwrap()
                .stream(UnifiedLogType::ValueDecodeCatalog)
                .read_to_end(&mut catalog)
                .unwrap();
            assert_eq!(catalog.len(), 100);
        }
    }
    #[test]
    fn standalone_writer_has_one_run_and_selection_ignores_other_run_payloads() {
        let dir = directory();
        let path = dir.path().join("standalone.copper");
        {
            let UnifiedLogger::Write(mut logger) = UnifiedLoggerBuilder::new()
                .file_base_name(&path)
                .preallocated_size(8192)
                .write(true)
                .create(true)
                .build()
                .unwrap()
            else {
                unreachable!()
            };
            let mut handle = logger
                .add_section(UnifiedLogType::CopperList, 1024)
                .unwrap();
            handle.append(42u32).unwrap();
            logger.flush_section(&mut handle);
        }
        let runs = discover(&path).unwrap();
        assert_eq!(runs.len(), 1);
        assert!(runs[0].stack.is_none());
        let mut bytes = Vec::new();
        select(&runs, None)
            .unwrap()
            .reader(&path)
            .unwrap()
            .stream(UnifiedLogType::CopperList)
            .read_to_end(&mut bytes)
            .unwrap();
        assert_eq!(bytes, [42]);
    }
}
