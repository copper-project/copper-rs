//! Runtime instance discovery and bounded access to a selected instance's sections.

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
pub(crate) struct RuntimeInstance {
    pub index: usize,
    pub started_at: Option<CuTime>,
    pub stack: Option<RuntimeLifecycleStackInfo>,
    pub config: Option<String>,
    pub missions: Vec<String>,
    pub shutdown_completed: bool,
    start: LogPosition,
    end: Option<LogPosition>,
    pub copperlist_bytes: u64,
    boundary_known: bool,
}

impl RuntimeInstance {
    fn new(index: usize, start: LogPosition) -> Self {
        Self {
            index,
            started_at: None,
            stack: None,
            config: None,
            missions: Vec::new(),
            shutdown_completed: false,
            start,
            end: None,
            copperlist_bytes: 0,
            boundary_known: true,
        }
    }

    pub fn reader(&self, path: &Path) -> CuResult<InstanceReader> {
        let mut inner = build_read_logger(path)?;
        inner.seek(self.start)?;
        Ok(InstanceReader {
            inner,
            end: self.end,
        })
    }
}

/// Inspect lifecycle records without decoding application-specific payloads.
pub(crate) fn discover(path: &Path) -> CuResult<Vec<RuntimeInstance>> {
    let mut reader = build_read_logger(path)?;
    let beginning = reader.position();
    let mut instances = Vec::<RuntimeInstance>::new();
    let mut sections = Vec::new();
    let mut startup_prefix = None;
    let mut previous_marker = 0;
    loop {
        let position = reader.position();
        let header = reader.raw_skip_section()?;
        if header.entry_type == UnifiedLogType::LastEntry {
            break;
        }
        if header.entry_type == UnifiedLogType::Empty
            || header.offset_to_next_section < u32::from(header.block_size)
            || header.used > header.offset_to_next_section - u32::from(header.block_size)
            || reader.position() == position
        {
            return Err(CuError::from(format!(
                "Invalid section at slab {} offset {}",
                position.slab_index, position.offset,
            )));
        }
        if header.entry_type == UnifiedLogType::RuntimeLifecycle {
            reader.seek(position)?;
            let (_, content) = reader.raw_read_section()?;
            let mut remaining = content.as_slice();
            while !remaining.is_empty() {
                let (record, used) =
                    decode_from_slice::<RuntimeLifecycleRecord, _>(remaining, standard()).map_err(
                        |e| CuError::new_with_cause("Invalid runtime lifecycle record", e),
                    )?;
                match record.event {
                    RuntimeLifecycleEvent::Instantiated {
                        effective_config_ron,
                        stack,
                        ..
                    } => {
                        if remaining.len() != content.len() {
                            return Err(CuError::from(
                                "Instantiated must start its runtime lifecycle section",
                            ));
                        }
                        // Generated runtimes reserve their streams before the lifecycle
                        // stream. Instantiated still identifies the new instance; include
                        // those initial reservations in its physical section range.
                        let prefix = startup_reservations(&sections[previous_marker..]);
                        let has_prefix = *startup_prefix.get_or_insert(!sections.is_empty());
                        let start = if has_prefix {
                            prefix.unwrap_or(position)
                        } else {
                            position
                        };
                        let mut instance = RuntimeInstance::new(instances.len(), start);
                        instance.boundary_known = !has_prefix || prefix.is_some();
                        previous_marker = sections.len();
                        instance.started_at = Some(record.timestamp);
                        instance.config = Some(effective_config_ron);
                        instance.stack = Some(stack);
                        instances.push(instance);
                    }
                    RuntimeLifecycleEvent::MissionStarted { mission } => {
                        if let Some(instance) = instances.last_mut()
                            && !instance.missions.contains(&mission)
                        {
                            instance.missions.push(mission);
                        }
                    }
                    RuntimeLifecycleEvent::ShutdownCompleted => {
                        if let Some(instance) = instances.last_mut() {
                            instance.shutdown_completed = true;
                        }
                    }
                    _ => {}
                }
                remaining = &remaining[used..];
            }
        }
        sections.push((position, header.entry_type, header.used));
    }

    if instances.is_empty() {
        // Logs from standalone writers can have no runtime lifecycle stream.
        instances.push(RuntimeInstance::new(0, beginning));
    } else if instances.len() == 1 {
        // Include streams reserved before Instantiated by older runtime builders.
        instances[0].start = beginning;
        instances[0].boundary_known = true;
    }

    // An unresolved boundary also makes the preceding instance's end ambiguous.
    if instances.iter().any(|instance| !instance.boundary_known) {
        for instance in &mut instances {
            instance.boundary_known = false;
        }
    }

    for index in 0..instances.len().saturating_sub(1) {
        instances[index].end = Some(instances[index + 1].start);
    }
    for instance in &mut instances {
        instance.copperlist_bytes = sections
            .iter()
            .filter(|(position, kind, _)| {
                *kind == UnifiedLogType::CopperList
                    && position_key(*position) >= position_key(instance.start)
                    && instance
                        .end
                        .is_none_or(|end| position_key(*position) < position_key(end))
            })
            .map(|(_, _, used)| u64::from(*used))
            .sum();
    }
    Ok(instances)
}

fn startup_reservations(sections: &[(LogPosition, UnifiedLogType, u32)]) -> Option<LogPosition> {
    use UnifiedLogType::{CopperList, FrozenTasks, StructuredLogLine};
    // cu29_derive::build_with_resources allocates text first for local logging,
    // and after CL/keyframes when logstream fanout is enabled. Keyframes are optional.
    for types in [
        &[StructuredLogLine, CopperList, FrozenTasks][..],
        &[CopperList, FrozenTasks, StructuredLogLine][..],
        &[StructuredLogLine, CopperList][..],
        &[CopperList, StructuredLogLine][..],
    ] {
        if sections.len() >= types.len() {
            let tail = &sections[sections.len() - types.len()..];
            if tail
                .iter()
                .zip(types)
                .all(|((_, kind, _), expected)| kind == expected)
            {
                return Some(tail[0].0);
            }
        }
    }
    None
}

fn position_key(position: LogPosition) -> (usize, usize) {
    (position.slab_index, position.offset)
}

pub(crate) fn select(
    instances: &[RuntimeInstance],
    requested: Option<usize>,
) -> CuResult<&RuntimeInstance> {
    let index = match requested {
        Some(index) => index,
        None if instances.len() == 1 => 0,
        None => {
            return Err(CuError::from(format!(
                "Log contains {} runtime instances; use list-instances and select --instance <index>",
                instances.len()
            )));
        }
    };
    let instance = instances.get(index).ok_or_else(|| {
        CuError::from(format!(
            "Runtime instance {index} does not exist; use list-instances to see available instances"
        ))
    })?;
    if !instance.boundary_known {
        return Err(CuError::from(
            "Cannot isolate runtime instances: startup sections do not match the recorded runtime layout",
        ));
    }
    Ok(instance)
}

pub(crate) struct InstanceReader {
    inner: UnifiedLoggerRead,
    end: Option<LogPosition>,
}

impl InstanceReader {
    pub fn raw_main_header(&self) -> &MainHeader {
        self.inner.raw_main_header()
    }

    fn at_end(&self) -> bool {
        self.end
            .is_some_and(|end| position_key(self.inner.position()) >= position_key(end))
    }

    pub fn raw_read_section(&mut self) -> CuResult<(SectionHeader, Vec<u8>)> {
        if self.at_end() {
            return Ok((
                SectionHeader {
                    entry_type: UnifiedLogType::LastEntry,
                    is_open: false,
                    ..SectionHeader::default()
                },
                Vec::new(),
            ));
        }
        self.inner.raw_read_section()
    }

    fn read_next_section_type(&mut self, kind: UnifiedLogType) -> CuResult<Option<Vec<u8>>> {
        while !self.at_end() {
            let position = self.inner.position();
            let header = self.inner.raw_skip_section()?;
            if header.entry_type == UnifiedLogType::LastEntry {
                return Ok(None);
            }
            if header.entry_type == kind {
                self.inner.seek(position)?;
                return self
                    .inner
                    .raw_read_section()
                    .map(|(_, content)| Some(content));
            }
        }
        Ok(None)
    }

    pub fn stream(self, kind: UnifiedLogType) -> InstanceStream {
        InstanceStream {
            reader: self,
            kind,
            buffer: Vec::new(),
            offset: 0,
            finished: false,
        }
    }
}

pub(crate) struct InstanceStream {
    reader: InstanceReader,
    kind: UnifiedLogType,
    buffer: Vec<u8>,
    offset: usize,
    finished: bool,
}

impl Read for InstanceStream {
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
    use crate::copperlists_reader;
    use bincode::{Decode, Encode};
    use cu29::prelude::memmap::MmapSectionStorage;
    use cu29::prelude::*;
    use std::sync::{Arc, Mutex};
    use tempfile::TempDir;

    #[derive(Debug, Default, Encode, Decode, serde::Serialize)]
    struct Messages(u32);

    impl ErasedCuStampedDataSet for Messages {
        fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
            Vec::new()
        }
    }

    impl MatchingTasks for Messages {
        fn get_all_task_ids() -> &'static [&'static str] {
            &[]
        }
    }

    impl CuPayloadRawBytes for Messages {
        fn payload_raw_bytes(&self) -> Vec<Option<u64>> {
            Vec::new()
        }
    }

    fn writer(path: &Path, append: bool) -> Arc<Mutex<UnifiedLoggerWrite>> {
        let UnifiedLogger::Write(writer) = UnifiedLoggerBuilder::new()
            .file_base_name(path)
            .write(true)
            .create(true)
            .append(append)
            .preallocated_size(64 * 1024)
            .build()
            .unwrap()
        else {
            panic!("Expected a writer");
        };
        Arc::new(Mutex::new(writer))
    }

    fn write_instance(logger: &Arc<Mutex<UnifiedLoggerWrite>>, value: u32, mission: &str) {
        write_instance_with_options(logger, value, mission, false, true, 3);
    }

    fn write_instance_with_options(
        logger: &Arc<Mutex<UnifiedLoggerWrite>>,
        value: u32,
        mission: &str,
        logstream: bool,
        capture_keyframes: bool,
        entries: u64,
    ) {
        let make_text = || {
            stream_write::<CuLogEntry, MmapSectionStorage>(
                logger.clone(),
                UnifiedLogType::StructuredLogLine,
                1024,
            )
            .unwrap()
        };
        let early_text = (!logstream).then(make_text);
        let mut cls = stream_write::<CopperList<Messages>, MmapSectionStorage>(
            logger.clone(),
            UnifiedLogType::CopperList,
            1024,
        )
        .unwrap();
        let mut keyframes = capture_keyframes.then(|| {
            stream_write::<KeyFrame, MmapSectionStorage>(
                logger.clone(),
                UnifiedLogType::FrozenTasks,
                1024,
            )
            .unwrap()
        });
        let mut text = early_text.unwrap_or_else(make_text);
        let mut lifecycle = stream_write::<RuntimeLifecycleRecord, MmapSectionStorage>(
            logger.clone(),
            UnifiedLogType::RuntimeLifecycle,
            1024,
        )
        .unwrap();
        lifecycle
            .log(&RuntimeLifecycleRecord {
                timestamp: CuTime::from_nanos(0),
                event: RuntimeLifecycleEvent::Instantiated {
                    config_source: RuntimeLifecycleConfigSource::BundledDefault,
                    effective_config_ron: format!("(missions: [(id: \"drive\"), (id: \"park\")], tasks: [], cnx: [], // {value}\n)"),
                    stack: RuntimeLifecycleStackInfo {
                        app_name: "instance-test".to_string(),
                        app_version: "1".to_string(),
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
                timestamp: CuTime::from_nanos(1),
                event: RuntimeLifecycleEvent::MissionStarted {
                    mission: mission.to_string(),
                },
            })
            .unwrap();
        for id in 0..entries {
            cls.log(&CopperList::new(id, Messages(value))).unwrap();
        }
        text.log(&CuLogEntry::new(value, CuLogLevel::Info)).unwrap();
        if let Some(keyframes) = &mut keyframes {
            keyframes
                .log(&KeyFrame {
                    culistid: 0,
                    timestamp: CuTime::from_nanos(u64::from(value)),
                    serialized_tasks: Vec::new(),
                })
                .unwrap();
        }
        lifecycle
            .log(&RuntimeLifecycleRecord {
                timestamp: CuTime::from_nanos(10),
                event: RuntimeLifecycleEvent::ShutdownCompleted,
            })
            .unwrap();
    }

    fn assert_instances(path: &Path) {
        let catalog = discover(path).unwrap();
        assert_eq!(catalog.len(), 2);
        assert!(select(&catalog, None).is_err());
        assert!(select(&catalog, Some(2)).is_err());
        for (index, expected) in [17, 29].into_iter().enumerate() {
            let instance = select(&catalog, Some(index)).unwrap();
            assert_eq!(instance.stack.as_ref().unwrap().instance_id, 0);
            assert_eq!(
                instance.missions,
                [if index == 0 { "drive" } else { "park" }]
            );
            assert!(instance.shutdown_completed);
            let entries = copperlists_reader::<Messages>(
                instance
                    .reader(path)
                    .unwrap()
                    .stream(UnifiedLogType::CopperList),
            )
            .collect::<Vec<_>>();
            assert_eq!(
                entries.iter().map(|entry| entry.id).collect::<Vec<_>>(),
                [0, 1, 2]
            );
            assert!(entries.iter().all(|entry| entry.msgs.0 == expected));
            crate::fsck::check::<Messages>(&mut instance.reader(path).unwrap(), 0, false).unwrap();
            for kind in [
                UnifiedLogType::StructuredLogLine,
                UnifiedLogType::FrozenTasks,
            ] {
                let mut bytes = Vec::new();
                instance
                    .reader(path)
                    .unwrap()
                    .stream(kind)
                    .read_to_end(&mut bytes)
                    .unwrap();
                match kind {
                    UnifiedLogType::StructuredLogLine => {
                        let (entry, used) =
                            decode_from_slice::<CuLogEntry, _>(&bytes, standard()).unwrap();
                        assert_eq!(entry.msg_index, expected);
                        assert_eq!(used, bytes.len());
                    }
                    UnifiedLogType::FrozenTasks => {
                        let (entry, used) =
                            decode_from_slice::<KeyFrame, _>(&bytes, standard()).unwrap();
                        assert_eq!(entry.culistid, 0);
                        assert_eq!(entry.timestamp.as_nanos(), u64::from(expected));
                        assert_eq!(used, bytes.len());
                    }
                    _ => unreachable!(),
                }
            }
        }
    }

    #[test]
    fn test_mission_change_with_repeated_ids_selects_all_streams() {
        let dir = TempDir::new_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("missions.copper");
        let logger = writer(&path, false);
        write_instance(&logger, 17, "drive");
        write_instance(&logger, 29, "park");
        drop(logger);
        assert_instances(&path);
    }

    #[test]
    fn test_appended_restart_with_repeated_ids_selects_all_streams() {
        let dir = TempDir::new_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("restart.copper");
        let logger = writer(&path, false);
        write_instance(&logger, 17, "drive");
        drop(logger);
        let logger = writer(&path, true);
        write_instance(&logger, 29, "park");
        drop(logger);
        assert_instances(&path);
    }

    #[test]
    fn test_log_without_lifecycle_has_one_implicit_instance() {
        let dir = TempDir::new_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("standalone.copper");
        drop(writer(&path, false));
        let catalog = discover(&path).unwrap();
        assert!(select(&catalog, None).unwrap().stack.is_none());
    }

    #[test]
    fn test_logstream_order_and_optional_keyframes_across_slabs() {
        let dir = TempDir::new_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("slabs.copper");
        let logger = writer(&path, false);
        write_instance_with_options(&logger, 17, "drive", true, false, 10_000);
        write_instance_with_options(&logger, 29, "park", false, true, 10_000);
        drop(logger);
        let catalog = discover(&path).unwrap();
        assert_eq!(catalog.len(), 2);
        assert!(catalog[1].start.slab_index > 0);
        for (index, expected) in [17, 29].into_iter().enumerate() {
            let instance = select(&catalog, Some(index)).unwrap();
            let entries = copperlists_reader::<Messages>(
                instance
                    .reader(&path)
                    .unwrap()
                    .stream(UnifiedLogType::CopperList),
            )
            .collect::<Vec<_>>();
            assert_eq!(entries.len(), 10_000);
            for (id, entry) in entries.iter().enumerate() {
                assert_eq!(entry.id, id as u64);
                assert_eq!(entry.msgs.0, expected);
            }
            crate::fsck::check::<Messages>(&mut instance.reader(&path).unwrap(), 0, false).unwrap();
        }
    }

    #[test]
    fn test_selection_does_not_decode_other_instances_payloads() {
        let dir = TempDir::new_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("schemas.copper");
        let logger = writer(&path, false);
        write_instance(&logger, 17, "drive");
        {
            let mut different_schema = stream_write::<String, MmapSectionStorage>(
                logger.clone(),
                UnifiedLogType::CopperList,
                1024,
            )
            .unwrap();
            different_schema
                .log(&"a different application payload".to_string())
                .unwrap();
        }
        write_instance(&logger, 29, "park");
        drop(logger);
        let catalog = discover(&path).unwrap();
        let selected = select(&catalog, Some(1)).unwrap();
        crate::fsck::check::<Messages>(&mut selected.reader(&path).unwrap(), 0, false).unwrap();
    }

    #[test]
    fn test_fsck_rejects_cl_reset_without_instantiated() {
        let dir = TempDir::new_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("reset.copper");
        let logger = writer(&path, false);
        write_instance(&logger, 17, "drive");
        {
            let mut stream = stream_write::<CopperList<Messages>, MmapSectionStorage>(
                logger.clone(),
                UnifiedLogType::CopperList,
                1024,
            )
            .unwrap();
            stream.log(&CopperList::new(0, Messages(29))).unwrap();
        }
        drop(logger);
        let catalog = discover(&path).unwrap();
        let selected = select(&catalog, None).unwrap();
        let error = crate::fsck::check::<Messages>(&mut selected.reader(&path).unwrap(), 0, false)
            .unwrap_err();
        assert!(error.to_string().contains("IDs must increase"));
    }

    #[test]
    fn test_fsck_reports_corrupted_keyframe_and_unclean_close() {
        let dir = TempDir::new_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("incomplete.copper");
        let logger = writer(&path, false);
        write_instance(&logger, 17, "drive");
        let catalog = discover(&path).unwrap();
        let selected = select(&catalog, None).unwrap();
        let error = crate::fsck::check::<Messages>(&mut selected.reader(&path).unwrap(), 0, false)
            .unwrap_err();
        assert!(error.to_string().contains("temporary end marker"));
        {
            let mut stream = stream_write::<u8, MmapSectionStorage>(
                logger.clone(),
                UnifiedLogType::FrozenTasks,
                1024,
            )
            .unwrap();
            stream.log(&255).unwrap();
        }
        drop(logger);
        let catalog = discover(&path).unwrap();
        let selected = select(&catalog, None).unwrap();
        let error = crate::fsck::check::<Messages>(&mut selected.reader(&path).unwrap(), 0, false)
            .unwrap_err();
        assert!(error.to_string().contains("Corrupted keyframe"));
    }

    #[test]
    fn test_unknown_startup_boundary_is_listed_but_cannot_be_selected() {
        let dir = TempDir::new_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("unknown.copper");
        let logger = writer(&path, false);
        write_instance(&logger, 17, "drive");
        let catalog = discover(&path).unwrap();
        let mut lifecycle = crate::runtime_lifecycle_reader(
            catalog[0]
                .reader(&path)
                .unwrap()
                .stream(UnifiedLogType::RuntimeLifecycle),
        );
        let instantiated = lifecycle.next().unwrap();
        drop(lifecycle);
        {
            let mut unrelated = stream_write::<u32, MmapSectionStorage>(
                logger.clone(),
                UnifiedLogType::CopperList,
                1024,
            )
            .unwrap();
            unrelated.log(&7).unwrap();
            let mut lifecycle = stream_write::<RuntimeLifecycleRecord, MmapSectionStorage>(
                logger.clone(),
                UnifiedLogType::RuntimeLifecycle,
                1024,
            )
            .unwrap();
            lifecycle.log(&instantiated).unwrap();
        }
        drop(logger);
        let catalog = discover(&path).unwrap();
        assert_eq!(catalog.len(), 2);
        assert!(select(&catalog, Some(0)).is_err());
        assert!(select(&catalog, Some(1)).is_err());
    }

    #[test]
    fn test_cli_selects_recorded_config_and_mission_for_stats() {
        use clap::Parser;
        let dir = TempDir::new_in(env!("CARGO_MANIFEST_DIR")).unwrap();
        let path = dir.path().join("cli.copper");
        let logger = writer(&path, false);
        write_instance(&logger, 17, "drive");
        write_instance(&logger, 29, "park");
        drop(logger);
        let path = path.to_str().unwrap();
        let args =
            crate::LogReaderCli::try_parse_from(["logreader", path, "list-instances"]).unwrap();
        crate::run_cli_with_args::<Messages>(args).unwrap();
        let args = crate::LogReaderCli::try_parse_from(["logreader", path, "fsck"]).unwrap();
        assert!(
            crate::run_cli_with_args::<Messages>(args)
                .unwrap_err()
                .to_string()
                .contains("--instance")
        );
        let args =
            crate::LogReaderCli::try_parse_from(["logreader", path, "fsck", "--instance", "1"])
                .unwrap();
        crate::run_cli_with_args::<Messages>(args).unwrap();
        let args = crate::LogReaderCli::try_parse_from([
            "logreader",
            path,
            "--instance",
            "1",
            "extract-copperlists",
        ])
        .unwrap();
        crate::run_cli_with_args::<Messages>(args).unwrap();
        let output = dir.path().join("stats.json");
        let args = crate::LogReaderCli::try_parse_from([
            "logreader",
            path,
            "--instance",
            "1",
            "log-stats",
            "--output",
            output.to_str().unwrap(),
        ])
        .unwrap();
        crate::run_cli_with_args::<Messages>(args).unwrap();
        let stats: serde_json::Value =
            serde_json::from_slice(&std::fs::read(output).unwrap()).unwrap();
        assert_eq!(stats["mission"], "park");
        assert!(
            cu29::logcodec::effective_config_entry::<Messages>("")
                .ron()
                .contains("// 29")
        );
    }
}
