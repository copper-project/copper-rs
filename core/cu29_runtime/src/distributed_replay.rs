//! Discovery, validation, planning, and causal execution helpers for
//! distributed deterministic replay.
//!
//! The distributed replay flow is:
//! - discover Copper logs and recover runtime identity from lifecycle metadata
//! - validate those logs against a strict multi-Copper topology
//! - register the generated replayable app type for each subsystem
//! - build one replay session per `(instance_id, subsystem_id)` assignment
//! - stitch sessions together through recorded message provenance
//! - replay the fleet in a stable causal order

use crate::app::{
    CuDistributedReplayApplication, CuRecordedReplayApplication, CuSimApplication, Subsystem,
};
use crate::config::{MultiCopperConfig, read_configuration_str, read_multi_configuration};
use crate::copperlist::CopperList;
use crate::curuntime::{
    KeyFrame, RuntimeLifecycleConfigSource, RuntimeLifecycleEvent, RuntimeLifecycleRecord,
    RuntimeLifecycleStackInfo,
};
use crate::debug::{
    SectionIndexEntry, build_read_logger, decode_copperlists, index_log, read_section_at,
};
use crate::simulation::recorded_copperlist_timestamp;
use bincode::config::standard;
use bincode::decode_from_std_read;
use bincode::error::DecodeError;
use cu29_clock::{RobotClock, RobotClockMock};
use cu29_traits::{CopperListTuple, CuError, CuResult, ErasedCuStampedDataSet, UnifiedLogType};
use cu29_unifiedlog::{
    NoopLogger, NoopSectionStorage, UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader,
    UnifiedLoggerRead,
};
use std::any::type_name;
use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::fmt::{Debug, Display, Formatter, Result as FmtResult};
use std::fs;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

/// One discovered Copper log that can participate in distributed replay.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DistributedReplayLog {
    pub base_path: PathBuf,
    pub stack: RuntimeLifecycleStackInfo,
    pub config_source: RuntimeLifecycleConfigSource,
    pub effective_config_ron: String,
    pub mission: Option<String>,
}

impl DistributedReplayLog {
    /// Discover a single Copper log from either its base path (`foo.copper`) or
    /// one of its slab paths (`foo_0.copper`, `foo_1.copper`, ...).
    pub fn discover(path: impl AsRef<Path>) -> CuResult<Self> {
        let requested_path = path.as_ref();
        let normalized_path = normalize_candidate_log_base(requested_path);
        match Self::discover_from_base_path(requested_path) {
            Ok(log) => Ok(log),
            Err(_) if normalized_path != requested_path => {
                Self::discover_from_base_path(&normalized_path)
            }
            Err(err) => Err(err),
        }
    }

    fn discover_from_base_path(base_path: &Path) -> CuResult<Self> {
        let UnifiedLogger::Read(read_logger) = UnifiedLoggerBuilder::new()
            .file_base_name(base_path)
            .build()
            .map_err(|err| {
                CuError::new_with_cause(
                    &format!(
                        "Failed to open Copper log '{}' for distributed replay discovery",
                        base_path.display()
                    ),
                    err,
                )
            })?
        else {
            return Err(CuError::from(
                "Expected a readable unified logger during distributed replay discovery",
            ));
        };

        let mut reader = UnifiedLoggerIOReader::new(read_logger, UnifiedLogType::RuntimeLifecycle);
        let mut instantiated: Option<(
            RuntimeLifecycleConfigSource,
            String,
            RuntimeLifecycleStackInfo,
        )> = None;
        let mut mission = None;

        while let Some(record) =
            read_next_entry::<RuntimeLifecycleRecord>(&mut reader).map_err(|err| {
                CuError::from(format!(
                    "Failed to decode runtime lifecycle for '{}': {err}",
                    base_path.display()
                ))
            })?
        {
            match record.event {
                RuntimeLifecycleEvent::Instantiated {
                    config_source,
                    effective_config_ron,
                    stack,
                } if instantiated.is_none() => {
                    instantiated = Some((config_source, effective_config_ron, stack));
                }
                RuntimeLifecycleEvent::MissionStarted {
                    mission: started_mission,
                } if mission.is_none() => {
                    mission = Some(started_mission);
                }
                _ => {}
            }

            if instantiated.is_some() && mission.is_some() {
                break;
            }
        }

        let Some((config_source, effective_config_ron, stack)) = instantiated else {
            return Err(CuError::from(format!(
                "Copper log '{}' has no RuntimeLifecycle::Instantiated record",
                base_path.display()
            )));
        };

        Ok(Self {
            base_path: base_path.to_path_buf(),
            stack,
            config_source,
            effective_config_ron,
            mission,
        })
    }

    #[inline]
    pub fn instance_id(&self) -> u32 {
        self.stack.instance_id
    }

    #[inline]
    pub fn subsystem_code(&self) -> u16 {
        self.stack.subsystem_code
    }

    #[inline]
    pub fn subsystem_id(&self) -> Option<&str> {
        self.stack.subsystem_id.as_deref()
    }
}

/// Discovery error recorded for one log candidate.
#[derive(Debug, Clone)]
pub struct DistributedReplayDiscoveryFailure {
    pub candidate_path: PathBuf,
    pub error: String,
}

impl Display for DistributedReplayDiscoveryFailure {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(
            f,
            "{}: {}",
            self.candidate_path.display(),
            self.error.as_str()
        )
    }
}

/// Result of scanning one or more paths for distributed replay logs.
#[derive(Debug, Clone, Default)]
pub struct DistributedReplayCatalog {
    pub logs: Vec<DistributedReplayLog>,
    pub failures: Vec<DistributedReplayDiscoveryFailure>,
}

impl DistributedReplayCatalog {
    /// Discover logs from a list of files and/or directories.
    ///
    /// Directories are traversed recursively. Any physical slab file
    /// (`*_0.copper`, `*_1.copper`, ...) is normalized back to its base log path.
    pub fn discover<I, P>(inputs: I) -> CuResult<Self>
    where
        I: IntoIterator<Item = P>,
        P: AsRef<Path>,
    {
        let mut candidates = BTreeSet::new();
        for input in inputs {
            collect_candidate_base_paths(input.as_ref(), &mut candidates)?;
        }

        let mut logs = Vec::new();
        let mut failures = Vec::new();

        for candidate in candidates {
            match DistributedReplayLog::discover(&candidate) {
                Ok(log) => logs.push(log),
                Err(err) => failures.push(DistributedReplayDiscoveryFailure {
                    candidate_path: candidate,
                    error: err.to_string(),
                }),
            }
        }

        logs.sort_by(|left, right| {
            (
                left.instance_id(),
                left.subsystem_code(),
                left.subsystem_id(),
                left.base_path.as_os_str(),
            )
                .cmp(&(
                    right.instance_id(),
                    right.subsystem_code(),
                    right.subsystem_id(),
                    right.base_path.as_os_str(),
                ))
        });
        failures.sort_by(|left, right| left.candidate_path.cmp(&right.candidate_path));

        Ok(Self { logs, failures })
    }

    /// Convenience wrapper for recursive discovery rooted at one directory.
    pub fn discover_under(root: impl AsRef<Path>) -> CuResult<Self> {
        Self::discover([root])
    }
}

type DistributedReplaySessionFactory =
    fn(&DistributedReplayAssignment) -> CuResult<DistributedReplaySessionBuild>;

const DEFAULT_SECTION_CACHE_CAP: usize = 8;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
struct DistributedReplayOriginKey {
    instance_id: u32,
    subsystem_code: u16,
    cl_id: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct DistributedReplayCursor {
    pub instance_id: u32,
    pub subsystem_id: String,
    pub cl_id: u64,
    subsystem_code: u16,
}

impl DistributedReplayCursor {
    #[inline]
    fn new(instance_id: u32, subsystem_id: String, subsystem_code: u16, cl_id: u64) -> Self {
        Self {
            instance_id,
            subsystem_id,
            cl_id,
            subsystem_code,
        }
    }

    #[inline]
    pub fn subsystem_code(&self) -> u16 {
        self.subsystem_code
    }
}

#[derive(Debug, Clone)]
struct DistributedReplayNodeDescriptor {
    cursor: DistributedReplayCursor,
    origin_key: DistributedReplayOriginKey,
    incoming_origins: BTreeSet<DistributedReplayOriginKey>,
}

#[derive(Debug, Clone)]
struct DistributedReplayGraphNode {
    cursor: DistributedReplayCursor,
    session_index: usize,
    outgoing: Vec<usize>,
    initial_dependencies: usize,
    remaining_dependencies: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct DistributedReplayReadyNode {
    instance_id: u32,
    subsystem_code: u16,
    cl_id: u64,
    node_index: usize,
}

struct DistributedReplaySessionBuild {
    session: Box<dyn DistributedReplaySession>,
    nodes: Vec<DistributedReplayNodeDescriptor>,
}

trait DistributedReplaySession {
    fn goto_cl(&mut self, cl_id: u64) -> CuResult<()>;
    fn shutdown(&mut self) -> CuResult<()>;
}

#[derive(Debug, Clone)]
struct RecordedReplayCachedSection<P: CopperListTuple> {
    entries: Vec<Arc<CopperList<P>>>,
}

struct RecordedReplaySession<App, P>
where
    App: CuDistributedReplayApplication<NoopSectionStorage, NoopLogger>,
    P: CopperListTuple,
{
    assignment: DistributedReplayAssignment,
    app: App,
    clock_mock: RobotClockMock,
    log_reader: UnifiedLoggerRead,
    sections: Vec<SectionIndexEntry>,
    total_entries: usize,
    keyframes: Vec<KeyFrame>,
    started: bool,
    current_idx: Option<usize>,
    last_keyframe: Option<u64>,
    cache: HashMap<usize, RecordedReplayCachedSection<P>>,
    cache_order: VecDeque<usize>,
    cache_cap: usize,
}

impl<App, P> RecordedReplaySession<App, P>
where
    App: CuDistributedReplayApplication<NoopSectionStorage, NoopLogger>
        + CuRecordedReplayApplication<NoopSectionStorage, NoopLogger, RecordedDataSet = P>,
    P: CopperListTuple,
{
    fn from_log(
        assignment: DistributedReplayAssignment,
        app: App,
        clock: RobotClock,
        clock_mock: RobotClockMock,
        log_base: &Path,
    ) -> CuResult<Self> {
        let (sections, keyframes, total_entries) =
            index_log::<P, _>(log_base, &recorded_copperlist_timestamp::<P>)?;
        let log_reader = build_read_logger(log_base)?;
        let _ = clock;
        Ok(Self {
            assignment,
            app,
            clock_mock,
            log_reader,
            sections,
            total_entries,
            keyframes,
            started: false,
            current_idx: None,
            last_keyframe: None,
            cache: HashMap::new(),
            cache_order: VecDeque::new(),
            cache_cap: DEFAULT_SECTION_CACHE_CAP,
        })
    }

    fn describe_nodes(&mut self) -> CuResult<Vec<DistributedReplayNodeDescriptor>> {
        let mut nodes = Vec::with_capacity(self.total_entries);
        for idx in 0..self.total_entries {
            let (copperlist, _) = self.copperlist_at(idx)?;
            let cursor = DistributedReplayCursor::new(
                self.assignment.instance_id,
                self.assignment.subsystem_id.clone(),
                self.assignment.log.subsystem_code(),
                copperlist.id,
            );
            nodes.push(DistributedReplayNodeDescriptor {
                origin_key: DistributedReplayOriginKey {
                    instance_id: cursor.instance_id,
                    subsystem_code: cursor.subsystem_code(),
                    cl_id: cursor.cl_id,
                },
                incoming_origins: copperlist_origins(copperlist.as_ref()),
                cursor,
            });
        }
        Ok(nodes)
    }

    fn ensure_started(&mut self) -> CuResult<()> {
        if self.started {
            return Ok(());
        }
        let mut noop = |_step: App::Step<'_>| crate::simulation::SimOverride::ExecuteByRuntime;
        <App as CuSimApplication<NoopSectionStorage, NoopLogger>>::start_all_tasks(
            &mut self.app,
            &mut noop,
        )?;
        self.started = true;
        Ok(())
    }

    fn nearest_keyframe(&self, target_cl_id: u64) -> Option<KeyFrame> {
        self.keyframes
            .iter()
            .filter(|keyframe| keyframe.culistid <= target_cl_id)
            .max_by_key(|keyframe| keyframe.culistid)
            .cloned()
    }

    fn restore_keyframe(&mut self, keyframe: &KeyFrame) -> CuResult<()> {
        <App as CuSimApplication<NoopSectionStorage, NoopLogger>>::restore_keyframe(
            &mut self.app,
            keyframe,
        )?;
        self.clock_mock.set_value(keyframe.timestamp.as_nanos());
        self.last_keyframe = Some(keyframe.culistid);
        Ok(())
    }

    fn find_section_for_index(&self, idx: usize) -> Option<usize> {
        self.sections
            .binary_search_by(|section| {
                if idx < section.start_idx {
                    std::cmp::Ordering::Greater
                } else if idx >= section.start_idx + section.len {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Equal
                }
            })
            .ok()
    }

    fn find_section_for_cl_id(&self, cl_id: u64) -> Option<usize> {
        self.sections
            .binary_search_by(|section| {
                if cl_id < section.first_id {
                    std::cmp::Ordering::Greater
                } else if cl_id > section.last_id {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Equal
                }
            })
            .ok()
    }

    fn touch_cache(&mut self, key: usize) {
        if let Some(position) = self.cache_order.iter().position(|entry| *entry == key) {
            self.cache_order.remove(position);
        }
        self.cache_order.push_back(key);
        while self.cache_order.len() > self.cache_cap {
            if let Some(oldest) = self.cache_order.pop_front() {
                self.cache.remove(&oldest);
            }
        }
    }

    fn load_section(&mut self, section_idx: usize) -> CuResult<&RecordedReplayCachedSection<P>> {
        if self.cache.contains_key(&section_idx) {
            self.touch_cache(section_idx);
            return Ok(self.cache.get(&section_idx).expect("cache entry exists"));
        }

        let entry = &self.sections[section_idx];
        let (header, data) = read_section_at(&mut self.log_reader, entry.pos)?;
        if header.entry_type != UnifiedLogType::CopperList {
            return Err(CuError::from(
                "Section type mismatch while loading distributed replay copperlists",
            ));
        }
        let (entries, _) = decode_copperlists::<P, _>(&data, &recorded_copperlist_timestamp::<P>)?;
        self.cache
            .insert(section_idx, RecordedReplayCachedSection { entries });
        self.touch_cache(section_idx);
        Ok(self.cache.get(&section_idx).expect("cache entry exists"))
    }

    fn copperlist_at(&mut self, idx: usize) -> CuResult<(Arc<CopperList<P>>, Option<KeyFrame>)> {
        let section_idx = self
            .find_section_for_index(idx)
            .ok_or_else(|| CuError::from("Distributed replay index is outside the log"))?;
        let start_idx = self.sections[section_idx].start_idx;
        let section = self.load_section(section_idx)?;
        let local_idx = idx - start_idx;
        let copperlist = section
            .entries
            .get(local_idx)
            .ok_or_else(|| CuError::from("Corrupt distributed replay section index"))?
            .clone();
        let keyframe = self
            .keyframes
            .iter()
            .find(|keyframe| keyframe.culistid == copperlist.id)
            .cloned();
        Ok((copperlist, keyframe))
    }

    fn index_for_cl_id(&mut self, cl_id: u64) -> CuResult<usize> {
        let section_idx = self
            .find_section_for_cl_id(cl_id)
            .ok_or_else(|| CuError::from("Requested CopperList id is not present in the log"))?;
        let start_idx = self.sections[section_idx].start_idx;
        let section = self.load_section(section_idx)?;
        for (offset, copperlist) in section.entries.iter().enumerate() {
            if copperlist.id == cl_id {
                return Ok(start_idx + offset);
            }
        }
        Err(CuError::from(
            "Requested CopperList id is missing from its indexed log section",
        ))
    }

    fn replay_range(
        &mut self,
        start_idx: usize,
        end_idx: usize,
        replay_keyframe: Option<&KeyFrame>,
    ) -> CuResult<()> {
        for idx in start_idx..=end_idx {
            let (copperlist, keyframe) = self.copperlist_at(idx)?;
            let keyframe = replay_keyframe
                .filter(|candidate| candidate.culistid == copperlist.id)
                .or(keyframe
                    .as_ref()
                    .filter(|candidate| candidate.culistid == copperlist.id));
            <App as CuRecordedReplayApplication<NoopSectionStorage, NoopLogger>>::replay_recorded_copperlist(
                &mut self.app,
                &self.clock_mock,
                copperlist.as_ref(),
                keyframe,
            )?;
            self.current_idx = Some(idx);
        }
        Ok(())
    }

    fn goto_index(&mut self, target_idx: usize) -> CuResult<()> {
        self.ensure_started()?;
        if target_idx >= self.total_entries {
            return Err(CuError::from(
                "Distributed replay target is outside the log",
            ));
        }

        let (target_copperlist, _) = self.copperlist_at(target_idx)?;
        let target_cl_id = target_copperlist.id;

        let replay_start_idx;
        let replay_keyframe;

        if let Some(current_idx) = self.current_idx {
            if current_idx == target_idx {
                return Ok(());
            }

            if target_idx > current_idx {
                replay_start_idx = current_idx + 1;
                replay_keyframe = None;
            } else {
                let keyframe = self.nearest_keyframe(target_cl_id).ok_or_else(|| {
                    CuError::from("No keyframe is available to rewind distributed replay")
                })?;
                self.restore_keyframe(&keyframe)?;
                replay_start_idx = self.index_for_cl_id(keyframe.culistid)?;
                replay_keyframe = Some(keyframe);
            }
        } else {
            let keyframe = self.nearest_keyframe(target_cl_id).ok_or_else(|| {
                CuError::from("No keyframe is available to initialize distributed replay")
            })?;
            self.restore_keyframe(&keyframe)?;
            replay_start_idx = self.index_for_cl_id(keyframe.culistid)?;
            replay_keyframe = Some(keyframe);
        }

        self.replay_range(replay_start_idx, target_idx, replay_keyframe.as_ref())
    }
}

impl<App, P> DistributedReplaySession for RecordedReplaySession<App, P>
where
    App: CuDistributedReplayApplication<NoopSectionStorage, NoopLogger>
        + CuRecordedReplayApplication<NoopSectionStorage, NoopLogger, RecordedDataSet = P>,
    P: CopperListTuple + 'static,
{
    fn goto_cl(&mut self, cl_id: u64) -> CuResult<()> {
        let target_idx = self.index_for_cl_id(cl_id)?;
        self.goto_index(target_idx)
    }

    fn shutdown(&mut self) -> CuResult<()> {
        if !self.started {
            return Ok(());
        }

        let mut noop = |_step: App::Step<'_>| crate::simulation::SimOverride::ExecuteByRuntime;
        <App as CuSimApplication<NoopSectionStorage, NoopLogger>>::stop_all_tasks(
            &mut self.app,
            &mut noop,
        )?;
        self.started = false;
        Ok(())
    }
}

/// One typed subsystem registration provided to the distributed replay builder.
#[derive(Clone)]
pub struct DistributedReplayAppRegistration {
    pub subsystem: Subsystem,
    pub app_type_name: &'static str,
    session_factory: DistributedReplaySessionFactory,
}

impl Debug for DistributedReplayAppRegistration {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        f.debug_struct("DistributedReplayAppRegistration")
            .field("subsystem", &self.subsystem)
            .field("app_type_name", &self.app_type_name)
            .finish()
    }
}

impl PartialEq for DistributedReplayAppRegistration {
    fn eq(&self, other: &Self) -> bool {
        self.subsystem == other.subsystem && self.app_type_name == other.app_type_name
    }
}

impl Eq for DistributedReplayAppRegistration {}

/// One validated log assignment for a subsystem instance.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DistributedReplayAssignment {
    pub instance_id: u32,
    pub subsystem_id: String,
    pub log: DistributedReplayLog,
    pub registration: DistributedReplayAppRegistration,
}

/// Validated replay plan produced by [`DistributedReplayBuilder`].
#[derive(Debug, Clone)]
pub struct DistributedReplayPlan {
    pub multi_config_path: PathBuf,
    pub multi_config: MultiCopperConfig,
    pub catalog: DistributedReplayCatalog,
    pub selected_instances: Vec<u32>,
    pub mission: Option<String>,
    pub registrations: Vec<DistributedReplayAppRegistration>,
    pub assignments: Vec<DistributedReplayAssignment>,
}

impl DistributedReplayPlan {
    #[inline]
    pub fn builder(multi_config_path: impl AsRef<Path>) -> CuResult<DistributedReplayBuilder> {
        DistributedReplayBuilder::new(multi_config_path)
    }

    #[inline]
    pub fn assignment(
        &self,
        instance_id: u32,
        subsystem_id: &str,
    ) -> Option<&DistributedReplayAssignment> {
        self.assignments.iter().find(|assignment| {
            assignment.instance_id == instance_id && assignment.subsystem_id == subsystem_id
        })
    }

    /// Build a causal distributed replay engine from this validated plan.
    pub fn start(self) -> CuResult<DistributedReplayEngine> {
        DistributedReplayEngine::new(self)
    }
}

/// Aggregated validation diagnostics emitted while constructing a distributed replay plan.
#[derive(Debug, Clone, Default)]
pub struct DistributedReplayValidationError {
    pub issues: Vec<String>,
}

impl DistributedReplayValidationError {
    fn push(&mut self, issue: impl Into<String>) {
        self.issues.push(issue.into());
    }

    fn is_empty(&self) -> bool {
        self.issues.is_empty()
    }
}

impl Display for DistributedReplayValidationError {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        writeln!(f, "Distributed replay validation failed:")?;
        for issue in &self.issues {
            writeln!(f, " - {issue}")?;
        }
        Ok(())
    }
}

/// Builder for a validated distributed replay plan.
#[derive(Debug, Clone)]
pub struct DistributedReplayBuilder {
    multi_config_path: PathBuf,
    multi_config: MultiCopperConfig,
    discovery_inputs: Vec<PathBuf>,
    catalog: Option<DistributedReplayCatalog>,
    registrations: BTreeMap<String, DistributedReplayAppRegistration>,
    selected_instances: Option<BTreeSet<u32>>,
}

impl DistributedReplayBuilder {
    /// Load a strict multi-Copper config and start building a distributed replay plan.
    pub fn new(multi_config_path: impl AsRef<Path>) -> CuResult<Self> {
        let multi_config_path = multi_config_path.as_ref().to_path_buf();
        let multi_config = read_multi_configuration(&multi_config_path.to_string_lossy())?;
        Ok(Self {
            multi_config_path,
            multi_config,
            discovery_inputs: Vec::new(),
            catalog: None,
            registrations: BTreeMap::new(),
            selected_instances: None,
        })
    }

    /// Replace the discovered catalog explicitly.
    pub fn with_catalog(mut self, catalog: DistributedReplayCatalog) -> Self {
        self.catalog = Some(catalog);
        self
    }

    /// Discover logs from files and/or directories.
    ///
    /// Directories are walked recursively by [`DistributedReplayCatalog`].
    pub fn discover_logs<I, P>(mut self, inputs: I) -> CuResult<Self>
    where
        I: IntoIterator<Item = P>,
        P: AsRef<Path>,
    {
        self.discovery_inputs
            .extend(inputs.into_iter().map(|path| path.as_ref().to_path_buf()));
        self.catalog = Some(DistributedReplayCatalog::discover(
            self.discovery_inputs.iter().collect::<Vec<_>>(),
        )?);
        Ok(self)
    }

    /// Convenience wrapper for recursive discovery under one root directory.
    pub fn discover_logs_under(self, root: impl AsRef<Path>) -> CuResult<Self> {
        self.discover_logs([root.as_ref().to_path_buf()])
    }

    /// Restrict plan construction to a subset of instance ids.
    pub fn instances<I>(mut self, instances: I) -> Self
    where
        I: IntoIterator<Item = u32>,
    {
        self.selected_instances = Some(instances.into_iter().collect());
        self
    }

    /// Register the generated app type expected for one subsystem.
    pub fn register<App>(mut self, subsystem_id: &str) -> CuResult<Self>
    where
        App: CuDistributedReplayApplication<NoopSectionStorage, NoopLogger> + 'static,
    {
        if self.registrations.contains_key(subsystem_id) {
            return Err(CuError::from(format!(
                "Subsystem '{}' is already registered for distributed replay",
                subsystem_id
            )));
        }

        let expected_subsystem = self.multi_config.subsystem(subsystem_id).ok_or_else(|| {
            CuError::from(format!(
                "Multi-Copper config '{}' does not define subsystem '{}'",
                self.multi_config_path.display(),
                subsystem_id
            ))
        })?;

        let registered_subsystem = App::subsystem();
        let Some(registered_subsystem_id) = registered_subsystem.id() else {
            return Err(CuError::from(format!(
                "App type '{}' was not generated for a multi-Copper subsystem and cannot be registered for distributed replay",
                type_name::<App>()
            )));
        };

        if registered_subsystem_id != subsystem_id {
            return Err(CuError::from(format!(
                "App type '{}' declares subsystem '{}' but was registered as '{}'",
                type_name::<App>(),
                registered_subsystem_id,
                subsystem_id
            )));
        }

        let registered_subsystem_code = registered_subsystem.code();
        if registered_subsystem_code != expected_subsystem.subsystem_code {
            return Err(CuError::from(format!(
                "App type '{}' declares subsystem code {} for '{}' but multi-Copper config '{}' expects {}",
                type_name::<App>(),
                registered_subsystem_code,
                subsystem_id,
                self.multi_config_path.display(),
                expected_subsystem.subsystem_code
            )));
        }

        self.registrations.insert(
            subsystem_id.to_string(),
            DistributedReplayAppRegistration {
                subsystem: registered_subsystem,
                app_type_name: type_name::<App>(),
                session_factory: build_distributed_replay_session::<App>,
            },
        );
        Ok(self)
    }

    /// Validate discovery + registrations and prepare a typed replay plan.
    pub fn build(self) -> CuResult<DistributedReplayPlan> {
        let catalog = match self.catalog {
            Some(catalog) => catalog,
            None if self.discovery_inputs.is_empty() => DistributedReplayCatalog::default(),
            None => DistributedReplayCatalog::discover(
                self.discovery_inputs.iter().collect::<Vec<_>>(),
            )?,
        };

        let mut validation = DistributedReplayValidationError::default();

        for failure in &catalog.failures {
            validation.push(format!(
                "discovery failure for '{}': {}",
                failure.candidate_path.display(),
                failure.error
            ));
        }

        let subsystem_map: BTreeMap<_, _> = self
            .multi_config
            .subsystems
            .iter()
            .map(|subsystem| (subsystem.id.clone(), subsystem))
            .collect();

        for subsystem in subsystem_map.keys() {
            if !self.registrations.contains_key(subsystem) {
                validation.push(format!(
                    "missing app registration for subsystem '{}'",
                    subsystem
                ));
            }
        }

        let mut discovered_instances = BTreeSet::new();
        let mut logs_by_target: BTreeMap<(u32, String), Vec<DistributedReplayLog>> =
            BTreeMap::new();

        for log in &catalog.logs {
            let Some(subsystem_id) = log.subsystem_id() else {
                validation.push(format!(
                    "discovered log '{}' is missing subsystem_id runtime metadata",
                    log.base_path.display()
                ));
                continue;
            };

            let Some(expected_subsystem) = subsystem_map.get(subsystem_id) else {
                validation.push(format!(
                    "discovered log '{}' belongs to subsystem '{}' which is not present in multi-Copper config '{}'",
                    log.base_path.display(),
                    subsystem_id,
                    self.multi_config_path.display()
                ));
                continue;
            };

            if log.subsystem_code() != expected_subsystem.subsystem_code {
                validation.push(format!(
                    "discovered log '{}' reports subsystem code {} for '{}' but multi-Copper config '{}' expects {}",
                    log.base_path.display(),
                    log.subsystem_code(),
                    subsystem_id,
                    self.multi_config_path.display(),
                    expected_subsystem.subsystem_code
                ));
            }

            discovered_instances.insert(log.instance_id());
            logs_by_target
                .entry((log.instance_id(), subsystem_id.to_string()))
                .or_default()
                .push(log.clone());
        }

        for ((instance_id, subsystem_id), logs) in &logs_by_target {
            if logs.len() > 1 {
                validation.push(format!(
                    "found {} logs for instance {} subsystem '{}': {}",
                    logs.len(),
                    instance_id,
                    subsystem_id,
                    join_log_paths(logs)
                ));
            }
        }

        let selected_instances: Vec<u32> =
            if let Some(selected_instances) = &self.selected_instances {
                let mut selected_instances: Vec<_> = selected_instances.iter().copied().collect();
                selected_instances.sort_unstable();
                for instance_id in &selected_instances {
                    if !discovered_instances.contains(instance_id) {
                        validation.push(format!(
                            "selected instance {} has no discovered logs",
                            instance_id
                        ));
                    }
                }
                selected_instances
            } else {
                discovered_instances.iter().copied().collect()
            };

        if selected_instances.is_empty() {
            validation.push("no instances selected for distributed replay");
        }

        for instance_id in &selected_instances {
            for subsystem in &self.multi_config.subsystems {
                if !logs_by_target.contains_key(&(*instance_id, subsystem.id.clone())) {
                    validation.push(format!(
                        "missing log for instance {} subsystem '{}'",
                        instance_id, subsystem.id
                    ));
                }
            }
        }

        let mut known_missions = BTreeSet::new();
        for instance_id in &selected_instances {
            for subsystem in &self.multi_config.subsystems {
                if let Some(logs) = logs_by_target.get(&(*instance_id, subsystem.id.clone()))
                    && let Some(log) = logs.first()
                    && let Some(mission) = &log.mission
                {
                    known_missions.insert(mission.clone());
                }
            }
        }
        if known_missions.len() > 1 {
            validation.push(format!(
                "selected logs disagree on mission: {}",
                known_missions.into_iter().collect::<Vec<_>>().join(", ")
            ));
        }

        if !validation.is_empty() {
            return Err(CuError::from(validation.to_string()));
        }

        let mission = selected_instances
            .iter()
            .flat_map(|instance_id| {
                self.multi_config.subsystems.iter().filter_map(|subsystem| {
                    logs_by_target
                        .get(&(*instance_id, subsystem.id.clone()))
                        .and_then(|logs| logs.first())
                        .and_then(|log| log.mission.clone())
                })
            })
            .next();

        let mut registrations: Vec<_> = self.registrations.into_values().collect();
        registrations.sort_by(|left, right| left.subsystem.id().cmp(&right.subsystem.id()));

        let mut assignments = Vec::new();
        for instance_id in &selected_instances {
            for subsystem in &self.multi_config.subsystems {
                let log = logs_by_target
                    .get(&(*instance_id, subsystem.id.clone()))
                    .and_then(|logs| logs.first())
                    .expect("validated distributed replay plan is missing a log")
                    .clone();
                let registration = registrations
                    .iter()
                    .find(|registration| registration.subsystem.id() == Some(subsystem.id.as_str()))
                    .expect("validated distributed replay plan is missing a registration")
                    .clone();
                assignments.push(DistributedReplayAssignment {
                    instance_id: *instance_id,
                    subsystem_id: subsystem.id.clone(),
                    log,
                    registration,
                });
            }
        }
        assignments.sort_by(|left, right| {
            (
                left.instance_id,
                left.registration.subsystem.code(),
                left.subsystem_id.as_str(),
            )
                .cmp(&(
                    right.instance_id,
                    right.registration.subsystem.code(),
                    right.subsystem_id.as_str(),
                ))
        });

        Ok(DistributedReplayPlan {
            multi_config_path: self.multi_config_path,
            multi_config: self.multi_config,
            catalog,
            selected_instances,
            mission,
            registrations,
            assignments,
        })
    }
}

fn build_distributed_replay_session<App>(
    assignment: &DistributedReplayAssignment,
) -> CuResult<DistributedReplaySessionBuild>
where
    App: CuDistributedReplayApplication<NoopSectionStorage, NoopLogger> + 'static,
{
    let config = read_configuration_str(assignment.log.effective_config_ron.clone(), None)
        .map_err(|err| {
            CuError::from(format!(
                "Failed to parse recorded effective config from '{}': {err}",
                assignment.log.base_path.display()
            ))
        })?;
    let (clock, clock_mock) = RobotClock::mock();
    let logger = Arc::new(Mutex::new(NoopLogger::new()));
    let app = <App as CuDistributedReplayApplication<NoopSectionStorage, NoopLogger>>::build_distributed_replay(
        clock.clone(),
        logger,
        assignment.instance_id,
        Some(config),
    )?;
    let mut session = RecordedReplaySession::<App, App::RecordedDataSet>::from_log(
        assignment.clone(),
        app,
        clock,
        clock_mock,
        &assignment.log.base_path,
    )?;
    let nodes = session.describe_nodes()?;
    Ok(DistributedReplaySessionBuild {
        session: Box::new(session),
        nodes,
    })
}

fn copperlist_origins<P: CopperListTuple>(
    copperlist: &CopperList<P>,
) -> BTreeSet<DistributedReplayOriginKey> {
    <CopperList<P> as ErasedCuStampedDataSet>::cumsgs(copperlist)
        .into_iter()
        .filter_map(|msg| msg.metadata().origin())
        .map(|origin| DistributedReplayOriginKey {
            instance_id: origin.instance_id,
            subsystem_code: origin.subsystem_code,
            cl_id: origin.cl_id,
        })
        .collect()
}

#[derive(Default)]
struct DistributedReplayEngineState {
    sessions: Vec<Box<dyn DistributedReplaySession>>,
    nodes: Vec<DistributedReplayGraphNode>,
    node_lookup: BTreeMap<(u32, String, u64), usize>,
    ready: BTreeSet<DistributedReplayReadyNode>,
    frontier: Vec<Option<DistributedReplayCursor>>,
}

/// One causal distributed replay engine built from a validated plan.
pub struct DistributedReplayEngine {
    plan: DistributedReplayPlan,
    sessions: Vec<Box<dyn DistributedReplaySession>>,
    nodes: Vec<DistributedReplayGraphNode>,
    node_lookup: BTreeMap<(u32, String, u64), usize>,
    ready: BTreeSet<DistributedReplayReadyNode>,
    frontier: Vec<Option<DistributedReplayCursor>>,
    executed: Vec<bool>,
    executed_count: usize,
}

impl DistributedReplayEngine {
    fn new(plan: DistributedReplayPlan) -> CuResult<Self> {
        let state = Self::build_state(&plan)?;
        let executed = vec![false; state.nodes.len()];
        Ok(Self {
            plan,
            sessions: state.sessions,
            nodes: state.nodes,
            node_lookup: state.node_lookup,
            ready: state.ready,
            frontier: state.frontier,
            executed,
            executed_count: 0,
        })
    }

    fn build_state(plan: &DistributedReplayPlan) -> CuResult<DistributedReplayEngineState> {
        let mut sessions = Vec::with_capacity(plan.assignments.len());
        let mut pending_nodes = Vec::new();
        let mut session_nodes = Vec::with_capacity(plan.assignments.len());

        for assignment in &plan.assignments {
            let build = (assignment.registration.session_factory)(assignment)?;
            let session_index = sessions.len();
            let mut node_indices = Vec::with_capacity(build.nodes.len());
            for node in build.nodes {
                let pending_index = pending_nodes.len();
                pending_nodes.push((session_index, node));
                node_indices.push(pending_index);
            }
            sessions.push(build.session);
            session_nodes.push(node_indices);
        }

        let mut nodes = Vec::with_capacity(pending_nodes.len());
        let mut origin_lookup = BTreeMap::new();
        let mut node_lookup = BTreeMap::new();

        for (node_index, (session_index, descriptor)) in pending_nodes.iter().enumerate() {
            if origin_lookup
                .insert(descriptor.origin_key.clone(), node_index)
                .is_some()
            {
                return Err(CuError::from(format!(
                    "Duplicate replay node detected for instance {} subsystem code {} CopperList {}",
                    descriptor.origin_key.instance_id,
                    descriptor.origin_key.subsystem_code,
                    descriptor.origin_key.cl_id
                )));
            }

            if node_lookup
                .insert(
                    (
                        descriptor.cursor.instance_id,
                        descriptor.cursor.subsystem_id.clone(),
                        descriptor.cursor.cl_id,
                    ),
                    node_index,
                )
                .is_some()
            {
                return Err(CuError::from(format!(
                    "Duplicate replay cursor detected for instance {} subsystem '{}' CopperList {}",
                    descriptor.cursor.instance_id,
                    descriptor.cursor.subsystem_id,
                    descriptor.cursor.cl_id
                )));
            }

            nodes.push(DistributedReplayGraphNode {
                cursor: descriptor.cursor.clone(),
                session_index: *session_index,
                outgoing: Vec::new(),
                initial_dependencies: 0,
                remaining_dependencies: 0,
            });
        }

        let mut edges = BTreeSet::new();

        for node_indices in &session_nodes {
            for pair in node_indices.windows(2) {
                let from = pair[0];
                let to = pair[1];
                if edges.insert((from, to)) {
                    nodes[from].outgoing.push(to);
                    nodes[to].initial_dependencies += 1;
                }
            }
        }

        for (target_index, (_, descriptor)) in pending_nodes.iter().enumerate() {
            for origin in &descriptor.incoming_origins {
                let source_index = origin_lookup.get(origin).copied().ok_or_else(|| {
                    CuError::from(format!(
                        "Unresolved recorded provenance edge into instance {} subsystem '{}' CopperList {} from instance {} subsystem code {} CopperList {}",
                        descriptor.cursor.instance_id,
                        descriptor.cursor.subsystem_id,
                        descriptor.cursor.cl_id,
                        origin.instance_id,
                        origin.subsystem_code,
                        origin.cl_id
                    ))
                })?;
                if source_index == target_index {
                    return Err(CuError::from(format!(
                        "Recorded provenance on instance {} subsystem '{}' CopperList {} points to itself",
                        descriptor.cursor.instance_id,
                        descriptor.cursor.subsystem_id,
                        descriptor.cursor.cl_id
                    )));
                }
                if edges.insert((source_index, target_index)) {
                    nodes[source_index].outgoing.push(target_index);
                    nodes[target_index].initial_dependencies += 1;
                }
            }
        }

        let mut ready = BTreeSet::new();
        for (node_index, node) in nodes.iter_mut().enumerate() {
            node.remaining_dependencies = node.initial_dependencies;
            if node.remaining_dependencies == 0 {
                ready.insert(DistributedReplayReadyNode {
                    instance_id: node.cursor.instance_id,
                    subsystem_code: node.cursor.subsystem_code(),
                    cl_id: node.cursor.cl_id,
                    node_index,
                });
            }
        }

        if !nodes.is_empty() && ready.is_empty() {
            return Err(CuError::from(
                "Distributed replay graph has no causally ready starting point",
            ));
        }

        Ok(DistributedReplayEngineState {
            frontier: vec![None; sessions.len()],
            sessions,
            nodes,
            node_lookup,
            ready,
        })
    }

    fn shutdown_sessions(sessions: &mut Vec<Box<dyn DistributedReplaySession>>) -> CuResult<()> {
        for session in sessions.iter_mut() {
            session.shutdown()?;
        }
        Ok(())
    }

    fn ready_key(&self, node_index: usize) -> DistributedReplayReadyNode {
        let node = &self.nodes[node_index];
        DistributedReplayReadyNode {
            instance_id: node.cursor.instance_id,
            subsystem_code: node.cursor.subsystem_code(),
            cl_id: node.cursor.cl_id,
            node_index,
        }
    }

    /// Reset all replay sessions and graph execution state back to the beginning.
    pub fn reset(&mut self) -> CuResult<()> {
        Self::shutdown_sessions(&mut self.sessions)?;
        let state = Self::build_state(&self.plan)?;
        self.sessions = state.sessions;
        self.nodes = state.nodes;
        self.node_lookup = state.node_lookup;
        self.ready = state.ready;
        self.frontier = state.frontier;
        self.executed = vec![false; self.nodes.len()];
        self.executed_count = 0;
        Ok(())
    }

    /// Replay the next causally ready CopperList, if any.
    pub fn step_causal(&mut self) -> CuResult<Option<DistributedReplayCursor>> {
        let Some(next_ready) = self.ready.iter().next().copied() else {
            if self.executed_count == self.nodes.len() {
                return Ok(None);
            }
            return Err(CuError::from(
                "Distributed replay is deadlocked: no causally ready CopperList remains",
            ));
        };
        self.ready.remove(&next_ready);

        let cursor = self.nodes[next_ready.node_index].cursor.clone();
        let session_index = self.nodes[next_ready.node_index].session_index;
        self.sessions[session_index].goto_cl(cursor.cl_id)?;
        self.executed[next_ready.node_index] = true;
        self.executed_count += 1;
        self.frontier[session_index] = Some(cursor.clone());

        let outgoing = self.nodes[next_ready.node_index].outgoing.clone();
        for dependent in outgoing {
            let node = &mut self.nodes[dependent];
            node.remaining_dependencies = node.remaining_dependencies.saturating_sub(1);
            if node.remaining_dependencies == 0 {
                self.ready.insert(self.ready_key(dependent));
            }
        }

        Ok(Some(cursor))
    }

    /// Replay the entire selected fleet to completion.
    pub fn run_all(&mut self) -> CuResult<()> {
        while self.step_causal()?.is_some() {}
        Ok(())
    }

    /// Rebuild the replay from scratch and advance until the target CopperList is reached.
    pub fn goto(&mut self, instance_id: u32, subsystem_id: &str, cl_id: u64) -> CuResult<()> {
        let target = self
            .node_lookup
            .get(&(instance_id, subsystem_id.to_string(), cl_id))
            .copied()
            .ok_or_else(|| {
                CuError::from(format!(
                    "Distributed replay target instance {} subsystem '{}' CopperList {} does not exist",
                    instance_id, subsystem_id, cl_id
                ))
            })?;
        self.reset()?;
        while !self.executed[target] {
            let Some(_) = self.step_causal()? else {
                return Err(CuError::from(format!(
                    "Distributed replay exhausted before reaching instance {} subsystem '{}' CopperList {}",
                    instance_id, subsystem_id, cl_id
                )));
            };
        }
        Ok(())
    }

    /// Return the latest executed CopperList cursor for each replay session.
    pub fn current_frontier(&self) -> Vec<DistributedReplayCursor> {
        self.frontier
            .iter()
            .filter_map(|cursor| cursor.clone())
            .collect()
    }

    #[inline]
    pub fn total_nodes(&self) -> usize {
        self.nodes.len()
    }

    #[inline]
    pub fn executed_nodes(&self) -> usize {
        self.executed_count
    }
}

fn join_log_paths(logs: &[DistributedReplayLog]) -> String {
    logs.iter()
        .map(|log| log.base_path.display().to_string())
        .collect::<Vec<_>>()
        .join(", ")
}

fn collect_candidate_base_paths(path: &Path, out: &mut BTreeSet<PathBuf>) -> CuResult<()> {
    if path.is_dir() {
        let mut entries = fs::read_dir(path)
            .map_err(|err| {
                CuError::new_with_cause(
                    &format!(
                        "Failed to read directory '{}' during distributed replay discovery",
                        path.display()
                    ),
                    err,
                )
            })?
            .collect::<Result<Vec<_>, _>>()
            .map_err(|err| {
                CuError::new_with_cause(
                    &format!(
                        "Failed to enumerate directory '{}' during distributed replay discovery",
                        path.display()
                    ),
                    err,
                )
            })?;
        entries.sort_by_key(|entry| entry.path());
        for entry in entries {
            collect_candidate_base_paths(&entry.path(), out)?;
        }
        return Ok(());
    }

    if path
        .extension()
        .and_then(|ext| ext.to_str())
        .is_some_and(|ext| ext == "copper")
    {
        out.insert(normalize_candidate_log_base(path));
    }

    Ok(())
}

fn normalize_candidate_log_base(path: &Path) -> PathBuf {
    let Some(extension) = path.extension().and_then(|ext| ext.to_str()) else {
        return path.to_path_buf();
    };
    let Some(stem) = path.file_stem().and_then(|stem| stem.to_str()) else {
        return path.to_path_buf();
    };
    let Some((base_stem, slab_suffix)) = stem.rsplit_once('_') else {
        return path.to_path_buf();
    };

    if slab_suffix.is_empty() || !slab_suffix.chars().all(|c| c.is_ascii_digit()) {
        return path.to_path_buf();
    }

    let mut normalized = path.to_path_buf();
    normalized.set_file_name(format!("{base_stem}.{extension}"));
    if slab_zero_path(&normalized).is_some_and(|slab_zero| slab_zero.exists()) {
        normalized
    } else {
        path.to_path_buf()
    }
}

fn slab_zero_path(base_path: &Path) -> Option<PathBuf> {
    let extension = base_path.extension()?.to_str()?;
    let stem = base_path.file_stem()?.to_str()?;
    let mut slab_zero = base_path.to_path_buf();
    slab_zero.set_file_name(format!("{stem}_0.{extension}"));
    Some(slab_zero)
}

fn read_next_entry<T: bincode::Decode<()>>(src: &mut impl Read) -> CuResult<Option<T>> {
    match decode_from_std_read::<T, _, _>(src, standard()) {
        Ok(entry) => Ok(Some(entry)),
        Err(DecodeError::UnexpectedEnd { .. }) => Ok(None),
        Err(DecodeError::Io { inner, .. }) if inner.kind() == std::io::ErrorKind::UnexpectedEof => {
            Ok(None)
        }
        Err(err) => Err(CuError::new_with_cause(
            "Failed to decode bincode entry during distributed replay discovery",
            err,
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::{
        CuDistributedReplayApplication, CuRecordedReplayApplication, CuSimApplication,
        CuSubsystemMetadata,
    };
    use crate::config::CuConfig;
    use crate::copperlist::CopperList;
    use crate::curuntime::KeyFrame;
    use crate::simulation::SimOverride;
    use bincode::{Decode, Encode};
    use cu29_clock::CuTime;
    use cu29_traits::{ErasedCuStampedData, ErasedCuStampedDataSet, MatchingTasks, WriteStream};
    use cu29_unifiedlog::memmap::MmapSectionStorage;
    use cu29_unifiedlog::{NoopLogger, NoopSectionStorage, stream_write};
    use serde::Serialize;
    use std::sync::{Arc, Mutex};
    use tempfile::TempDir;

    fn write_runtime_lifecycle_log(
        base_path: &Path,
        stack: RuntimeLifecycleStackInfo,
        mission: Option<&str>,
    ) -> CuResult<()> {
        if let Some(parent) = base_path.parent() {
            fs::create_dir_all(parent).map_err(|err| {
                CuError::new_with_cause(
                    &format!("Failed to create test log directory '{}'", parent.display()),
                    err,
                )
            })?;
        }

        let UnifiedLogger::Write(writer) = UnifiedLoggerBuilder::new()
            .write(true)
            .create(true)
            .preallocated_size(256 * 1024)
            .file_base_name(base_path)
            .build()
            .map_err(|err| {
                CuError::new_with_cause(
                    &format!("Failed to create test log '{}'", base_path.display()),
                    err,
                )
            })?
        else {
            return Err(CuError::from("Expected writable unified logger in test"));
        };

        let logger = Arc::new(Mutex::new(writer));
        let mut stream = stream_write::<RuntimeLifecycleRecord, MmapSectionStorage>(
            logger.clone(),
            UnifiedLogType::RuntimeLifecycle,
            4096,
        )?;
        stream.log(&RuntimeLifecycleRecord {
            timestamp: CuTime::default(),
            event: RuntimeLifecycleEvent::Instantiated {
                config_source: RuntimeLifecycleConfigSource::ExternalFile,
                effective_config_ron: "(runtime: ())".to_string(),
                stack,
            },
        })?;
        if let Some(mission) = mission {
            stream.log(&RuntimeLifecycleRecord {
                timestamp: CuTime::from_nanos(1),
                event: RuntimeLifecycleEvent::MissionStarted {
                    mission: mission.to_string(),
                },
            })?;
        }
        drop(stream);
        drop(logger);
        Ok(())
    }

    fn test_stack(
        subsystem_id: &str,
        subsystem_code: u16,
        instance_id: u32,
    ) -> RuntimeLifecycleStackInfo {
        RuntimeLifecycleStackInfo {
            app_name: "demo".to_string(),
            app_version: "0.1.0".to_string(),
            git_commit: Some("abc123".to_string()),
            git_dirty: Some(false),
            subsystem_id: Some(subsystem_id.to_string()),
            subsystem_code,
            instance_id,
        }
    }

    fn write_multi_config_fixture(temp_dir: &TempDir, subsystem_ids: &[&str]) -> CuResult<PathBuf> {
        for subsystem_id in subsystem_ids {
            let subsystem_config = temp_dir.path().join(format!("{subsystem_id}_config.ron"));
            fs::write(&subsystem_config, "(tasks: [], cnx: [])").map_err(|err| {
                CuError::new_with_cause(
                    &format!(
                        "Failed to write subsystem config '{}'",
                        subsystem_config.display()
                    ),
                    err,
                )
            })?;
        }

        let subsystem_entries = subsystem_ids
            .iter()
            .map(|subsystem_id| {
                format!(
                    r#"(
            id: "{subsystem_id}",
            config: "{subsystem_id}_config.ron",
        )"#
                )
            })
            .collect::<Vec<_>>()
            .join(",\n");

        let multi_config = format!(
            "(\n    subsystems: [\n{entries}\n    ],\n    interconnects: [],\n)\n",
            entries = subsystem_entries
        );
        let multi_config_path = temp_dir.path().join("multi_copper.ron");
        fs::write(&multi_config_path, multi_config).map_err(|err| {
            CuError::new_with_cause(
                &format!(
                    "Failed to write multi-Copper config '{}'",
                    multi_config_path.display()
                ),
                err,
            )
        })?;
        Ok(multi_config_path)
    }

    #[derive(Debug, Default, Encode, Decode, Serialize)]
    struct DummyRecordedDataSet;

    impl ErasedCuStampedDataSet for DummyRecordedDataSet {
        fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
            Vec::new()
        }
    }

    impl MatchingTasks for DummyRecordedDataSet {
        fn get_all_task_ids() -> &'static [&'static str] {
            &[]
        }
    }

    macro_rules! impl_registered_test_app {
        ($name:ident, $subsystem_id:expr, $subsystem_code:expr) => {
            struct $name;

            impl CuSubsystemMetadata for $name {
                fn subsystem() -> Subsystem {
                    Subsystem::new(Some($subsystem_id), $subsystem_code)
                }
            }

            impl CuSimApplication<NoopSectionStorage, NoopLogger> for $name {
                type Step<'z> = ();

                fn get_original_config() -> String {
                    "(tasks: [], cnx: [])".to_string()
                }

                fn new(
                    _clock: RobotClock,
                    _unified_logger: Arc<Mutex<NoopLogger>>,
                    _config_override: Option<CuConfig>,
                    _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
                ) -> CuResult<Self> {
                    Ok(Self)
                }

                fn start_all_tasks(
                    &mut self,
                    _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
                ) -> CuResult<()> {
                    Ok(())
                }

                fn run_one_iteration(
                    &mut self,
                    _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
                ) -> CuResult<()> {
                    Ok(())
                }

                fn run(
                    &mut self,
                    _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
                ) -> CuResult<()> {
                    Ok(())
                }

                fn stop_all_tasks(
                    &mut self,
                    _sim_callback: &mut impl for<'z> FnMut(Self::Step<'z>) -> SimOverride,
                ) -> CuResult<()> {
                    Ok(())
                }

                fn restore_keyframe(&mut self, _freezer: &KeyFrame) -> CuResult<()> {
                    Ok(())
                }
            }

            impl CuRecordedReplayApplication<NoopSectionStorage, NoopLogger> for $name {
                type RecordedDataSet = DummyRecordedDataSet;

                fn replay_recorded_copperlist(
                    &mut self,
                    _clock_mock: &RobotClockMock,
                    _copperlist: &CopperList<Self::RecordedDataSet>,
                    _keyframe: Option<&KeyFrame>,
                ) -> CuResult<()> {
                    Ok(())
                }
            }

            impl CuDistributedReplayApplication<NoopSectionStorage, NoopLogger> for $name {
                fn build_distributed_replay(
                    clock: RobotClock,
                    unified_logger: Arc<Mutex<NoopLogger>>,
                    _instance_id: u32,
                    config_override: Option<CuConfig>,
                ) -> CuResult<Self> {
                    let mut noop = |_step: ()| SimOverride::ExecuteByRuntime;
                    <Self as CuSimApplication<NoopSectionStorage, NoopLogger>>::new(
                        clock,
                        unified_logger,
                        config_override,
                        &mut noop,
                    )
                }
            }
        };
    }

    impl_registered_test_app!(PingRegisteredApp, "ping", 0);
    impl_registered_test_app!(PongRegisteredApp, "pong", 1);
    impl_registered_test_app!(PingWrongCodeApp, "ping", 99);

    struct FakeReplaySession;

    impl DistributedReplaySession for FakeReplaySession {
        fn goto_cl(&mut self, _cl_id: u64) -> CuResult<()> {
            Ok(())
        }

        fn shutdown(&mut self) -> CuResult<()> {
            Ok(())
        }
    }

    fn fake_registration(
        subsystem_id: &'static str,
        subsystem_code: u16,
        session_factory: DistributedReplaySessionFactory,
    ) -> DistributedReplayAppRegistration {
        DistributedReplayAppRegistration {
            subsystem: Subsystem::new(Some(subsystem_id), subsystem_code),
            app_type_name: "fake",
            session_factory,
        }
    }

    fn fake_assignment(
        instance_id: u32,
        subsystem_id: &'static str,
        subsystem_code: u16,
        session_factory: DistributedReplaySessionFactory,
    ) -> DistributedReplayAssignment {
        DistributedReplayAssignment {
            instance_id,
            subsystem_id: subsystem_id.to_string(),
            log: DistributedReplayLog {
                base_path: PathBuf::from(format!("{subsystem_id}_{instance_id}.copper")),
                stack: test_stack(subsystem_id, subsystem_code, instance_id),
                config_source: RuntimeLifecycleConfigSource::ExternalFile,
                effective_config_ron: "(tasks: [], cnx: [])".to_string(),
                mission: Some("default".to_string()),
            },
            registration: fake_registration(subsystem_id, subsystem_code, session_factory),
        }
    }

    fn fake_plan(assignments: Vec<DistributedReplayAssignment>) -> DistributedReplayPlan {
        let mut registrations: Vec<_> = assignments
            .iter()
            .map(|assignment| assignment.registration.clone())
            .collect();
        registrations.sort_by(|left, right| left.subsystem.id().cmp(&right.subsystem.id()));
        DistributedReplayPlan {
            multi_config_path: PathBuf::from("fake_multi.ron"),
            multi_config: MultiCopperConfig {
                subsystems: Vec::new(),
                interconnects: Vec::new(),
            },
            catalog: DistributedReplayCatalog::default(),
            selected_instances: vec![1],
            mission: Some("default".to_string()),
            registrations,
            assignments,
        }
    }

    fn fake_ping_session(
        assignment: &DistributedReplayAssignment,
    ) -> CuResult<DistributedReplaySessionBuild> {
        Ok(DistributedReplaySessionBuild {
            session: Box::new(FakeReplaySession),
            nodes: vec![
                DistributedReplayNodeDescriptor {
                    cursor: DistributedReplayCursor::new(
                        assignment.instance_id,
                        assignment.subsystem_id.clone(),
                        assignment.log.subsystem_code(),
                        0,
                    ),
                    origin_key: DistributedReplayOriginKey {
                        instance_id: assignment.instance_id,
                        subsystem_code: assignment.log.subsystem_code(),
                        cl_id: 0,
                    },
                    incoming_origins: BTreeSet::new(),
                },
                DistributedReplayNodeDescriptor {
                    cursor: DistributedReplayCursor::new(
                        assignment.instance_id,
                        assignment.subsystem_id.clone(),
                        assignment.log.subsystem_code(),
                        1,
                    ),
                    origin_key: DistributedReplayOriginKey {
                        instance_id: assignment.instance_id,
                        subsystem_code: assignment.log.subsystem_code(),
                        cl_id: 1,
                    },
                    incoming_origins: BTreeSet::new(),
                },
            ],
        })
    }

    fn fake_pong_session(
        assignment: &DistributedReplayAssignment,
    ) -> CuResult<DistributedReplaySessionBuild> {
        Ok(DistributedReplaySessionBuild {
            session: Box::new(FakeReplaySession),
            nodes: vec![
                DistributedReplayNodeDescriptor {
                    cursor: DistributedReplayCursor::new(
                        assignment.instance_id,
                        assignment.subsystem_id.clone(),
                        assignment.log.subsystem_code(),
                        0,
                    ),
                    origin_key: DistributedReplayOriginKey {
                        instance_id: assignment.instance_id,
                        subsystem_code: assignment.log.subsystem_code(),
                        cl_id: 0,
                    },
                    incoming_origins: BTreeSet::from([DistributedReplayOriginKey {
                        instance_id: assignment.instance_id,
                        subsystem_code: 0,
                        cl_id: 0,
                    }]),
                },
                DistributedReplayNodeDescriptor {
                    cursor: DistributedReplayCursor::new(
                        assignment.instance_id,
                        assignment.subsystem_id.clone(),
                        assignment.log.subsystem_code(),
                        1,
                    ),
                    origin_key: DistributedReplayOriginKey {
                        instance_id: assignment.instance_id,
                        subsystem_code: assignment.log.subsystem_code(),
                        cl_id: 1,
                    },
                    incoming_origins: BTreeSet::from([DistributedReplayOriginKey {
                        instance_id: assignment.instance_id,
                        subsystem_code: 0,
                        cl_id: 1,
                    }]),
                },
            ],
        })
    }

    fn fake_bad_pong_session(
        assignment: &DistributedReplayAssignment,
    ) -> CuResult<DistributedReplaySessionBuild> {
        Ok(DistributedReplaySessionBuild {
            session: Box::new(FakeReplaySession),
            nodes: vec![DistributedReplayNodeDescriptor {
                cursor: DistributedReplayCursor::new(
                    assignment.instance_id,
                    assignment.subsystem_id.clone(),
                    assignment.log.subsystem_code(),
                    0,
                ),
                origin_key: DistributedReplayOriginKey {
                    instance_id: assignment.instance_id,
                    subsystem_code: assignment.log.subsystem_code(),
                    cl_id: 0,
                },
                incoming_origins: BTreeSet::from([DistributedReplayOriginKey {
                    instance_id: assignment.instance_id,
                    subsystem_code: 0,
                    cl_id: 99,
                }]),
            }],
        })
    }

    #[test]
    fn discovers_single_log_identity_from_runtime_lifecycle() -> CuResult<()> {
        let temp_dir = TempDir::new()
            .map_err(|err| CuError::new_with_cause("Failed to create temp dir", err))?;
        let base_path = temp_dir.path().join("logs/ping.copper");
        write_runtime_lifecycle_log(&base_path, test_stack("ping", 7, 42), Some("default"))?;

        let discovered = DistributedReplayLog::discover(&base_path)?;
        assert_eq!(discovered.base_path, base_path);
        assert_eq!(discovered.subsystem_id(), Some("ping"));
        assert_eq!(discovered.subsystem_code(), 7);
        assert_eq!(discovered.instance_id(), 42);
        assert_eq!(discovered.mission.as_deref(), Some("default"));
        Ok(())
    }

    #[test]
    fn catalog_discovery_normalizes_slab_paths_and_deduplicates_candidates() -> CuResult<()> {
        let temp_dir = TempDir::new()
            .map_err(|err| CuError::new_with_cause("Failed to create temp dir", err))?;
        let base_path = temp_dir.path().join("logs/pong.copper");
        let slab_zero_path = temp_dir.path().join("logs/pong_0.copper");
        write_runtime_lifecycle_log(&base_path, test_stack("pong", 3, 9), Some("default"))?;

        let catalog = DistributedReplayCatalog::discover([base_path.clone(), slab_zero_path])?;
        assert!(
            catalog.failures.is_empty(),
            "unexpected failures: {:?}",
            catalog.failures
        );
        assert_eq!(catalog.logs.len(), 1);
        assert_eq!(catalog.logs[0].base_path, base_path);
        assert_eq!(catalog.logs[0].subsystem_id(), Some("pong"));
        Ok(())
    }

    #[test]
    fn catalog_discovery_walks_directories_using_physical_slab_files() -> CuResult<()> {
        let temp_dir = TempDir::new()
            .map_err(|err| CuError::new_with_cause("Failed to create temp dir", err))?;
        let ping_base = temp_dir.path().join("logs/ping.copper");
        let pong_base = temp_dir.path().join("logs/pong.copper");
        write_runtime_lifecycle_log(&ping_base, test_stack("ping", 0, 1), Some("alpha"))?;
        write_runtime_lifecycle_log(&pong_base, test_stack("pong", 1, 1), Some("alpha"))?;

        let catalog = DistributedReplayCatalog::discover_under(temp_dir.path())?;
        assert!(
            catalog.failures.is_empty(),
            "unexpected failures: {:?}",
            catalog.failures
        );
        assert_eq!(catalog.logs.len(), 2);
        assert_eq!(catalog.logs[0].subsystem_id(), Some("ping"));
        assert_eq!(catalog.logs[1].subsystem_id(), Some("pong"));
        assert_eq!(catalog.logs[0].base_path, ping_base);
        assert_eq!(catalog.logs[1].base_path, pong_base);
        Ok(())
    }

    #[test]
    fn catalog_reports_invalid_logs_without_aborting_scan() -> CuResult<()> {
        let temp_dir = TempDir::new()
            .map_err(|err| CuError::new_with_cause("Failed to create temp dir", err))?;
        let good_base = temp_dir.path().join("logs/good.copper");
        write_runtime_lifecycle_log(&good_base, test_stack("good", 2, 5), Some("beta"))?;

        let bad_slab = temp_dir.path().join("logs/bad_0.copper");
        if let Some(parent) = bad_slab.parent() {
            fs::create_dir_all(parent).map_err(|err| {
                CuError::new_with_cause(
                    &format!("Failed to create bad log dir '{}'", parent.display()),
                    err,
                )
            })?;
        }
        fs::write(&bad_slab, b"not a copper log").map_err(|err| {
            CuError::new_with_cause(
                &format!("Failed to create bad log '{}'", bad_slab.display()),
                err,
            )
        })?;

        let catalog = DistributedReplayCatalog::discover_under(temp_dir.path())?;
        assert_eq!(catalog.logs.len(), 1);
        assert_eq!(catalog.failures.len(), 1);
        assert_eq!(catalog.logs[0].subsystem_id(), Some("good"));
        assert_eq!(
            catalog.failures[0].candidate_path,
            temp_dir.path().join("logs/bad.copper")
        );
        Ok(())
    }

    #[test]
    fn builder_builds_validated_plan_for_selected_instances() -> CuResult<()> {
        let temp_dir = TempDir::new()
            .map_err(|err| CuError::new_with_cause("Failed to create temp dir", err))?;
        let multi_config_path = write_multi_config_fixture(&temp_dir, &["ping", "pong"])?;
        let logs_root = temp_dir.path().join("logs");

        write_runtime_lifecycle_log(
            &logs_root.join("instance1_ping.copper"),
            test_stack("ping", 0, 1),
            Some("default"),
        )?;
        write_runtime_lifecycle_log(
            &logs_root.join("instance1_pong.copper"),
            test_stack("pong", 1, 1),
            Some("default"),
        )?;
        write_runtime_lifecycle_log(
            &logs_root.join("instance2_ping.copper"),
            test_stack("ping", 0, 2),
            Some("default"),
        )?;
        write_runtime_lifecycle_log(
            &logs_root.join("instance2_pong.copper"),
            test_stack("pong", 1, 2),
            Some("default"),
        )?;

        let plan = DistributedReplayPlan::builder(&multi_config_path)?
            .discover_logs_under(&logs_root)?
            .register::<PingRegisteredApp>("ping")?
            .register::<PongRegisteredApp>("pong")?
            .instances([2])
            .build()?;

        assert_eq!(plan.selected_instances, vec![2]);
        assert_eq!(plan.mission.as_deref(), Some("default"));
        assert_eq!(plan.assignments.len(), 2);
        assert_eq!(
            plan.assignment(2, "ping").unwrap().log.base_path,
            logs_root.join("instance2_ping.copper")
        );
        assert_eq!(
            plan.assignment(2, "pong").unwrap().log.base_path,
            logs_root.join("instance2_pong.copper")
        );
        Ok(())
    }

    #[test]
    fn register_rejects_subsystem_code_mismatch() -> CuResult<()> {
        let temp_dir = TempDir::new()
            .map_err(|err| CuError::new_with_cause("Failed to create temp dir", err))?;
        let multi_config_path = write_multi_config_fixture(&temp_dir, &["ping", "pong"])?;

        let err = DistributedReplayPlan::builder(&multi_config_path)?
            .register::<PingWrongCodeApp>("ping")
            .unwrap_err();
        assert!(err.to_string().contains("declares subsystem code 99"));
        Ok(())
    }

    #[test]
    fn build_reports_missing_logs_and_missing_registrations() -> CuResult<()> {
        let temp_dir = TempDir::new()
            .map_err(|err| CuError::new_with_cause("Failed to create temp dir", err))?;
        let multi_config_path = write_multi_config_fixture(&temp_dir, &["ping", "pong"])?;
        let logs_root = temp_dir.path().join("logs");

        write_runtime_lifecycle_log(
            &logs_root.join("instance1_ping.copper"),
            test_stack("ping", 0, 1),
            Some("default"),
        )?;

        let err = DistributedReplayPlan::builder(&multi_config_path)?
            .discover_logs_under(&logs_root)?
            .register::<PingRegisteredApp>("ping")?
            .build()
            .unwrap_err();
        let err_text = err.to_string();
        assert!(err_text.contains("missing app registration for subsystem 'pong'"));
        assert!(err_text.contains("missing log for instance 1 subsystem 'pong'"));
        Ok(())
    }

    #[test]
    fn build_reports_duplicate_logs_for_one_target() -> CuResult<()> {
        let temp_dir = TempDir::new()
            .map_err(|err| CuError::new_with_cause("Failed to create temp dir", err))?;
        let multi_config_path = write_multi_config_fixture(&temp_dir, &["ping", "pong"])?;
        let logs_root = temp_dir.path().join("logs");

        write_runtime_lifecycle_log(
            &logs_root.join("instance1_ping_a.copper"),
            test_stack("ping", 0, 1),
            Some("default"),
        )?;
        write_runtime_lifecycle_log(
            &logs_root.join("instance1_ping_b.copper"),
            test_stack("ping", 0, 1),
            Some("default"),
        )?;
        write_runtime_lifecycle_log(
            &logs_root.join("instance1_pong.copper"),
            test_stack("pong", 1, 1),
            Some("default"),
        )?;

        let err = DistributedReplayPlan::builder(&multi_config_path)?
            .discover_logs_under(&logs_root)?
            .register::<PingRegisteredApp>("ping")?
            .register::<PongRegisteredApp>("pong")?
            .build()
            .unwrap_err();
        assert!(
            err.to_string()
                .contains("found 2 logs for instance 1 subsystem 'ping'")
        );
        Ok(())
    }

    #[test]
    fn build_reports_mission_mismatch_across_selected_logs() -> CuResult<()> {
        let temp_dir = TempDir::new()
            .map_err(|err| CuError::new_with_cause("Failed to create temp dir", err))?;
        let multi_config_path = write_multi_config_fixture(&temp_dir, &["ping", "pong"])?;
        let logs_root = temp_dir.path().join("logs");

        write_runtime_lifecycle_log(
            &logs_root.join("instance1_ping.copper"),
            test_stack("ping", 0, 1),
            Some("default"),
        )?;
        write_runtime_lifecycle_log(
            &logs_root.join("instance1_pong.copper"),
            test_stack("pong", 1, 1),
            Some("recovery"),
        )?;

        let err = DistributedReplayPlan::builder(&multi_config_path)?
            .discover_logs_under(&logs_root)?
            .register::<PingRegisteredApp>("ping")?
            .register::<PongRegisteredApp>("pong")?
            .build()
            .unwrap_err();
        assert!(
            err.to_string()
                .contains("selected logs disagree on mission: default, recovery")
        );
        Ok(())
    }

    #[test]
    fn engine_steps_in_stable_causal_order() -> CuResult<()> {
        let plan = fake_plan(vec![
            fake_assignment(1, "ping", 0, fake_ping_session),
            fake_assignment(1, "pong", 1, fake_pong_session),
        ]);

        let mut engine = plan.start()?;
        let mut order = Vec::new();
        while let Some(cursor) = engine.step_causal()? {
            order.push((cursor.subsystem_id, cursor.cl_id));
        }

        assert_eq!(
            order,
            vec![
                ("ping".to_string(), 0),
                ("ping".to_string(), 1),
                ("pong".to_string(), 0),
                ("pong".to_string(), 1),
            ]
        );
        assert_eq!(engine.executed_nodes(), 4);
        Ok(())
    }

    #[test]
    fn engine_goto_rebuilds_and_replays_to_target() -> CuResult<()> {
        let plan = fake_plan(vec![
            fake_assignment(1, "ping", 0, fake_ping_session),
            fake_assignment(1, "pong", 1, fake_pong_session),
        ]);

        let mut engine = plan.start()?;
        engine.run_all()?;
        engine.goto(1, "pong", 0)?;

        assert_eq!(engine.executed_nodes(), 3);
        let frontier = engine.current_frontier();
        assert_eq!(frontier.len(), 2);
        assert!(frontier.iter().any(|cursor| {
            cursor.instance_id == 1 && cursor.subsystem_id == "ping" && cursor.cl_id == 1
        }));
        assert!(frontier.iter().any(|cursor| {
            cursor.instance_id == 1 && cursor.subsystem_id == "pong" && cursor.cl_id == 0
        }));
        Ok(())
    }

    #[test]
    fn engine_reports_unresolved_recorded_provenance() -> CuResult<()> {
        let plan = fake_plan(vec![
            fake_assignment(1, "ping", 0, fake_ping_session),
            fake_assignment(1, "pong", 1, fake_bad_pong_session),
        ]);

        let err = match plan.start() {
            Ok(_) => return Err(CuError::from("expected distributed replay startup failure")),
            Err(err) => err,
        };
        assert!(
            err.to_string()
                .contains("Unresolved recorded provenance edge")
        );
        Ok(())
    }
}
