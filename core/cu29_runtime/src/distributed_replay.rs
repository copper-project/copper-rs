//! Discovery, validation, and planning helpers for distributed deterministic replay.
//!
//! This module currently stops at a validated replay plan:
//! it scans Copper unified logs, normalizes slab paths back to their base
//! `.copper` path, extracts runtime identity from `RuntimeLifecycle::Instantiated`,
//! matches those logs against a strict multi-Copper topology, validates typed
//! subsystem registrations, and prepares a per-instance/per-subsystem plan.

use crate::app::{CuSubsystemMetadata, Subsystem};
use crate::config::{MultiCopperConfig, read_multi_configuration};
use crate::curuntime::{
    RuntimeLifecycleConfigSource, RuntimeLifecycleEvent, RuntimeLifecycleRecord,
    RuntimeLifecycleStackInfo,
};
use bincode::config::standard;
use bincode::decode_from_std_read;
use bincode::error::DecodeError;
use cu29_traits::{CuError, CuResult, UnifiedLogType};
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use std::any::type_name;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter, Result as FmtResult};
use std::fs;
use std::io::Read;
use std::path::{Path, PathBuf};

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

/// One typed subsystem registration provided to the distributed replay builder.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DistributedReplayAppRegistration {
    pub subsystem: Subsystem,
    pub app_type_name: &'static str,
}

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
        App: CuSubsystemMetadata + 'static,
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
    use crate::app::CuSubsystemMetadata;
    use cu29_clock::CuTime;
    use cu29_traits::WriteStream;
    use cu29_unifiedlog::memmap::MmapSectionStorage;
    use cu29_unifiedlog::stream_write;
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

    struct PingRegisteredApp;

    impl CuSubsystemMetadata for PingRegisteredApp {
        fn subsystem() -> Subsystem {
            Subsystem::new(Some("ping"), 0)
        }
    }

    struct PongRegisteredApp;

    impl CuSubsystemMetadata for PongRegisteredApp {
        fn subsystem() -> Subsystem {
            Subsystem::new(Some("pong"), 1)
        }
    }

    struct PingWrongCodeApp;

    impl CuSubsystemMetadata for PingWrongCodeApp {
        fn subsystem() -> Subsystem {
            Subsystem::new(Some("ping"), 99)
        }
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
}
