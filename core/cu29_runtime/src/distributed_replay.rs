//! Discovery/catalog helpers for distributed deterministic replay.
//!
//! This module is intentionally limited to offline log discovery for now:
//! it scans Copper unified logs, normalizes slab paths back to their base
//! `.copper` path, and extracts the runtime identity recorded in the
//! `RuntimeLifecycle::Instantiated` event.

use crate::curuntime::{
    RuntimeLifecycleConfigSource, RuntimeLifecycleEvent, RuntimeLifecycleRecord,
    RuntimeLifecycleStackInfo,
};
use bincode::config::standard;
use bincode::decode_from_std_read;
use bincode::error::DecodeError;
use cu29_traits::{CuError, CuResult, UnifiedLogType};
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use std::collections::BTreeSet;
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
}
