//! Shared Clap-backed helpers for replay and resimulation binaries.
//!
//! Use [`ReplayCli`] when the binary only needs the standard replay flags.
//! Use [`ReplayArgs`] with `#[command(flatten)]` when the binary has its own
//! app-specific CLI, such as an extra mission selector.

use clap::{Args, Parser};
use core::sync::atomic::{AtomicU64, Ordering};
use cu29_traits::{CuError, CuResult};
use std::ffi::OsString;
use std::fs;
use std::path::{Path, PathBuf};

#[cfg(feature = "remote-debug")]
use crate::app::{CuSimApplication, CurrentRuntimeCopperList};
#[cfg(feature = "remote-debug")]
use crate::copperlist::CopperList;
#[cfg(feature = "remote-debug")]
use crate::reflect::ReflectTaskIntrospection;
#[cfg(feature = "remote-debug")]
use crate::remote_debug::{RemoteDebugPaths, RemoteDebugZenohServer, SessionOpenParams};
#[cfg(feature = "remote-debug")]
use crate::simulation::SimOverride;
#[cfg(feature = "remote-debug")]
use cu29_clock::{CuTime, RobotClock, RobotClockMock};
#[cfg(feature = "remote-debug")]
use cu29_traits::CopperListTuple;
#[cfg(feature = "remote-debug")]
use cu29_unifiedlog::{SectionStorage, UnifiedLogWrite};

static NEXT_REPLAY_SESSION_ID: AtomicU64 = AtomicU64::new(0);

#[derive(Debug, Clone)]
pub struct ReplayDefaults {
    pub debug_base: Option<String>,
    pub log_base: PathBuf,
    pub replay_log_base: PathBuf,
}

impl ReplayDefaults {
    pub fn new(log_base: impl Into<PathBuf>, replay_log_base: impl Into<PathBuf>) -> Self {
        Self {
            debug_base: None,
            log_base: log_base.into(),
            replay_log_base: replay_log_base.into(),
        }
    }

    pub fn with_debug_base(mut self, debug_base: impl Into<String>) -> Self {
        self.debug_base = Some(debug_base.into());
        self
    }
}

#[derive(Debug, Clone, Args)]
pub struct ReplayArgs {
    /// Remote debug namespace base. When set, starts the remote debug server.
    #[arg(long, value_name = "PATH")]
    pub debug_base: Option<String>,

    /// Recorded Copper log base.
    #[arg(long, value_name = "PATH")]
    pub log_base: Option<PathBuf>,

    /// Replay log base or per-session template.
    #[arg(long, value_name = "PATH")]
    pub replay_log_base: Option<PathBuf>,
}

impl ReplayArgs {
    pub fn resolve(self, defaults: &ReplayDefaults) -> ReplayCli {
        ReplayCli {
            debug_base: self
                .debug_base
                .or_else(|| defaults.debug_base.as_ref().cloned()),
            log_base: self.log_base.unwrap_or_else(|| defaults.log_base.clone()),
            replay_log_base: self
                .replay_log_base
                .unwrap_or_else(|| defaults.replay_log_base.clone()),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ReplayCli {
    pub debug_base: Option<String>,
    pub log_base: PathBuf,
    pub replay_log_base: PathBuf,
}

#[derive(Debug, Clone, Parser)]
struct StandaloneReplayParser {
    #[command(flatten)]
    replay: ReplayArgs,
}

impl ReplayCli {
    pub fn parse(defaults: ReplayDefaults) -> Self {
        match Self::try_parse_from(std::env::args_os(), defaults) {
            Ok(cli) => cli,
            Err(err) => err.exit(),
        }
    }

    pub fn try_parse_from<I, T>(args: I, defaults: ReplayDefaults) -> Result<Self, clap::Error>
    where
        I: IntoIterator<Item = T>,
        T: Into<OsString> + Clone,
    {
        let mut argv: Vec<OsString> = args.into_iter().map(Into::into).collect();

        if let Some(debug_base) = defaults.debug_base.as_ref()
            && !has_long_flag(&argv, "--debug-base")
        {
            argv.push("--debug-base".into());
            argv.push(debug_base.clone().into());
        }

        if !has_long_flag(&argv, "--log-base") {
            argv.push("--log-base".into());
            argv.push(defaults.log_base.clone().into_os_string());
        }

        if !has_long_flag(&argv, "--replay-log-base") {
            argv.push("--replay-log-base".into());
            argv.push(defaults.replay_log_base.clone().into_os_string());
        }

        let parsed = StandaloneReplayParser::try_parse_from(argv)?;
        Ok(parsed.replay.resolve(&defaults))
    }
}

pub fn ensure_log_family_exists(log_base: &Path) -> CuResult<()> {
    if log_base.exists() {
        return Ok(());
    }

    let first_slab = first_slab_path(log_base)?;
    if first_slab.exists() {
        return Ok(());
    }

    Err(CuError::from(format!(
        "log family not found for {} (expected {})",
        log_base.display(),
        first_slab.display()
    )))
}

pub fn remove_log_family(log_base: &Path) -> CuResult<()> {
    let parent = log_base
        .parent()
        .ok_or_else(|| CuError::from("log path must have a parent directory"))?;
    fs::create_dir_all(parent)
        .map_err(|err| CuError::new_with_cause("failed to create replay log directory", err))?;

    let stem = log_base
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| CuError::from("invalid UTF-8 replay log stem"))?;
    let ext = log_base
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("copper");
    let family_prefix = format!("{stem}_");
    let family_ext = format!(".{ext}");
    let base_file_name = format!("{stem}.{ext}");

    let entries = fs::read_dir(parent)
        .map_err(|err| CuError::new_with_cause("failed to scan replay log directory", err))?;
    for entry_result in entries {
        let entry = entry_result
            .map_err(|err| CuError::new_with_cause("failed to read replay log entry", err))?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        let Some(name) = path.file_name().and_then(|s| s.to_str()) else {
            continue;
        };
        let in_family = name == base_file_name
            || (name.starts_with(&family_prefix) && name.ends_with(&family_ext));
        if in_family {
            fs::remove_file(&path)
                .map_err(|err| CuError::new_with_cause("failed to remove replay log file", err))?;
        }
    }

    Ok(())
}

pub fn first_slab_path(log_base: &Path) -> CuResult<PathBuf> {
    let parent = log_base
        .parent()
        .ok_or_else(|| CuError::from("log path must have a parent directory"))?;
    let stem = log_base
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| CuError::from("invalid UTF-8 log stem"))?;
    let ext = log_base
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("copper");
    Ok(parent.join(format!("{stem}_0.{ext}")))
}

pub fn per_session_replay_log_base<I, S>(template: &Path, labels: I) -> PathBuf
where
    I: IntoIterator<Item = S>,
    S: AsRef<str>,
{
    let seq = NEXT_REPLAY_SESSION_ID.fetch_add(1, Ordering::Relaxed);
    let parent = template.parent().unwrap_or_else(|| Path::new("."));
    let stem = template
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("replay");
    let ext = template
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("copper");

    let mut parts = Vec::new();
    for label in labels {
        let sanitized = sanitize_label(label.as_ref());
        if !sanitized.is_empty() {
            parts.push(sanitized);
        }
    }
    if parts.is_empty() {
        parts.push("session".to_owned());
    }

    parent.join(format!("{stem}_{}_{}.{}", parts.join("_"), seq, ext))
}

#[cfg(feature = "remote-debug")]
pub fn serve_remote_debug<App, P, CB, TF, S, L, AF>(
    debug_base: &str,
    log_base: &Path,
    app_factory: AF,
    build_callback: CB,
    time_of: TF,
) -> CuResult<()>
where
    App: CuSimApplication<S, L> + ReflectTaskIntrospection + CurrentRuntimeCopperList<P>,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple + 'static,
    CB: for<'a> Fn(
            &'a CopperList<P>,
            RobotClock,
            RobotClockMock,
        ) -> Box<dyn for<'z> FnMut(App::Step<'z>) -> SimOverride + 'a>
        + Clone,
    TF: Fn(&CopperList<P>) -> Option<CuTime> + Clone,
    AF: Fn(&SessionOpenParams) -> CuResult<(App, RobotClock, RobotClockMock)>,
{
    ensure_log_family_exists(log_base)?;
    let paths = RemoteDebugPaths::new(debug_base);
    let mut server = RemoteDebugZenohServer::<App, P, CB, TF, S, L, AF>::new(
        paths,
        app_factory,
        build_callback,
        time_of,
    )?;
    server.serve_until_stopped()
}

fn sanitize_label(label: &str) -> String {
    let mut sanitized = String::with_capacity(label.len());
    for ch in label.chars() {
        if ch.is_ascii_alphanumeric() {
            sanitized.push(ch.to_ascii_lowercase());
        } else {
            sanitized.push('_');
        }
    }
    sanitized.trim_matches('_').to_owned()
}

fn has_long_flag(args: &[OsString], flag: &str) -> bool {
    args.iter().any(|arg| {
        let rendered = arg.to_string_lossy();
        rendered == flag
            || rendered
                .strip_prefix(flag)
                .is_some_and(|suffix| suffix.starts_with('='))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replay_cli_applies_runtime_defaults() {
        let defaults = ReplayDefaults::new("logs/input.copper", "logs/output.copper")
            .with_debug_base("copper/test/debug/v1");
        let cli = ReplayCli::try_parse_from(["resim"], defaults).expect("parse replay CLI");
        assert_eq!(cli.debug_base.as_deref(), Some("copper/test/debug/v1"));
        assert_eq!(cli.log_base, PathBuf::from("logs/input.copper"));
        assert_eq!(cli.replay_log_base, PathBuf::from("logs/output.copper"));
    }

    #[test]
    fn per_session_replay_log_base_sanitizes_labels() {
        let path = per_session_replay_log_base(
            Path::new("logs/example_resim.copper"),
            ["Controller Session", "role/a"],
        );
        assert!(path.starts_with(Path::new("logs")));
        let file_name = path
            .file_name()
            .and_then(|name| name.to_str())
            .expect("UTF-8 replay log file name");
        assert!(file_name.starts_with("example_resim_controller_session_role_a_"));
        assert!(file_name.ends_with(".copper"));
    }

    #[test]
    fn replay_cli_respects_equals_style_flags() {
        let defaults = ReplayDefaults::new("logs/input.copper", "logs/output.copper")
            .with_debug_base("copper/test/debug/v1");
        let cli = ReplayCli::try_parse_from(
            [
                "resim",
                "--log-base=logs/override.copper",
                "--replay-log-base=logs/replay.copper",
            ],
            defaults,
        )
        .expect("parse replay CLI");
        assert_eq!(cli.debug_base.as_deref(), Some("copper/test/debug/v1"));
        assert_eq!(cli.log_base, PathBuf::from("logs/override.copper"));
        assert_eq!(cli.replay_log_base, PathBuf::from("logs/replay.copper"));
    }
}
