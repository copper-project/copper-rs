extern crate alloc;

mod messages;
mod sim_support;
mod tasks;

use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29::remote_debug::{RemoteDebugPaths, RemoteDebugZenohServer, SessionOpenParams};
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

const DEFAULT_DEBUG_BASE: &str = "copper/examples/cu_flight_controller/debug/v1";
const DEFAULT_LOG_BASE: &str = "logs/flight_controller_sim.copper";
const DEFAULT_REPLAY_LOG_BASE: &str = "logs/flight_controller_resim.copper";
const REPLAY_LOG_SLAB_SIZE: Option<usize> = Some(128 * 1024 * 1024);

static NEXT_REPLAY_SESSION_ID: AtomicU64 = AtomicU64::new(0);

#[copper_runtime(config = "copperconfig.ron", sim_mode = true, ignore_resources = true)]
struct FlightControllerReSim {}

type ReplayCopperList = CopperList<gnss::CuStampedDataSet>;
type ReplayBuildCallback =
    for<'a> fn(
        &'a ReplayCopperList,
        RobotClock,
    ) -> Box<dyn for<'z> FnMut(gnss::SimStep<'z>) -> SimOverride + 'a>;
type ReplayTimeExtractor = fn(&ReplayCopperList) -> Option<CuTime>;

#[derive(Debug, Clone)]
struct Cli {
    debug_base: String,
    log_base: Option<PathBuf>,
    replay_log_base: Option<PathBuf>,
}

fn usage(program: &str) -> String {
    format!(
        "Usage: {program} [--debug-base <path>] [--log-base <path>] [--replay-log-base <path>]\n\
\n\
Options:\n\
  --debug-base       Remote debug namespace base.\n\
                     default: {DEFAULT_DEBUG_BASE}\n\
  --log-base         Optional Copper base log path to validate at startup.\n\
                     default: {DEFAULT_LOG_BASE}\n\
  --replay-log-base  Optional replay log base used as the per-session template.\n\
                     default: {DEFAULT_REPLAY_LOG_BASE}\n\
  -h, --help         Show this help.\n"
    )
}

fn parse_args(args: &[String]) -> Result<Cli, String> {
    let mut debug_base = Some(DEFAULT_DEBUG_BASE.to_owned());
    let mut log_base = None::<PathBuf>;
    let mut replay_log_base = None::<PathBuf>;

    let mut iter = args.iter().skip(1);
    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--debug-base" => {
                let value = iter
                    .next()
                    .ok_or_else(|| "missing value for --debug-base".to_owned())?;
                debug_base = Some(value.clone());
            }
            "--log-base" => {
                let value = iter
                    .next()
                    .ok_or_else(|| "missing value for --log-base".to_owned())?;
                log_base = Some(PathBuf::from(value));
            }
            "--replay-log-base" => {
                let value = iter
                    .next()
                    .ok_or_else(|| "missing value for --replay-log-base".to_owned())?;
                replay_log_base = Some(PathBuf::from(value));
            }
            "-h" | "--help" => return Err(String::new()),
            unknown => return Err(format!("unknown argument: {unknown}")),
        }
    }

    Ok(Cli {
        debug_base: debug_base.unwrap_or_else(|| DEFAULT_DEBUG_BASE.to_owned()),
        log_base,
        replay_log_base,
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().collect();
    let program = args.first().map_or("quad-resim", String::as_str);
    let cli = match parse_args(&args) {
        Ok(parsed) => parsed,
        Err(msg) if msg.is_empty() => {
            println!("{}", usage(program));
            return Ok(());
        }
        Err(msg) => {
            eprintln!("{msg}\n\n{}", usage(program));
            return Err(msg.into());
        }
    };

    let startup_log_base = cli
        .log_base
        .clone()
        .unwrap_or_else(|| PathBuf::from(DEFAULT_LOG_BASE));
    ensure_log_family_exists(&startup_log_base)?;

    let replay_template = cli
        .replay_log_base
        .clone()
        .unwrap_or_else(|| PathBuf::from(DEFAULT_REPLAY_LOG_BASE));
    let paths = RemoteDebugPaths::new(&cli.debug_base);
    let replay_template_for_factory = replay_template.clone();
    let mut server = RemoteDebugZenohServer::<
        gnss::FlightControllerReSim,
        gnss::CuStampedDataSet,
        ReplayBuildCallback,
        ReplayTimeExtractor,
        MmapSectionStorage,
        MmapUnifiedLoggerWrite,
        _,
    >::new(
        paths,
        move |params| app_factory(params, &replay_template_for_factory),
        build_callback,
        extract_time,
    )?;
    server.serve_until_stopped()?;
    Ok(())
}

fn app_factory(
    params: &SessionOpenParams,
    replay_template: &Path,
) -> CuResult<(gnss::FlightControllerReSim, RobotClock, RobotClockMock)> {
    let (clock, clock_mock) = RobotClock::mock();
    let replay_log_base = per_session_replay_log_base(replay_template, params.role.as_deref());
    remove_log_family(&replay_log_base)?;

    let mut default_callback = |_step: gnss::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let app = gnss::FlightControllerReSim::builder()
        .with_clock(clock.clone())
        .with_log_path(replay_log_base, REPLAY_LOG_SLAB_SIZE)?
        .with_sim_callback(&mut default_callback)
        .build()?;

    Ok((app, clock, clock_mock))
}

fn build_callback<'a>(
    copperlist: &'a ReplayCopperList,
    _clock_for_cb: RobotClock,
) -> Box<dyn for<'z> FnMut(gnss::SimStep<'z>) -> SimOverride + 'a> {
    Box::new(move |step: gnss::SimStep<'_>| gnss::recorded_replay_step(step, copperlist))
}

fn extract_time(copperlist: &ReplayCopperList) -> Option<CuTime> {
    cu29::simulation::recorded_copperlist_timestamp(copperlist)
}

fn ensure_log_family_exists(log_base: &Path) -> CuResult<()> {
    if log_base.exists() {
        return Ok(());
    }

    let first_slab = first_slab_path(log_base)?;
    if first_slab.exists() {
        return Ok(());
    }

    Err(CuError::from(format!(
        "log family not found for {} (expected {}); run `just sim` first or pass --log-base",
        log_base.display(),
        first_slab.display()
    )))
}

fn remove_log_family(log_base: &Path) -> CuResult<()> {
    let parent = log_base
        .parent()
        .ok_or_else(|| CuError::from("log path must have a parent directory"))?;
    fs::create_dir_all(parent)
        .map_err(|e| CuError::new_with_cause("failed to create replay log directory", e))?;

    let stem = log_base
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| CuError::from("invalid UTF-8 replay log stem"))?
        .to_owned();

    for entry in fs::read_dir(parent)
        .map_err(|e| CuError::new_with_cause("failed to scan replay log directory", e))?
    {
        let entry =
            entry.map_err(|e| CuError::new_with_cause("failed to read replay log entry", e))?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        let Some(name) = path.file_name().and_then(|s| s.to_str()) else {
            continue;
        };
        if name.starts_with(&stem) && name.ends_with(".copper") {
            fs::remove_file(&path)
                .map_err(|e| CuError::new_with_cause("failed to remove replay log file", e))?;
        }
    }

    Ok(())
}

fn first_slab_path(log_base: &Path) -> CuResult<PathBuf> {
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

fn per_session_replay_log_base(template: &Path, role: Option<&str>) -> PathBuf {
    let seq = NEXT_REPLAY_SESSION_ID.fetch_add(1, Ordering::Relaxed);
    let label = sanitize_label(role.unwrap_or("session"));
    let parent = template.parent().unwrap_or_else(|| Path::new("."));
    let stem = template
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("flight_controller_resim");
    let ext = template
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("copper");
    parent.join(format!("{stem}_{label}_{seq}.{ext}"))
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
    let sanitized = sanitized.trim_matches('_').to_owned();
    if sanitized.is_empty() {
        "session".to_owned()
    } else {
        sanitized
    }
}
