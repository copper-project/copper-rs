pub mod tasks;

use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29::remote_debug::SessionOpenParams;
use cu29::replay::{
    ReplayCli, ReplayDefaults, ensure_log_family_exists, per_session_replay_log_base,
    remove_log_family, serve_remote_debug,
};
use cu29_export::{copperlists_reader, keyframes_reader};
use std::error::Error;
use std::path::{Path, PathBuf};

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);
const APP_NAME: &str = env!("CARGO_PKG_NAME");

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct {{project-name | upper_camel_case}}ReSim {}

type ReplayCopperList = CopperList<default::CuStampedDataSet>;
type ReplayBuildCallback =
    for<'a> fn(
        &'a ReplayCopperList,
        RobotClock,
    ) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a>;
type ReplayTimeExtractor = fn(&ReplayCopperList) -> Option<CuTime>;

fn default_log_base() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("logs")
        .join(format!("{APP_NAME}.copper"))
}

fn default_replay_log_base() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("logs")
        .join(format!("{APP_NAME}_resim.copper"))
}

fn default_callback(_step: default::SimStep<'_>) -> SimOverride {
    SimOverride::ExecuteByRuntime
}

fn build_callback<'a>(
    copperlist: &'a ReplayCopperList,
    _clock_for_callbacks: RobotClock,
) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a> {
    Box::new(move |step: default::SimStep<'_>| default::recorded_replay_step(step, copperlist))
}

fn extract_time(copperlist: &ReplayCopperList) -> Option<CuTime> {
    cu29::simulation::recorded_copperlist_timestamp(copperlist)
}

fn make_app(
    replay_log_base: &Path,
) -> CuResult<(
    {{project-name | upper_camel_case}}ReSim,
    RobotClock,
    RobotClockMock,
)> {
    let (clock, clock_mock) = RobotClock::mock();
    let app = {{project-name | upper_camel_case}}ReSim::builder()
        .with_clock(clock.clone())
        .with_log_path(replay_log_base, PREALLOCATED_STORAGE_SIZE)?
        .with_sim_callback(&mut default_callback)
        .build()?;
    Ok((app, clock, clock_mock))
}

fn run_one_shot(log_base: &Path, replay_log_base: &Path) -> CuResult<()> {
    remove_log_family(replay_log_base)?;
    let (mut app, clock, clock_mock) = make_app(replay_log_base)?;

    app.start_all_tasks(&mut default_callback)?;

    let UnifiedLogger::Read(dl_kf) = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .map_err(|err| CuError::new_with_cause("failed to open log for keyframes", err))?
    else {
        return Err(CuError::from("expected read logger for keyframes"));
    };
    let mut keyframe_reader = UnifiedLoggerIOReader::new(dl_kf, UnifiedLogType::FrozenTasks);
    let mut keyframes = keyframes_reader(&mut keyframe_reader).peekable();

    if let Some(first_kf) = keyframes.peek() {
        app.copper_runtime_mut().lock_keyframe(first_kf);
        app.copper_runtime_mut()
            .set_forced_keyframe_timestamp(first_kf.timestamp);
        clock_mock.set_value(first_kf.timestamp.as_nanos());
    }

    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .map_err(|err| CuError::new_with_cause("failed to open copperlist log", err))?
    else {
        return Err(CuError::from("expected read logger for copperlists"));
    };
    let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
    for entry in copperlists_reader::<default::CuStampedDataSet>(&mut reader) {
        if let Some(kf) = keyframes.peek() {
            if kf.culistid == entry.id {
                app.copper_runtime_mut().lock_keyframe(kf);
                app.copper_runtime_mut()
                    .set_forced_keyframe_timestamp(kf.timestamp);
                clock_mock.set_value(kf.timestamp.as_nanos());
                keyframes.next();
            }
        }

        if let Some(ts) = extract_time(&entry) {
            clock_mock.set_value(ts.as_nanos());
        }

        let mut sim_callback = build_callback(&entry, clock.clone());
        app.run_one_iteration(&mut sim_callback)?;
    }

    app.stop_all_tasks(&mut default_callback)?;
    let _ = app.log_shutdown_completed();
    Ok(())
}

fn app_factory(
    params: &SessionOpenParams,
    replay_template: &Path,
) -> CuResult<(
    {{project-name | upper_camel_case}}ReSim,
    RobotClock,
    RobotClockMock,
)> {
    let replay_log_base = per_session_replay_log_base(
        replay_template,
        [params.role.as_deref().unwrap_or("session")],
    );
    remove_log_family(&replay_log_base)?;
    make_app(&replay_log_base)
}

fn run_remote_debug_server(
    debug_base: &str,
    log_base: &Path,
    replay_log_base: &Path,
) -> CuResult<()> {
    let replay_template = replay_log_base.to_path_buf();
    serve_remote_debug::<
        {{project-name | upper_camel_case}}ReSim,
        default::CuStampedDataSet,
        ReplayBuildCallback,
        ReplayTimeExtractor,
        MmapSectionStorage,
        MmapUnifiedLoggerWrite,
        _,
    >(
        debug_base,
        log_base,
        move |params| app_factory(params, &replay_template),
        build_callback,
        extract_time,
    )
}

fn main() -> Result<(), Box<dyn Error>> {
    let cli = ReplayCli::parse(ReplayDefaults::new(
        default_log_base(),
        default_replay_log_base(),
    ));

    ensure_log_family_exists(&cli.log_base)?;
    if let Some(debug_base) = cli.debug_base.as_deref() {
        run_remote_debug_server(debug_base, &cli.log_base, &cli.replay_log_base)?;
    } else {
        run_one_shot(&cli.log_base, &cli.replay_log_base)?;
    }
    Ok(())
}
