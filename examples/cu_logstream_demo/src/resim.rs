// Recorded replay needs the raw application API to restore clock/keyframe boundaries.
#![allow(deprecated)]

use cu_logstream_demo::SLAB_BYTES;
use cu29::prelude::*;
use cu29::replay::{ReplayCli, ReplayDefaults, per_session_replay_log_base, serve_remote_debug};
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use std::path::Path;

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct Replay {}

type List = CopperList<default::CuStampedDataSet>;
type Callback = for<'a> fn(
    &'a List,
    RobotClock,
    RobotClockMock,
) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a>;
type Timestamp = fn(&List) -> Option<CuTime>;

fn callback<'a>(
    list: &'a List,
    _: RobotClock,
    _: RobotClockMock,
) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a> {
    Box::new(move |step| default::recorded_replay_step(step, list))
}

fn build(path: &Path) -> CuResult<(Replay, RobotClock, RobotClockMock)> {
    let (clock, mock) = RobotClock::mock();
    let app = Replay::builder()
        .with_clock(clock.clone())
        .with_log_path(path, Some(SLAB_BYTES))?
        .with_sim_callback(&mut |_| SimOverride::ExecuteByRuntime)
        .build()?
        .into_inner();
    Ok((app, clock, mock))
}

fn reader(path: &Path, kind: UnifiedLogType) -> CuResult<UnifiedLoggerIOReader> {
    Ok(UnifiedLoggerIOReader::new(
        UnifiedLoggerRead::new(path)
            .map_err(|error| CuError::new_with_cause("Open archive", error))?,
        kind,
    ))
}

fn main() -> CuResult<()> {
    let cli = ReplayCli::parse(ReplayDefaults::new(
        "logs/received.copper",
        "logs/replay.copper",
    ));
    cu29::replay::ensure_log_family_exists(&cli.log_base)?;
    if let Some(debug_base) = cli.debug_base {
        let template = cli.replay_log_base;
        return serve_remote_debug::<
            Replay,
            default::CuStampedDataSet,
            Callback,
            Timestamp,
            MmapSectionStorage,
            MmapUnifiedLoggerWrite,
            _,
        >(
            &debug_base,
            &cli.log_base,
            move |params| {
                build(&per_session_replay_log_base(
                    &template,
                    [params.role.as_deref().unwrap_or("session")],
                ))
            },
            callback,
            cu29::simulation::recorded_copperlist_timestamp,
        );
    }
    if cu29::replay::first_slab_path(&cli.replay_log_base)?.exists() {
        return Err("Replay output already exists; choose a new --replay-log-base".into());
    }
    let (mut app, _, mock) = build(&cli.replay_log_base)?;
    app.start_all_tasks(&mut |_| SimOverride::ExecuteByRuntime)?;
    let keyframes: Vec<_> =
        cu29_export::keyframes_reader(reader(&cli.log_base, UnifiedLogType::FrozenTasks)?)
            .collect();
    let mut count = 0;
    for list in cu29_export::copperlists_reader::<default::CuStampedDataSet>(reader(
        &cli.log_base,
        UnifiedLogType::CopperList,
    )?) {
        let keyframe = keyframes.iter().find(|frame| frame.culistid == list.id);
        if let Some(frame) = keyframe {
            <Replay as cu29::prelude::app::CuSimApplication<
                MmapSectionStorage,
                MmapUnifiedLoggerWrite,
            >>::restore_keyframe(&mut app, frame)?;
        }
        // Generated replay checks continuity before touching the clock or tasks.
        app.replay_recorded_copperlist(&mock, &list, keyframe)?;
        count += 1;
    }
    app.stop_all_tasks(&mut |_| SimOverride::ExecuteByRuntime)?;
    drop(app);
    let recorded = cu_logstream_demo::read_lists(&cli.log_base)?;
    let replayed = cu_logstream_demo::read_lists(&cli.replay_log_base)?;
    if recorded.len() != replayed.len() {
        return Err("Recorded replay lost CopperLists".into());
    }
    for (expected, actual) in recorded.iter().zip(&replayed) {
        let encode = |list| {
            cu29::bincode::encode_to_vec(list, cu29::bincode::config::standard())
                .map_err(|error| CuError::new_with_cause("Encode replay comparison", error))
        };
        if encode(expected)? != encode(actual)? {
            return Err(format!("Recorded replay changed CopperList {}", expected.id).into());
        }
    }
    println!("Replayed {count} captured CopperLists with verified keyframe boundaries.");
    Ok(())
}
