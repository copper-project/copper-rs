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

fn main() -> CuResult<()> {
    let cli = ReplayCli::parse(ReplayDefaults::new(
        "logs/received.copper",
        "logs/replay.copper",
    ));
    cu29::replay::ensure_log_family_exists(&cli.log_base)?;
    if let Some(debug_base) = cli.debug_base {
        if cu_logstream_demo::read_lists(&cli.log_base)?
            .iter()
            .any(|list| list.msgs.get_derived_output().payload().is_none())
        {
            return Err(
                "Run just resim first, then use resim-debug on the reconstructed full log".into(),
            );
        }
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
    let mut twin = cu29_logstream::twin::LiveTwin::<cu_logstream_demo::twin::Twin>::new()?;
    let keyframes = cu_logstream_demo::read_keyframes(&cli.log_base)?;
    let logger = UnifiedLoggerBuilder::new()
        .file_base_name(&cli.replay_log_base)
        .preallocated_size(SLAB_BYTES)
        .write(true)
        .create(true)
        .build()
        .map_err(|e| CuError::new_with_cause("Create replay output", e))?;
    let UnifiedLogger::Write(logger) = logger else {
        unreachable!()
    };
    let logger = std::sync::Arc::new(std::sync::Mutex::new(logger));
    let mut output = cu29_unifiedlog::stream_write::<cu_logstream_demo::List, MmapSectionStorage>(
        logger.clone(),
        UnifiedLogType::CopperList,
        65536,
    )?;
    let mut states = cu29_unifiedlog::stream_write::<cu29::curuntime::KeyFrame, MmapSectionStorage>(
        logger.clone(),
        UnifiedLogType::FrozenTasks,
        65536,
    )?;
    let mut provenance = cu29_unifiedlog::stream_write::<
        cu29::continuity::StreamContinuityRecord,
        MmapSectionStorage,
    >(logger, UnifiedLogType::StreamContinuity, 65536)?;
    let source = UnifiedLoggerIOReader::new(
        UnifiedLoggerRead::new(&cli.log_base)
            .map_err(|e| CuError::new_with_cause("Open capture provenance", e))?,
        UnifiedLogType::StreamContinuity,
    );
    for record in cu29_export::stream_continuity_reader(source) {
        if !matches!(
            record,
            cu29::continuity::StreamContinuityRecord::Capture { .. }
        ) {
            provenance.log(&record)?;
        }
    }
    let mut count = 0;
    for capture in cu_logstream_demo::read_captures(&cli.log_base)? {
        let keyframe = keyframes
            .iter()
            .find(|k| k.culistid == capture.copperlist.id);
        let list = twin
            .reconstruct(capture, keyframe)?
            .ok_or_else(|| CuError::from("Missing replay anchor"))?;
        if let Some(keyframe) = keyframe {
            states.log(keyframe)?;
        }
        output.log(&list)?;
        count += 1;
    }
    drop(output);
    println!("Reconstructed {count} full CopperLists from captured inputs.");
    Ok(())
}
