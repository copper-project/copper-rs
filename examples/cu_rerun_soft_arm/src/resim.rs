//! Serve recorded Copper messages and their custom representation to Timetraveler.

use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29::remote_debug::SessionOpenParams;
use cu29::replay::{
    ReplayCli, ReplayDefaults, per_session_replay_log_base, remove_log_family, serve_remote_debug,
};
use std::path::Path;

#[copper_runtime(config = "copperconfig.ron", sim_mode = true, ignore_resources = true)]
struct SoftArmReplay {}

type Recorded = CopperList<default::CuStampedDataSet>;
type Callback = for<'a> fn(
    &'a Recorded,
    RobotClock,
    RobotClockMock,
) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a>;
type TimeOf = fn(&Recorded) -> Option<CuTime>;

fn app_factory(
    params: &SessionOpenParams,
    template: &Path,
) -> CuResult<(default::SoftArmReplay, RobotClock, RobotClockMock)> {
    let (clock, mock) = RobotClock::mock();
    let path = per_session_replay_log_base(template, [params.role.as_deref().unwrap_or("session")]);
    remove_log_family(&path)?;
    let mut callback = |_: default::SimStep<'_>| SimOverride::ExecuteByRuntime;
    let app = SoftArmReplay::builder()
        .with_clock(clock.clone())
        .with_log_path(path, Some(16 * 1024 * 1024))?
        .with_sim_callback(&mut callback)
        .build()?;
    Ok((app.into_inner(), clock, mock))
}

fn build_callback<'a>(
    recorded: &'a Recorded,
    _: RobotClock,
    _: RobotClockMock,
) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a> {
    // Inject logged source messages and skip the live Rerun sink.
    Box::new(move |step| default::recorded_debug_replay_step(step, recorded))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = ReplayCli::parse(
        ReplayDefaults::new(
            concat!(env!("CARGO_MANIFEST_DIR"), "/logs/soft-arm.copper"),
            concat!(env!("CARGO_MANIFEST_DIR"), "/logs/replay.copper"),
        )
        .with_debug_base("copper/examples/cu_rerun_soft_arm/debug/v1"),
    );
    let template = cli.replay_log_base;
    serve_remote_debug::<
        default::SoftArmReplay,
        default::CuStampedDataSet,
        Callback,
        TimeOf,
        MmapSectionStorage,
        MmapUnifiedLoggerWrite,
        _,
    >(
        cli.debug_base
            .as_deref()
            .expect("default debug base is set"),
        &cli.log_base,
        move |params| app_factory(params, &template),
        build_callback,
        cu29::simulation::recorded_copperlist_timestamp,
    )?;
    Ok(())
}

#[cfg(test)]
#[path = "replay_test.rs"]
mod tests;
