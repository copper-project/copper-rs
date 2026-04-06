extern crate alloc;

mod messages;
mod sim_support;
mod tasks;

use cu29::prelude::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use cu29::prelude::*;
use cu29::remote_debug::SessionOpenParams;
use cu29::replay::{
    ReplayCli, ReplayDefaults, per_session_replay_log_base, remove_log_family, serve_remote_debug,
};
use std::error::Error;
use std::path::Path;

const DEFAULT_DEBUG_BASE: &str = "copper/examples/cu_flight_controller/debug/v1";
const DEFAULT_LOG_BASE: &str = "logs/flight_controller_sim.copper";
const DEFAULT_REPLAY_LOG_BASE: &str = "logs/flight_controller_resim.copper";
const REPLAY_LOG_SLAB_SIZE: Option<usize> = Some(128 * 1024 * 1024);

#[copper_runtime(config = "copperconfig.ron", sim_mode = true, ignore_resources = true)]
struct FlightControllerReSim {}

type ReplayCopperList = CopperList<gnss::CuStampedDataSet>;
type ReplayBuildCallback =
    for<'a> fn(
        &'a ReplayCopperList,
        RobotClock,
    ) -> Box<dyn for<'z> FnMut(gnss::SimStep<'z>) -> SimOverride + 'a>;
type ReplayTimeExtractor = fn(&ReplayCopperList) -> Option<CuTime>;

fn main() -> Result<(), Box<dyn Error>> {
    let cli = ReplayCli::parse(
        ReplayDefaults::new(DEFAULT_LOG_BASE, DEFAULT_REPLAY_LOG_BASE)
            .with_debug_base(DEFAULT_DEBUG_BASE),
    );
    let replay_template = cli.replay_log_base.clone();
    serve_remote_debug::<
        gnss::FlightControllerReSim,
        gnss::CuStampedDataSet,
        ReplayBuildCallback,
        ReplayTimeExtractor,
        MmapSectionStorage,
        MmapUnifiedLoggerWrite,
        _,
    >(
        cli.debug_base
            .as_deref()
            .expect("default debug base is always set"),
        &cli.log_base,
        move |params| app_factory(params, &replay_template),
        build_callback,
        extract_time,
    )?;
    Ok(())
}

fn app_factory(
    params: &SessionOpenParams,
    replay_template: &Path,
) -> CuResult<(gnss::FlightControllerReSim, RobotClock, RobotClockMock)> {
    let (clock, clock_mock) = RobotClock::mock();
    let replay_log_base = per_session_replay_log_base(
        replay_template,
        [params.role.as_deref().unwrap_or("session")],
    );
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
