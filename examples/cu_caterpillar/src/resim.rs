pub mod tasks;

use cu29::prelude::app::CuSimApplication;
use cu29::prelude::*;
use cu29::remote_debug::SessionOpenParams;
use cu29::replay::{
    ReplayCli, ReplayDefaults, ensure_log_family_exists, per_session_replay_log_base,
    remove_log_family, serve_remote_debug,
};
use cu29_export::{copperlists_reader, keyframes_reader};
use cu29_unifiedlog::memmap::{MmapSectionStorage, MmapUnifiedLoggerWrite};
use default::SimStep::{Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5, Gpio6, Gpio7, Src};
use std::error::Error;
use std::path::Path;

const DEFAULT_LOG_BASE: &str = "logs/caterpillar.copper";
const DEFAULT_REPLAY_LOG_BASE: &str = "logs/caterpillarresim.copper";
#[allow(clippy::identity_op)]
const REPLAY_LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);

// To enable resim, it is just your regular macro with sim_mode true
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct CaterpillarReSim {}

type ReplayCopperList = CopperList<default::CuStampedDataSet>;
type ReplayBuildCallback =
    for<'a> fn(
        &'a ReplayCopperList,
        RobotClock,
        RobotClockMock,
    ) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a>;
type ReplayTimeExtractor = fn(&ReplayCopperList) -> Option<CuTime>;

fn default_callback(step: default::SimStep) -> SimOverride {
    match step {
        Src(_) | Gpio0(_) | Gpio1(_) | Gpio2(_) | Gpio3(_) | Gpio4(_) | Gpio5(_) | Gpio6(_)
        | Gpio7(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn run_one_copperlist(
    copper_app: &mut CaterpillarReSim,
    robot_clock: &mut RobotClockMock,
    copper_list: CopperList<default::CuStampedDataSet>,
    pending_kf_ts: Option<CuTime>,
) {
    // Sync the copper clock to the recorded physics clock and replay every task's output
    // byte-for-byte from the captured copperlist so we don't re-stamp metadata.
    let msgs = copper_list.msgs;

    if let Some(ts) = pending_kf_ts {
        copper_app
            .copper_runtime_mut()
            .set_forced_keyframe_timestamp(ts);
        robot_clock.set_value(ts.as_nanos());
    } else {
        let process_time = msgs
            .get_src_output()
            .metadata
            .process_time
            .start
            .unwrap()
            .as_nanos();
        robot_clock.set_value(process_time);
    }

    let mut sim_callback = move |step: default::SimStep| -> SimOverride {
        use CuTaskCallbackState::*;
        use default::SimStep::*;
        match step {
            Src(Process(_, output)) => {
                *output = msgs.get_src_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct0(Process(_, output)) => {
                *output = msgs.get_ct_0_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct1(Process(_, output)) => {
                *output = msgs.get_ct_1_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct2(Process(_, output)) => {
                *output = msgs.get_ct_2_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct3(Process(_, output)) => {
                *output = msgs.get_ct_3_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct4(Process(_, output)) => {
                *output = msgs.get_ct_4_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct5(Process(_, output)) => {
                *output = msgs.get_ct_5_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct6(Process(_, output)) => {
                *output = msgs.get_ct_6_output().clone();
                SimOverride::ExecutedBySim
            }
            Ct7(Process(_, output)) => {
                *output = msgs.get_ct_7_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio0(Process(_, output)) => {
                *output = msgs.get_gpio_0_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio1(Process(_, output)) => {
                *output = msgs.get_gpio_1_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio2(Process(_, output)) => {
                *output = msgs.get_gpio_2_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio3(Process(_, output)) => {
                *output = msgs.get_gpio_3_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio4(Process(_, output)) => {
                *output = msgs.get_gpio_4_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio5(Process(_, output)) => {
                *output = msgs.get_gpio_5_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio6(Process(_, output)) => {
                *output = msgs.get_gpio_6_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio7(Process(_, output)) => {
                *output = msgs.get_gpio_7_output().clone();
                SimOverride::ExecutedBySim
            }
            _ => SimOverride::ExecuteByRuntime,
        }
    };
    copper_app
        .run_one_iteration(&mut sim_callback)
        .expect("failed to run resim iteration");
}

fn open_log_reader(log_path: &Path, log_type: UnifiedLogType) -> CuResult<UnifiedLoggerIOReader> {
    let UnifiedLogger::Read(reader) = UnifiedLoggerBuilder::new()
        .file_base_name(log_path)
        .build()
        .map_err(|err| CuError::new_with_cause("failed to open log", err))?
    else {
        return Err(CuError::from("log file could not be opened for reading"));
    };
    Ok(UnifiedLoggerIOReader::new(reader, log_type))
}

fn build_callback<'a>(
    copper_list: &'a ReplayCopperList,
    _process_clock: RobotClock,
    _clock_for_callbacks: RobotClockMock,
) -> Box<dyn for<'z> FnMut(default::SimStep<'z>) -> SimOverride + 'a> {
    Box::new(move |step: default::SimStep<'_>| default::recorded_replay_step(step, copper_list))
}

fn extract_time(copperlist: &ReplayCopperList) -> Option<CuTime> {
    cu29::simulation::recorded_copperlist_timestamp(copperlist)
}

fn make_app(log_base: &Path) -> CuResult<(CaterpillarReSim, RobotClock, RobotClockMock)> {
    let (robot_clock, robot_clock_mock) = RobotClock::mock();
    let copper_app = CaterpillarReSim::builder()
        .with_clock(robot_clock.clone())
        .with_log_path(log_base, REPLAY_LOG_SLAB_SIZE)?
        .with_sim_callback(&mut default_callback)
        .build()?;
    Ok((copper_app, robot_clock, robot_clock_mock))
}

fn run_one_shot(log_base: &Path, replay_log_base: &Path) -> CuResult<()> {
    remove_log_family(replay_log_base)?;
    let (mut copper_app, _robot_clock, mut robot_clock_mock) = make_app(replay_log_base)?;

    copper_app.start_all_tasks(&mut default_callback)?;

    let mut keyframes_ioreader = open_log_reader(log_base, UnifiedLogType::FrozenTasks)?;
    let mut kf_iter = keyframes_reader(&mut keyframes_ioreader).peekable();

    if let Some(first_kf) = kf_iter.peek() {
        <CaterpillarReSim as CuSimApplication<MmapSectionStorage, MmapUnifiedLoggerWrite>>::restore_keyframe(
            &mut copper_app,
            first_kf,
        )?;
        let ts = first_kf.timestamp;
        robot_clock_mock.set_value(ts.as_nanos());
    }

    let mut copperlists = open_log_reader(log_base, UnifiedLogType::CopperList)?;
    let cl_iter = copperlists_reader::<default::CuStampedDataSet>(&mut copperlists);
    for entry in cl_iter {
        let pending_kf_ts = if let Some(kf) = kf_iter.peek() {
            if kf.culistid == entry.id {
                let ts = kf.timestamp;
                copper_app.copper_runtime_mut().lock_keyframe(kf);
                <CaterpillarReSim as CuSimApplication<
                    MmapSectionStorage,
                    MmapUnifiedLoggerWrite,
                >>::restore_keyframe(&mut copper_app, kf)?;
                kf_iter.next();
                Some(ts)
            } else {
                None
            }
        } else {
            None
        };

        run_one_copperlist(&mut copper_app, &mut robot_clock_mock, entry, pending_kf_ts);
    }

    Ok(())
}

fn app_factory(
    params: &SessionOpenParams,
    replay_template: &Path,
) -> CuResult<(CaterpillarReSim, RobotClock, RobotClockMock)> {
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
        CaterpillarReSim,
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
        DEFAULT_LOG_BASE,
        DEFAULT_REPLAY_LOG_BASE,
    ));

    ensure_log_family_exists(&cli.log_base)?;
    if let Some(debug_base) = cli.debug_base.as_deref() {
        run_remote_debug_server(debug_base, &cli.log_base, &cli.replay_log_base)?;
    } else {
        run_one_shot(&cli.log_base, &cli.replay_log_base)?;
    }
    Ok(())
}
