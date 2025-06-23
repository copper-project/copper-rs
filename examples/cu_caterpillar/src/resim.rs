pub mod tasks;
use cu29::prelude::*;
use cu29_export::copperlists_dump;
use cu29_helpers::basic_copper_setup;
use default::SimStep::{Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5, Gpio6, Gpio7, Src};
use std::path::{Path, PathBuf};

// To enable resim, it is just your regular macro with sim_mode true
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct CaterpillarReSim {}

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
    copper_list: CopperList<default::CuMsgs>,
) {
    // Sync the copper clock to the simulated physics clock.
    let msgs = &copper_list.msgs;

    // Simulate what the sim is doing here
    let CuDuration(process_time) = msgs.get_src_output().metadata.process_time.start.unwrap();
    robot_clock.set_value(process_time);

    let mut sim_callback = move |step: default::SimStep| -> SimOverride {
        match step {
            Src(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_src_output().clone();
                SimOverride::ExecutedBySim
            }
            Gpio0(_) | Gpio1(_) | Gpio2(_) | Gpio3(_) | Gpio4(_) | Gpio5(_) | Gpio6(_)
            | Gpio7(_) => SimOverride::ExecutedBySim, // Locally it is not doing anything.
            _ => SimOverride::ExecuteByRuntime,
        }
    };
    copper_app
        .run_one_iteration(&mut sim_callback)
        .expect("Failed to run application.");
}

fn main() {
    // Create the Copper App in simulation mode.
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);
    let logger_path = "logs/caterpillarresim.copper";
    let (robot_clock, mut robot_clock_mock) = RobotClock::mock();
    let copper_ctx = basic_copper_setup(
        &PathBuf::from(logger_path),
        LOG_SLAB_SIZE,
        true,
        Some(robot_clock.clone()),
    )
    .expect("Failed to setup logger.");

    let mut copper_app = CaterpillarReSimBuilder::new()
        .with_context(&copper_ctx)
        .with_sim_callback(&mut default_callback)
        .build()
        .expect("Failed to create runtime.");

    copper_app
        .start_all_tasks(&mut default_callback)
        .expect("Failed to start all tasks.");

    // Read back the logs from a previous run
    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_base_name(Path::new("logs/caterpillar.copper"))
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger");
    };
    let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
    let iter = copperlists_dump::<default::CuMsgs>(&mut reader);
    for entry in iter {
        println!("{entry:#?}");
        run_one_copperlist(&mut copper_app, &mut robot_clock_mock, entry);
    }
    copper_app
        .stop_all_tasks(&mut default_callback)
        .expect("Failed to start all tasks.");
}
