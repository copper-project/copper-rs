pub mod tasks;
use cu29::clock::{RobotClock, RobotClockMock};
use cu29::copperlist::CopperList;
use cu29::simulation::{CuTaskCallbackState, SimOverride};
use cu29_derive::copper_runtime;
use cu29_export::copperlists_dump;
use cu29_helpers::basic_copper_setup;
use cu29_log_derive::debug;
use cu29_traits::UnifiedLogType;
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use std::path::{Path, PathBuf};

// To enable resim, it is just your regular macro with sim_mode true
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct BalanceBotReSim {}

fn default_callback(step: SimStep) -> SimOverride {
    match step {
        // Don't let the real task execute process and override with our logic.
        SimStep::Balpos(_) => SimOverride::ExecutedBySim,
        SimStep::Railpos(_) => SimOverride::ExecutedBySim,
        SimStep::Motor(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn run_one_copperlist(
    copper_app: &mut BalanceBotReSim,
    robot_clock: &mut RobotClockMock,
    copper_list: CopperList<CuMsgs>,
) {
    // Sync the copper clock to the simulated physics clock.
    robot_clock.set_value(0u64); // get it from the log
    let msgs = &copper_list.msgs.0; // TODO: dewrap this.
    let mut sim_callback = move |step: SimStep<'_>| -> SimOverride {
        match step {
            SimStep::Balpos(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.0.clone(); // TODO: <- that is scheduling dependent.
                SimOverride::ExecutedBySim
            }
            SimStep::Balpos(_) => SimOverride::ExecutedBySim,
            SimStep::Railpos(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.2.clone(); // TODO: <- that is scheduling dependent.
                SimOverride::ExecutedBySim
            }
            SimStep::Railpos(_) => SimOverride::ExecutedBySim,
            SimStep::Motor(CuTaskCallbackState::Process(input, output)) => {
                // And now when copper wants to use the motor
                // we apply a force in the simulation.
                let maybe_motor_actuation = input.payload();
                // just log the behavior
                SimOverride::ExecutedBySim
            }
            SimStep::Motor(_) => SimOverride::ExecutedBySim,
            _ => SimOverride::ExecuteByRuntime,
        }
    };
    copper_app
        .run_one_iteration(&mut sim_callback)
        .expect("Failed to run application.");
}

fn stop_copper_on_exit(copper_app: &mut BalanceBotReSim) {
    copper_app
        .stop_all_tasks(&mut default_callback) // let the tasks clean themselves up
        .expect("Failed to stop all tasks.");
}

fn main() {
    // Create the Copper App in simulation mode.
    const LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);
    let logger_path = "logs/balanceresim.copper";
    let (robot_clock, mut robot_clock_mock) = RobotClock::mock();
    let copper_ctx = basic_copper_setup(
        &PathBuf::from(logger_path),
        LOG_SLAB_SIZE,
        false,
        Some(robot_clock.clone()),
    )
    .expect("Failed to setup logger.");

    let mut copper_app = BalanceBotReSim::new(
        robot_clock.clone(),
        copper_ctx.unified_logger.clone(),
        &mut default_callback,
    )
    .expect("Failed to create runtime.");

    copper_app
        .start_all_tasks(&mut default_callback)
        .expect("Failed to start all tasks.");

    // Read back the logs from a previous run
    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_base_name(Path::new("logs/balance.copper"))
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger");
    };
    let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
    let iter = copperlists_dump::<CuMsgs>(&mut reader);
    for entry in iter {
        println!("{:#?}", entry);
        run_one_copperlist(&mut copper_app, &mut robot_clock_mock, entry);
    }
}
