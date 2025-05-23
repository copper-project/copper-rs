pub mod tasks;
use cu29::prelude::*;
use cu29_export::copperlists_dump;
use cu29_helpers::basic_copper_setup;
use std::path::{Path, PathBuf};

// To enable resim, it is just your regular macro with sim_mode true
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct BalanceBotReSim {}

fn default_callback(step: default::SimStep) -> SimOverride {
    match step {
        // Don't let the real task execute process and override with our logic.
        default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::Motor(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn run_one_copperlist(
    copper_app: &mut BalanceBotReSim,
    robot_clock: &mut RobotClockMock,
    copper_list: CopperList<default::CuMsgs>,
) {
    // Sync the copper clock to the simulated physics clock.
    let msgs = &copper_list.msgs;

    // Simulate what the sim is doing here
    // TODO: set that at every step instead of just at the CL level like the sim.
    let CuDuration(process_time) = msgs
        .get_balpos_output()
        .metadata
        .process_time
        .start
        .unwrap();
    robot_clock.set_value(process_time);

    let mut sim_callback = move |step: default::SimStep| -> SimOverride {
        match step {
            default::SimStep::Balpos(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_balpos_output().clone();
                SimOverride::ExecutedBySim
            }
            default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
            default::SimStep::Railpos(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_railpos_output().clone();
                SimOverride::ExecutedBySim
            }
            default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
            default::SimStep::Motor(CuTaskCallbackState::Process(input, output)) => {
                // Here the Sim change stuff in the message, just redo the same thing
                // so we can only show possible differences from copper's execution itself.
                let maybe_motor_actuation = input.payload();
                if let Some(motor_actuation) = maybe_motor_actuation {
                    if motor_actuation.power.is_nan() {
                        return SimOverride::ExecutedBySim;
                    }
                    let force_magnitude = motor_actuation.power * 2.0;
                    output
                        .metadata
                        .set_status(format!("Applied force: {force_magnitude}"));
                }
                SimOverride::ExecutedBySim
            }
            default::SimStep::Motor(_) => SimOverride::ExecutedBySim,
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
    let logger_path = "logs/balanceresim.copper";
    let (robot_clock, mut robot_clock_mock) = RobotClock::mock();
    let copper_ctx = basic_copper_setup(
        &PathBuf::from(logger_path),
        LOG_SLAB_SIZE,
        true,
        Some(robot_clock.clone()),
    )
    .expect("Failed to setup logger.");

    let mut copper_app = BalanceBotReSimBuilder::new()
        .with_context(&copper_ctx)
        .with_sim_callback(&mut default_callback)
        .build()
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
    let iter = copperlists_dump::<default::CuMsgs>(&mut reader);
    for entry in iter {
        println!("{entry:#?}");
        run_one_copperlist(&mut copper_app, &mut robot_clock_mock, entry);
    }
    copper_app
        .stop_all_tasks(&mut default_callback)
        .expect("Failed to start all tasks.");
}
