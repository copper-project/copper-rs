pub mod tasks;
mod world;

use bevy::app::App;
use cu29::cutask::{CuTaskCallbackState, SimOverride};
use cu29_derive::copper_runtime;
use cu29_helpers::basic_copper_setup;
use cu29_log_derive::debug;
use cu_ads7883_new::ADSReadingPayload;
use cu_rp_encoder::EncoderPayload;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct BalanceBotSim {}

// preallocate a lot.
const SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);

fn sim_callback(step: SimStep) -> SimOverride {
    match step {
        // Don't let the real task execute process and override with our logic.
        SimStep::Balpos(CuTaskCallbackState::Process(_, output)) => {
            output.set_payload(ADSReadingPayload { analog_value: 3141 });
            SimOverride::ExecutedBySim
        }
        // Don't let the real task execute anything else from its lifecycle (start / stop ...)
        SimStep::Balpos(_) => SimOverride::ExecutedBySim,
        SimStep::Railpos(CuTaskCallbackState::Process(_, output)) => {
            output.set_payload(EncoderPayload { ticks: 0 });
            SimOverride::ExecutedBySim
        }
        SimStep::Railpos(_) => SimOverride::ExecutedBySim,
        SimStep::Motor(CuTaskCallbackState::Process(input, _)) => {
            println!("Motor input: {:?}", input.payload());
            SimOverride::ExecutedBySim
        }
        // All the other task at any lifecycle just ask copper to execute them normally.
        SimStep::Motor(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn main() {
    let mut world = App::new();
    let world = world::build_world(&mut world);
    world.run();

    static STOP_FLAG: AtomicBool = AtomicBool::new(false);
    let logger_path = "logs/balance.copper";
    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, false)
        .expect("Failed to setup logger.");
    debug!("Logger created at {}.", path = logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");

    let mut application = BalanceBotSim::new(
        clock.clone(),
        copper_ctx.unified_logger.clone(),
        sim_callback,
    )
    .expect("Failed to create runtime.");

    ctrlc::set_handler(move || {
        println!("Ctrl-C pressed. Stopping all tasks...");
        STOP_FLAG.store(true, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    debug!("Running... starting clock: {}.", clock.now());
    application
        .start_all_tasks()
        .expect("Failed to start all tasks.");
    while !STOP_FLAG.load(Ordering::SeqCst) {
        application
            .run_one_iteration()
            .expect("Failed to run application.");
    }
    application
        .stop_all_tasks()
        .expect("Failed to stop all tasks.");
    debug!("End of app: final clock: {}.", clock.now());
}
