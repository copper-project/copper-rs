pub mod tasks;
mod world;

use crate::world::{Cart, Rod};
use avian3d::math::Vector;
use avian3d::prelude::{ExternalForce, Physics};
use bevy::prelude::*;
use cu29::clock::{RobotClock, RobotClockMock};
use cu29::simulation::{CuTaskCallbackState, SimOverride};
use cu29_derive::copper_runtime;
use cu29_helpers::{basic_copper_setup, CopperContext};
use cu29_log_derive::debug;
use cu_ads7883_new::ADSReadingPayload;
use cu_rp_encoder::EncoderPayload;
use std::path::PathBuf;

// To enable sim, it is just your regular macro with sim_mode true
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct BalanceBotSim {}

// Encapsulate the Copper mock clock as a Bevy resource
#[derive(Resource)]
struct CopperMockClock {
    clock: RobotClockMock,
}

// Encapsulate the Copper runtime as a Bevy resource
#[derive(Resource)]
struct Copper {
    _copper_ctx: CopperContext,
    copper_app: BalanceBotSim,
}

// Some default do nothing for out sources and sinks.
// This is queried by the sim runtime to know what it should do
// at any stage passed as a parameter to the enums
// see the more complete example below for the runtime part.
fn default_callback(step: SimStep) -> SimOverride {
    match step {
        // Don't let the real task execute process and override with our logic.
        SimStep::Balpos(_) => SimOverride::ExecutedBySim,
        SimStep::Railpos(_) => SimOverride::ExecutedBySim,
        SimStep::Motor(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

// a system callback from bevy to setup the copper part of the house.
fn setup_copper(mut commands: Commands) {
    const LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);
    let logger_path = "balance.copper";
    // here we set up a mock clock so the simulation can take control of it.
    let (robot_clock, mock) = RobotClock::mock();
    let copper_ctx = basic_copper_setup(
        &PathBuf::from(logger_path),
        LOG_SLAB_SIZE,
        false,
        Some(robot_clock.clone()),
    )
    .expect("Failed to setup logger.");
    debug!(
        "Logger created at {}. This is a simulation.",
        path = logger_path
    );

    let mut copper_app = BalanceBotSim::new(
        robot_clock.clone(),
        copper_ctx.unified_logger.clone(),
        &mut default_callback,
    )
    .expect("Failed to create runtime.");
    copper_app
        .start_all_tasks(&mut default_callback)
        .expect("Failed to start all tasks.");

    // save all this in resources so we can grab them later during the simulation.
    commands.insert_resource(CopperMockClock { clock: mock });
    commands.insert_resource(Copper {
        _copper_ctx: copper_ctx,
        copper_app,
    });
}

/// This is a bevy system to trigger the Copper application to run one iteration.
/// We can query the state of the world from here and pass it to the Copper application.
fn run_copper_callback(
    mut query_set: ParamSet<(
        Query<(&mut Transform, &mut ExternalForce), With<Cart>>,
        Query<&Transform, With<Rod>>,
    )>,
    physics_time: Res<Time<Physics>>,
    robot_clock: ResMut<CopperMockClock>,
    mut copper_ctx: ResMut<Copper>,
) {
    // Check if the Cart spawned; if not, return early.
    if query_set.p0().is_empty() {
        return;
    }

    // Sync the copper clock to the simulated physics clock.
    robot_clock
        .clock
        .set_value(physics_time.elapsed().as_nanos() as u64);
    let mut sim_callback = move |step: SimStep<'_>| -> SimOverride {
        match step {
            SimStep::Balpos(CuTaskCallbackState::Process(_, output)) => {
                // so here we jump when the balpos source (the adc giving the rod position) is called
                // we get the physical state of the work and inject back to copper what would the sensor read
                let bindings = query_set.p1();
                let rod_transform = bindings.single();

                let (_roll, _pitch, yaw) = rod_transform.rotation.to_euler(EulerRot::YXZ);

                let mut angle_radians = yaw - std::f32::consts::FRAC_PI_2; // Offset by 90 degrees -> 0 is lying on the left.
                if angle_radians < 0.0 {
                    angle_radians += 2.0 * std::f32::consts::PI;
                }

                // Convert the angle from radians to the actual adc value from the sensor
                let analog_value = (angle_radians / (2.0 * std::f32::consts::PI) * 4096.0) as u16;
                output.set_payload(ADSReadingPayload { analog_value });
                output.metadata.tov = robot_clock.clock.now().into();
                SimOverride::ExecutedBySim
            }
            SimStep::Balpos(_) => SimOverride::ExecutedBySim,
            SimStep::Railpos(CuTaskCallbackState::Process(_, output)) => {
                // Here same thing for the rail encoder.
                let bindings = query_set.p0();
                let (cart_transform, _) = bindings.single();
                let ticks = (cart_transform.translation.x * 2000.0) as i32;
                output.set_payload(EncoderPayload { ticks });
                output.metadata.tov = robot_clock.clock.now().into();
                SimOverride::ExecutedBySim
            }
            SimStep::Railpos(_) => SimOverride::ExecutedBySim,
            SimStep::Motor(CuTaskCallbackState::Process(input, _)) => {
                // And now when copper wants to use the motor
                // we apply a force in the simulation.
                let mut bindings = query_set.p0();
                let (_, mut cart_force) = bindings.single_mut();
                let maybe_motor_actuation = input.payload();
                if let Some(motor_actuation) = maybe_motor_actuation {
                    if motor_actuation.power.is_nan() {
                        cart_force.apply_force(Vector::ZERO);
                        return SimOverride::ExecutedBySim;
                    }
                    let force_magnitude = motor_actuation.power * 2.0; // 4_000.0;
                                                                       // let current_force = cart_force.force(); // Get existing force which includes gravity
                    let new_force = /*current_force */ Vector::new(force_magnitude as f64, 0.0, 0.0);
                    cart_force.apply_force(new_force);
                } else {
                    cart_force.apply_force(Vector::ZERO);
                }
                SimOverride::ExecutedBySim
            }
            SimStep::Motor(_) => SimOverride::ExecutedBySim,
            _ => SimOverride::ExecuteByRuntime,
        }
    };
    copper_ctx
        .copper_app
        .run_one_iteration(&mut sim_callback)
        .expect("Failed to run application.");
}

fn stop_copper_on_exit(mut exit_events: EventReader<AppExit>, mut copper_ctx: ResMut<Copper>) {
    for _ in exit_events.read() {
        copper_ctx
            .copper_app
            .stop_all_tasks(&mut default_callback) // let the tasks clean themselves up
            .expect("Failed to stop all tasks.");
    }
}

fn main() {
    let mut world = App::new();

    // minimal setup to load the assets
    world.add_plugins(DefaultPlugins);

    // setup everything that is simulation specific.
    let world = world::build_world(&mut world);

    // setup all the systems related to copper and the glue logic.
    world.add_systems(Startup, setup_copper);
    world.add_systems(Update, run_copper_callback);
    world.add_systems(PostUpdate, stop_copper_on_exit);
    world.run();
}
