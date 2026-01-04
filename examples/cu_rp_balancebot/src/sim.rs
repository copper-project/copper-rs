mod motor_model;
pub mod tasks;
mod world;
use crate::world::{AppliedForce, Cart, DragState, Rod};
use avian3d::math::Vector;
use avian3d::prelude::{ConstantForce, Physics};
use bevy::asset::{AssetApp, UnapprovedPathMode};
use bevy::prelude::*;
use bevy::render::RenderPlugin;
use bevy::scene::ScenePlugin;
// disembiguation as there is also a bevy::prelude::debug
use cu_ads7883_new::ADSReadingPayload;
use cu_rp_encoder::EncoderPayload;
use cu29::prelude::debug;
#[allow(unused_imports)]
use cu29::prelude::error;
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::fs;
use std::path::{Path, PathBuf};

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
fn default_callback(step: default::SimStep) -> SimOverride {
    match step {
        // Don't let the real task execute process and override with our logic.
        default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
        default::SimStep::Motor(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

// a system callback from bevy to set up the copper part of the house.
fn setup_copper(mut commands: Commands) {
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);
    let logger_path = "logs/balance.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }

    // here we set up a mock clock so the simulation can take control of it.
    let (robot_clock, mock) = RobotClock::mock();
    let copper_ctx = basic_copper_setup(
        &PathBuf::from(logger_path),
        LOG_SLAB_SIZE,
        true,
        Some(robot_clock.clone()),
    )
    .expect("Failed to setup logger.");
    debug!(
        "Logger created at {}. This is a simulation.",
        path = logger_path
    );

    let mut copper_app = BalanceBotSimBuilder::new()
        .with_context(&copper_ctx)
        .with_sim_callback(&mut default_callback)
        .build()
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
/// ConstantForce persists across physics steps and mimics the old ExternalForce behavior.
#[allow(clippy::type_complexity)]
fn run_copper_callback(
    mut query_set: ParamSet<(
        Query<(&Transform, &mut ConstantForce, &mut AppliedForce), With<Cart>>,
        Query<&Transform, With<Rod>>,
    )>,
    physics_time: Res<Time<Physics>>,
    robot_clock: ResMut<CopperMockClock>,
    mut copper_ctx: ResMut<Copper>,
    drag_state: Res<DragState>,
    mut exit_writer: MessageWriter<AppExit>,
) {
    // Check if the Cart spawned; if not, return early.
    if query_set.p0().is_empty() {
        return;
    }

    // Sync the copper clock to the simulated physics clock.
    robot_clock
        .clock
        .set_value(physics_time.elapsed().as_nanos() as u64);
    let mut sim_callback = move |step: default::SimStep| -> SimOverride {
        match step {
            default::SimStep::Balpos(CuTaskCallbackState::Process(_, output)) => {
                // we run this code when the balpos source (the adc giving the rod position) is called
                // we get the physical state of the world and inject what the sensor would read back to copper
                let bindings = query_set.p1();
                let rod_transform = bindings.single().expect("Failed to get rod transform");

                let (_roll, _pitch, yaw) = rod_transform.rotation.to_euler(EulerRot::YXZ);

                let mut angle_radians = yaw - std::f32::consts::FRAC_PI_2; // Offset by 90 degrees -> 0 is lying on the left.
                if angle_radians < 0.0 {
                    angle_radians += 2.0 * std::f32::consts::PI;
                }

                // Convert the angle from radians to the actual adc value from the sensor
                let analog_value = (angle_radians / (2.0 * std::f32::consts::PI) * 4096.0) as u16;
                output.set_payload(ADSReadingPayload { analog_value });
                output.tov = robot_clock.clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
            default::SimStep::Railpos(CuTaskCallbackState::Process(_, output)) => {
                // Here same thing for the rail encoder.
                let mut bindings = query_set.p0();
                let (cart_transform, _, _) =
                    bindings.single_mut().expect("Failed to get cart transform");
                let ticks = (cart_transform.translation.x * 2000.0) as i32;
                output.set_payload(EncoderPayload { ticks });
                output.tov = robot_clock.clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
            default::SimStep::Motor(CuTaskCallbackState::Process(input, output)) => {
                // And now when copper wants to use the motor
                // we apply a force in the simulation.
                let mut bindings = query_set.p0();
                let (_, mut cart_force, mut applied_force) =
                    bindings.single_mut().expect("Failed to get cart force");
                let maybe_motor_actuation = input.payload();
                let override_motor = drag_state.override_motor;
                if override_motor {
                    if let Some(motor_actuation) = maybe_motor_actuation
                        && !motor_actuation.power.is_nan()
                    {
                        let total_mass = motor_model::total_mass_kg();
                        let force_magnitude =
                            motor_model::force_from_power(motor_actuation.power, total_mass);
                        output
                            .metadata
                            .set_status(format!("Applied force: {force_magnitude}"));
                    }
                    return SimOverride::ExecutedBySim;
                }
                if let Some(motor_actuation) = maybe_motor_actuation {
                    if motor_actuation.power.is_nan() {
                        cart_force.0 = Vector::ZERO;
                        return SimOverride::ExecutedBySim;
                    }
                    let total_mass = motor_model::total_mass_kg();
                    let force_magnitude =
                        motor_model::force_from_power(motor_actuation.power, total_mass);
                    let new_force = Vector::new(force_magnitude, 0.0, 0.0);
                    cart_force.0 = new_force;
                    applied_force.0 = new_force;
                    output
                        .metadata
                        .set_status(format!("Applied force: {force_magnitude}"));
                    SimOverride::ExecutedBySim
                } else {
                    cart_force.0 = Vector::ZERO;
                    SimOverride::Errored("Safety Mode.".into())
                }
            }
            default::SimStep::Motor(_) => SimOverride::ExecutedBySim,
            _ => SimOverride::ExecuteByRuntime,
        }
    };
    if let Err(err) = copper_ctx.copper_app.run_one_iteration(&mut sim_callback) {
        error!("Simulation stopped: {err}");
        eprintln!("Simulation stopped: {err}");
        exit_writer.write(AppExit::Success);
    }
}

fn stop_copper_on_exit(mut exit_events: MessageReader<AppExit>, mut copper_ctx: ResMut<Copper>) {
    for _ in exit_events.read() {
        println!("Exiting copper");
        copper_ctx
            .copper_app
            .stop_all_tasks(&mut default_callback) // let the tasks clean themselves up
            .expect("Failed to stop all tasks.");
    }
}

// this function creates the bevy::App used in both the integration test and the simulation
pub fn make_world(headless: bool) -> App {
    let mut world = App::new();
    #[cfg(target_os = "macos")]
    let render_plugin = RenderPlugin::default(); // This let macos pick their own backend.

    #[cfg(not(target_os = "macos"))]
    let render_plugin = RenderPlugin {
        render_creation: bevy::render::settings::WgpuSettings {
            backends: Some(bevy::render::settings::Backends::VULKAN), // Force Vulkan backend when we know it is good.
            // This is to avoid some bugs when bevy tries out all the possible backends.
            ..Default::default()
        }
        .into(),
        ..Default::default()
    };

    if headless {
        // add the minimal plugins as well as others needed for our simulation to run
        world.add_plugins((
            MinimalPlugins,
            AssetPlugin {
                unapproved_path_mode: UnapprovedPathMode::Allow,
                ..default()
            },
            ScenePlugin,
            ImagePlugin::default(),
        ));

        // MinimalPlugins doesn't register render asset types; initialize what the sim uses.
        world.init_asset::<Mesh>();
        world.init_asset::<StandardMaterial>();
        world.init_asset::<Font>();
        world.init_resource::<SceneSpawner>();
    } else {
        world.add_plugins(
            DefaultPlugins
                .set(render_plugin)
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Copper Simulator".into(),
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin {
                    unapproved_path_mode: UnapprovedPathMode::Allow,
                    ..default()
                }),
        );
    };

    // set up everything that is simulation specific.
    // this adds systems on the bevy side of things to handle things like the UI, keyboard inputs, etc.
    let simulation = world::build_world(&mut world, headless);

    // set up all the systems related to copper and the glue logic.
    simulation.add_systems(Startup, setup_copper);
    simulation.add_systems(Update, run_copper_callback);
    simulation.add_systems(PostUpdate, stop_copper_on_exit);
    world
}

fn main() {
    let mut world = make_world(false);
    world.run();
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_balancebot_runs() {
        let mut world = make_world(true);
        world.update();
    }
}
