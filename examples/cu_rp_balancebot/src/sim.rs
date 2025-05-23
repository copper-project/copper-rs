pub mod tasks;
mod world;

use crate::world::{Cart, Rod};
use avian3d::math::Vector;
use avian3d::prelude::{ExternalForce, Physics};
use bevy::asset::UnapprovedPathMode;
use bevy::prelude::*;
use bevy::render::RenderPlugin;
// disembiguation as there is also a bevy::prelude::debug
use cu29::prelude::debug;
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use cu_ads7883_new::ADSReadingPayload;
use cu_rp_encoder::EncoderPayload;
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

// a system callback from bevy to setup the copper part of the house.
fn setup_copper(mut commands: Commands) {
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);
    let logger_path = "logs/balance.copper";
    if let Some(parent) = Path::new(logger_path).parent() {
        if !parent.exists() {
            fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
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
#[allow(clippy::type_complexity)]
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
    let mut sim_callback = move |step: default::SimStep| -> SimOverride {
        match step {
            default::SimStep::Balpos(CuTaskCallbackState::Process(_, output)) => {
                // so here we jump when the balpos source (the adc giving the rod position) is called
                // we get the physical state of the work and inject back to copper what would the sensor read
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
                output.metadata.tov = robot_clock.clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
            default::SimStep::Railpos(CuTaskCallbackState::Process(_, output)) => {
                // Here same thing for the rail encoder.
                let bindings = query_set.p0();
                let (cart_transform, _) = bindings.single().expect("Failed to get cart transform");
                let ticks = (cart_transform.translation.x * 2000.0) as i32;
                output.set_payload(EncoderPayload { ticks });
                output.metadata.tov = robot_clock.clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
            default::SimStep::Motor(CuTaskCallbackState::Process(input, output)) => {
                // And now when copper wants to use the motor
                // we apply a force in the simulation.
                let mut bindings = query_set.p0();
                let (_, mut cart_force) = bindings.single_mut().expect("Failed to get cart force");
                let maybe_motor_actuation = input.payload();
                if let Some(motor_actuation) = maybe_motor_actuation {
                    if motor_actuation.power.is_nan() {
                        cart_force.apply_force(Vector::ZERO);
                        return SimOverride::ExecutedBySim;
                    }
                    let force_magnitude = motor_actuation.power * 2.0;
                    let new_force = Vector::new(force_magnitude, 0.0, 0.0);
                    cart_force.apply_force(new_force);
                    output
                        .metadata
                        .set_status(format!("Applied force: {force_magnitude}"));
                    SimOverride::ExecutedBySim
                } else {
                    cart_force.apply_force(Vector::ZERO);
                    SimOverride::Errored("Safety Mode.".into())
                }
            }
            default::SimStep::Motor(_) => SimOverride::ExecutedBySim,
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
        println!("Exiting copper");
        copper_ctx
            .copper_app
            .stop_all_tasks(&mut default_callback) // let the tasks clean themselves up
            .expect("Failed to stop all tasks.");
    }
}

fn main() {
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

    let default_plugin = DefaultPlugins
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
        });

    world.add_plugins(default_plugin);

    // setup everything that is simulation specific.
    let world = world::build_world(&mut world);

    // setup all the systems related to copper and the glue logic.
    world.add_systems(Startup, setup_copper);
    world.add_systems(Update, run_copper_callback);
    world.add_systems(PostUpdate, stop_copper_on_exit);
    world.run();
}
