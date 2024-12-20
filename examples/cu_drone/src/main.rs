mod tasks;
mod world;

use avian3d::prelude::Physics;
use bevy::color::palettes::basic::{BLUE, RED};
use bevy::prelude::*;
use bevy::render::settings::{Backends, WgpuSettings};
use bevy::render::RenderPlugin;
use bevy_mod_raycast::prelude::Raycast;
use cu29::prelude::debug;
use cu29::prelude::*;
use cu29_helpers::{basic_copper_setup, CopperContext};
use cu_sim_depthsense::{compute_pointcloud, VirtPointCloud};
use std::path::{Path, PathBuf};

// To enable sim, it is just your regular macro with sim_mode true
#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct DroneSim {}

// Encapsulate the Copper mock clock as a Bevy resource
#[derive(Resource)]
struct CopperMockClock {
    clock: RobotClockMock,
}

// Encapsulate the Copper runtime as a Bevy resource
#[derive(Resource)]
struct Copper {
    _copper_ctx: CopperContext,
    copper_app: DroneSim,
}

fn default_callback(step: SimStep) -> SimOverride {
    SimOverride::ExecuteByRuntime
}

// a system callback from bevy to setup the copper part of the house.
fn setup_copper(mut commands: Commands) {
    const LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);
    let logger_path = "logs/drone.copper";
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

    let mut copper_app = DroneSim::new(
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

fn stop_copper_on_exit(mut exit_events: EventReader<AppExit>, mut copper_ctx: ResMut<Copper>) {
    for _ in exit_events.read() {
        copper_ctx
            .copper_app
            .stop_all_tasks(&mut default_callback) // let the tasks clean themselves up
            .expect("Failed to stop all tasks.");
    }
}

fn run_copper_callback(
    mut raycast: Raycast,
    physics_time: Res<Time<Physics>>,
    robot_clock: ResMut<CopperMockClock>,
    mut copper_ctx: ResMut<Copper>,
) {
    // Sync the copper clock to the simulated physics clock.
    robot_clock
        .clock
        .set_value(physics_time.elapsed().as_nanos() as u64);
    let mut sim_callback = move |step: SimStep<'_>| -> SimOverride {
        match step {
            SimStep::L(CuTaskCallbackState::Process(_, output)) => {
                output.metadata.tov = robot_clock.clock.now().into();
                let payload = output.payload_mut();
                *payload = Some(VirtPointCloud::default());
                compute_pointcloud(
                    Vec3::new(3.0, 0.0, 3.0),
                    &mut raycast,
                    robot_clock.clock.now(),
                    payload.as_mut().unwrap(),
                );
                SimOverride::ExecutedBySim
            }
            _ => SimOverride::ExecutedBySim,
        }
    };
    copper_ctx
        .copper_app
        .run_one_iteration(&mut sim_callback)
        .expect("Failed to run application.");
}

fn extract_transforms_from_geometry(vertices: Vec<Vec3>, normals: Vec<Vec3>) -> Vec<Transform> {
    let mut transforms = Vec::new();
    for (position, normal) in vertices.iter().zip(normals.iter()) {
        // Compute rotation from normal vector (example: aligning z-axis with normal)
        let rotation = Quat::from_rotation_arc(Vec3::Z, *normal);

        // Create a Transform for this position
        transforms.push(Transform {
            translation: *position,
            rotation,
            scale: Vec3::ONE, // Uniform scaling
        });
    }
    transforms
}

fn read_trajectory_fxb(path: &str) -> (Vec<Vec3>, Vec<Vec3>) {
    let path = Path::new(path);

    let file = std::fs::File::open(path).expect("Could not open the trajectory file");
    let root: fbx::File = fbx::File::read_from(file).expect("Could not read the trajectory file");
    let mut vertices: Option<Vec<f64>> = None;
    let mut normals: Option<Vec<f64>> = None;
    for section in root.children {
        if section.name == "Objects" {
            for objects_section in section.children {
                if objects_section.name == "Geometry" {
                    for geometry_section in objects_section.children {
                        if geometry_section.name == "Vertices" {
                            if let Some(vertices_data) = geometry_section.properties.get(0) {
                                if let fbx::Property::F64Array(vertices_array) = vertices_data {
                                    vertices = Some(vertices_array.clone());
                                }
                            }
                        } else if geometry_section.name == "LayerElementNormal" {
                            for normals_section in geometry_section.children {
                                if normals_section.name == "Normals" {
                                    if let Some(normals_data) = normals_section.properties.get(0) {
                                        if let fbx::Property::F64Array(normals_array) = normals_data
                                        {
                                            normals = Some(normals_array.clone());
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    let vertices: Vec<Vec3> = vertices
        .unwrap()
        .chunks(3)
        .map(|chunk| Vec3::new(chunk[0] as f32, chunk[1] as f32, chunk[2] as f32))
        .collect();
    let normals: Vec<Vec3> = normals
        .unwrap()
        .chunks(3)
        .map(|chunk| Vec3::new(chunk[0] as f32, chunk[1] as f32, chunk[2] as f32))
        .collect();

    // Rotation correction (Blender Z-up → Bevy Y-up)
    let rotation = Mat4::from_rotation_x(-std::f32::consts::FRAC_PI_2);

    // Adjusted translation: Swap Blender Y ↔ Z
    let translation = Mat4::from_translation(Vec3::new(-4.4253, -0.40866, 0.78979));

    // Apply rotation first, then translation
    let correction = rotation * translation;

    // * Mat4::from_rotation_z(std::f32::consts::PI);
    let vertices = vertices
        .iter()
        .map(|v| correction.transform_point3(*v))
        .collect();
    let normals = normals
        .iter()
        .map(|n| correction.transform_vector3(*n))
        .collect();

    (vertices, normals)
}

#[derive(Resource)]
struct DroneTransforms(Vec<Transform>);

#[derive(Resource)]
struct DroneState {
    current_index: usize,
    progress: f32, // Range [0.0, 1.0], representing interpolation progress
}

#[derive(Component)]
struct Drone;

fn setup_drone(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cone = meshes.add(Mesh::from(Cone {
        height: 0.5,
        radius: 0.2,
        ..default()
    }));
    // Create a red material
    let red_material = materials.add(StandardMaterial {
        base_color: Color::from(RED),
        emissive: LinearRgba::RED,
        ..default()
    });
    commands.spawn((
        PbrBundle {
            mesh: cone,
            material: red_material,
            transform: Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
            ..Default::default()
        },
        Drone,
    ));

    // Add resources
    commands.insert_resource(DroneState {
        current_index: 0,
        progress: 0.0,
    });
}

fn draw_trajectory(trajectory: Res<DroneTransforms>, mut gizmos: Gizmos) {
    for transform in &trajectory.0 {
        // Draw a sphere for each point in the trajectory
        gizmos.sphere(
            transform.translation,
            Quat::IDENTITY,
            0.01,
            Color::from(BLUE),
        );
    }
}

fn draw_trajectory_lines(trajectory: Res<DroneTransforms>, mut gizmos: Gizmos) {
    for i in 0..trajectory.0.len() - 1 {
        let start = trajectory.0[i].translation;
        let end = trajectory.0[i + 1].translation;

        // Draw a line between points
        gizmos.line(start, end, Color::from(BLUE));
    }
}

fn update_drone_transform(
    time: Res<Time>,
    drone_transforms: Res<DroneTransforms>,
    mut drone_state: ResMut<DroneState>,
    mut query: Query<&mut Transform, With<Drone>>,
) {
    let transforms = &drone_transforms.0;
    if transforms.len() < 2 {
        return; // Not enough points to interpolate
    }

    if drone_state.current_index >= transforms.len() - 1 {
        drone_state.current_index = 0; // Reached the final point
    }

    // Update interpolation progress
    drone_state.progress += time.delta_seconds() * 0.5; // Adjust speed as needed

    if drone_state.progress >= 1.0 {
        drone_state.progress = 0.0; // Reset progress for the next segment
        drone_state.current_index += 1; // Move to the next segment
    }

    let start = transforms[drone_state.current_index];
    let end = transforms[drone_state.current_index + 1];
    let t = drone_state.progress;
    let interpolated_translation = start.translation.lerp(end.translation, t);
    let interpolated_rotation = start.rotation.slerp(end.rotation, t);
    let interpolated_scale = start.scale.lerp(end.scale, t);

    for mut drone_transform in query.iter_mut() {
        drone_transform.translation = interpolated_translation;
        drone_transform.rotation = interpolated_rotation;
        drone_transform.scale = interpolated_scale;
    }
}

fn main() {
    let trajectory = read_trajectory_fxb("assets/powerloop.fbx");
    let transforms = extract_transforms_from_geometry(trajectory.0, trajectory.1);

    let mut world = App::new();

    world
        .insert_resource(DroneTransforms(transforms))
        .insert_resource(DroneState {
            current_index: 0,
            progress: 0.0,
        })
        .add_systems(Startup, setup_drone)
        .add_systems(Update, draw_trajectory)
        .add_systems(Update, draw_trajectory_lines)
        .add_systems(Update, update_drone_transform);

    let render_plugin = RenderPlugin {
        render_creation: WgpuSettings {
            backends: Some(Backends::VULKAN), // Force Vulkan backend when we know it is good.
            // This is to avoid some bugs when bevy tries out all the possible backends.
            ..Default::default()
        }
        .into(),
        ..Default::default()
    };

    let default_plugin = DefaultPlugins.set(render_plugin).set(WindowPlugin {
        primary_window: Some(Window {
            title: "Copper Simulator".into(),
            ..default()
        }),
        ..default()
    });
    // .set(LogPlugin {
    //     level: bevy::log::Level::DEBUG, // Set to debug
    //     filter: "bevy_gltf=debug,wgpu=warn,naga=error,bevy_render=warn,bevy_app=warn"
    //         .to_string(),
    //     custom_layer: |_| None,
    // });

    world.add_plugins(default_plugin);

    // setup everything that is simulation specific.
    let world = world::build_world(&mut world);

    // setup all the systems related to copper and the glue logic.
    world.add_systems(Startup, setup_copper);
    // world.add_systems(Update, run_copper_callback);
    world.add_systems(PostUpdate, stop_copper_on_exit);
    world.run();
}
