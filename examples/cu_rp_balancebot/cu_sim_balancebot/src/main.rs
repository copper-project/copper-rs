use avian3d::prelude::*;
use bevy::input::{
    keyboard::KeyCode,
    mouse::{MouseButton, MouseMotion, MouseWheel},
};
use bevy::prelude::*;
use bevy_mod_picking::prelude::*;

#[derive(Resource)]
struct CameraControl {
    rotate_sensitivity: f32,
    zoom_sensitivity: f32,
    move_sensitivity: f32,
}

#[derive(Resource, PartialEq, Eq)]
enum SimulationState {
    Running,
    Paused,
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(low_latency_window_plugin()),
            DefaultPickingPlugins.build().disable::<RaycastBackend>(),
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
        ))
        .insert_resource(DebugPickingMode::Normal)
        .insert_resource(SimulationState::Paused)
        .insert_resource(AvianBackendSettings {
            require_markers: true,
        })
        .insert_resource(CameraControl {
            rotate_sensitivity: 0.005,
            zoom_sensitivity: 3.5,
            move_sensitivity: 0.01,
        })
        .insert_resource(Gravity(Vec3::new(0.0, -9.81, 0.0)))
        .insert_resource(Time::<Physics>::default())
        .add_systems(Startup, setup)
        .add_systems(Update, toggle_simulation_state)
        .add_systems(Update, camera_control_system)
        .add_systems(Update, update_physics)
        .run();
}

// fn setup(
//     mut commands: Commands,
//     mut meshes: ResMut<Assets<Mesh>>,
//     mut materials: ResMut<Assets<StandardMaterial>>,
// ) {
//     commands.spawn((
//         PbrBundle {
//             mesh: meshes.add(bevy_render::mesh::PlaneMeshBuilder::from_length(5.0)),
//             material: materials.add(Color::WHITE),
//             ..default()
//         },
//         Collider::cuboid(5.0, 0.01, 5.0),
//         RigidBody::Static,
//         PickableBundle::default(),
//         AvianPickable,
//     ));
//     commands.spawn((
//         PbrBundle {
//             mesh: meshes.add(Cuboid::default()),
//             material: materials.add(Color::WHITE),
//             transform: Transform::from_xyz(0.0, 1.5, 0.0),
//             ..default()
//         },
//         Collider::cuboid(1.0, 1.0, 1.0),
//         RigidBody::Dynamic,
//         PickableBundle::default(),
//         AvianPickable,
//     ));
//     commands.spawn(PointLightBundle {
//         point_light: PointLight {
//             shadows_enabled: true,
//             ..default()
//         },
//         transform: Transform::from_xyz(4.0, 8.0, 4.0),
//         ..default()
//     });
//     commands.spawn((
//         Camera3dBundle {
//             transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
//             ..default()
//         },
//         AvianPickable,
//     ));
// }

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let rail_entity = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(10.0, 0.2, 0.5)),
                material: materials.add(Color::WHITE),
                transform: Transform::from_xyz(0.0, 0.1, 0.0), // Lower the starting height
                ..default()
            },
            Collider::cuboid(10.0, 0.2, 0.5), // Thin, long rail
            RigidBody::Static,                // The rail doesn't move
        ))
        .id();
    // Spawn the cart (Dynamic, constrained to move along the rail)
    let cart_entity = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.5, 0.2, 0.5)),
                material: materials.add(Color::srgb(0.0, 0.0, 1.0)),
                transform: Transform::from_xyz(0.0, 0.3, 0.0), // Lower the starting height
                ..default()
            },
            Collider::cuboid(0.5, 0.2, 0.5), // Shorter height for the cart
            RigidBody::Dynamic,
        ))
        .id();

    commands.spawn(
        PrismaticJoint::new(rail_entity, cart_entity)
            .with_free_axis(Vec3::X)
            .with_local_anchor_1(Vec3::new(0.0, 0.2, 0.0)) // Rail top
            .with_local_anchor_2(Vec3::new(0.0, 0.0, 0.0)), // cart bottom
    );

    let rod_entity = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.1, 5.0, 0.1)),
                material: materials.add(Color::srgb(0.0, 1.0, 0.0)),
                transform: Transform::from_xyz(0.0, 2.8, 0.3), // Start higher than the cart
                ..default()
            },
            Collider::cuboid(0.1, 5.0, 0.1), // A thin rod for the pendulum
            RigidBody::Dynamic,              // The rod can move
        ))
        .id();

    // Create a revolute joint to connect the rod to the cart and allow swinging
    commands.spawn(
        RevoluteJoint::new(cart_entity, rod_entity)
            .with_aligned_axis(Vec3::Z) // Align the axis of rotation along the Z-axis
            .with_local_anchor_1(Vec3::new(0.0, 0.00, 0.25)) // Anchor on the cart (center top)
            .with_local_anchor_2(Vec3::new(0.0, -2.5, -0.06)), // Anchor on the rod (bottom)
    );

    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });

    // Camera
    commands.spawn((Camera3dBundle {
        transform: Transform::from_xyz(-5.0, 2.5, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    },));
}
fn camera_control_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut scroll_evr: EventReader<MouseWheel>,
    mut mouse_motion: EventReader<MouseMotion>,
    mut query: Query<&mut Transform, With<Camera>>,
    time: Res<Time>,
    control: Res<CameraControl>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
) {
    let mut camera_transform = query.single_mut();
    let focal_point = Vec3::ZERO; // Define the point to orbit around (usually the center of the scene)

    // Calculate the direction vector from the camera to the focal point
    let direction = camera_transform.translation - focal_point;
    let radius = direction.length(); // Distance from the focal point

    // Zoom with scroll
    for ev in scroll_evr.read() {
        let forward = camera_transform.forward(); // Store forward vector in a variable
        let zoom_amount = ev.y * control.zoom_sensitivity * time.delta_seconds();
        camera_transform.translation += forward * zoom_amount;
    }

    // Rotate camera around the focal point with right mouse button + drag
    if mouse_button_input.pressed(MouseButton::Right) {
        for ev in mouse_motion.read() {
            let yaw = Quat::from_rotation_y(-ev.delta.x * control.rotate_sensitivity);
            let pitch = Quat::from_rotation_x(-ev.delta.y * control.rotate_sensitivity);

            // Apply the rotation to the direction vector
            let new_direction = yaw * pitch * direction;

            // Update the camera position while maintaining the distance from the focal point
            camera_transform.translation = focal_point + new_direction.normalize() * radius;

            // Ensure the camera is always looking at the focal point
            camera_transform.look_at(focal_point, Vec3::Y);
        }
    }

    // Pan camera with middle mouse button + drag
    if mouse_button_input.pressed(MouseButton::Middle) {
        for ev in mouse_motion.read() {
            let right = camera_transform.right();
            let up = camera_transform.up();
            camera_transform.translation += right * -ev.delta.x * control.move_sensitivity;
            camera_transform.translation += up * ev.delta.y * control.move_sensitivity;
        }
    }

    let forward = if keys.pressed(KeyCode::KeyW) {
        camera_transform.forward() * control.move_sensitivity
    } else if keys.pressed(KeyCode::KeyS) {
        camera_transform.back() * control.move_sensitivity
    } else {
        Vec3::ZERO
    };

    let strafe = if keys.pressed(KeyCode::KeyA) {
        camera_transform.left() * control.move_sensitivity
    } else if keys.pressed(KeyCode::KeyD) {
        camera_transform.right() * control.move_sensitivity
    } else {
        Vec3::ZERO
    };

    let vertical = if keys.pressed(KeyCode::KeyQ) {
        Vec3::Y * control.move_sensitivity
    } else if keys.pressed(KeyCode::KeyE) {
        Vec3::NEG_Y * control.move_sensitivity
    } else {
        Vec3::ZERO
    };

    camera_transform.translation += forward + strafe + vertical;
}

fn toggle_simulation_state(
    mut state: ResMut<SimulationState>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
) {
    if keyboard_input.just_pressed(KeyCode::Space) {
        // Toggle between Running and Paused
        if *state == SimulationState::Running {
            *state = SimulationState::Paused;
        } else {
            *state = SimulationState::Running;
        }
    }
}

fn update_physics(state: Res<SimulationState>, mut time: ResMut<Time<Physics>>) {
    if *state == SimulationState::Paused {
        time.pause();
        return;
    }
    time.unpause();
}
