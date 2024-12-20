use avian3d::prelude::*;
use bevy::app::App;
use bevy::core_pipeline::fxaa::Fxaa;
use bevy::core_pipeline::Skybox;
use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::pbr::{
    DefaultOpaqueRendererMethod, ScreenSpaceReflectionsBundle, ScreenSpaceReflectionsSettings,
};
use bevy::prelude::*;
use iyes_perf_ui::entries::PerfUiCompleteBundle;
use iyes_perf_ui::PerfUiPlugin;

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

#[derive(Resource)]
struct SetupCompleted(bool);

fn setup_entities(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut setup_completed: ResMut<SetupCompleted>,
) {
    if setup_completed.0 {
        return;
    }

    setup_completed.0 = true; // Mark as completed
}
pub fn build_world(app: &mut App) -> &mut App {
    let app = app
        .insert_resource(Msaa::Off)
        .add_plugins(bevy::diagnostic::FrameTimeDiagnosticsPlugin)
        .add_plugins(bevy::diagnostic::EntityCountDiagnosticsPlugin)
        .insert_resource(DefaultOpaqueRendererMethod::deferred())
        .insert_resource(SimulationState::Running)
        .insert_resource(CameraControl {
            rotate_sensitivity: 0.05,
            zoom_sensitivity: 30.0,
            move_sensitivity: 0.1,
        })
        .insert_resource(Gravity::default())
        .insert_resource(Time::<Physics>::default())
        .add_systems(Startup, setup_scene)
        .add_systems(Startup, setup_ui)
        .add_systems(Update, setup_entities) // Wait for the scene to load fully
        .add_systems(Update, toggle_simulation_state)
        .add_systems(Update, camera_control_system)
        .add_systems(Update, update_physics);
    app.add_plugins(PerfUiPlugin);

    app
}

// pub const BASE_ASSETS_URL: &str = "https://cdn.copper-robotics.com/";

fn setup_scene(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Precache where the user executes the binary
    // let cache = Cache::builder()
    //     .progress_bar(Some(ProgressBar::Full))
    //     .build()
    //     .expect("Failed to create the file cache.");

    // let balance_bot_hashed = cache
    //     .cached_path(format!("{}{}", BASE_ASSETS_URL, BALANCEBOT).as_str())
    //     .expect("Failed to download and cache balancebot.glb.");
    // let balance_bot_path = balance_bot_hashed.parent().unwrap().join(BALANCEBOT);

    // create_symlink(
    //     balance_bot_hashed.to_str().unwrap(),
    //     balance_bot_path.to_str().unwrap(),
    // )
    // .expect("Failed to create symlink to balancebot.glb.");

    // let skybox_path_hashed = cache
    //     .cached_path(format!("{}{}", BASE_ASSETS_URL, SKYBOX).as_str())
    //     .expect("Failed download and cache skybox.ktx2.");

    // let skybox_path = skybox_path_hashed.parent().unwrap().join(SKYBOX);
    // create_symlink(
    //     skybox_path_hashed.to_str().unwrap(),
    //     skybox_path.to_str().unwrap(),
    // )
    // .expect("Failed to create symlink to skybox.ktx2.");

    // let diffuse_map_path_hashed = cache
    //     .cached_path(format!("{}{}", BASE_ASSETS_URL, DIFFUSE_MAP).as_str())
    //     .expect("Failed download and cache diffuse_map.");

    // let diffuse_map_path = diffuse_map_path_hashed.parent().unwrap().join(DIFFUSE_MAP);
    // create_symlink(
    //     diffuse_map_path_hashed.to_str().unwrap(),
    //     diffuse_map_path.to_str().unwrap(),
    // )
    // .expect("Failed to create symlink to diffuse_map.ktx2.");

    // Load the resources
    let scene_handle = asset_server.load(GltfAssetLabel::Scene(0).from_asset("house.glb#scene0"));
    let skybox_handle = asset_server.load("skybox.ktx2");
    let diffuse_map_handle = asset_server.load("diffuse_map.ktx2");
    let specular_map_handle = skybox_handle.clone(); // some quirk

    // Fiat Lux
    commands.insert_resource(AmbientLight {
        color: Color::srgb_u8(210, 220, 240),
        brightness: 1.0,
    });

    // load the house
    commands.spawn(SceneBundle {
        scene: scene_handle,
        ..default()
    });

    // Spawn the camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-10.0, 2.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
            camera: Camera {
                hdr: true,
                ..default()
            },
            ..default()
        },
        Skybox {
            image: skybox_handle,
            brightness: 1000.0,
        },
        EnvironmentMapLight {
            diffuse_map: diffuse_map_handle,
            specular_map: specular_map_handle,
            intensity: 600.0,
        },
        ScreenSpaceReflectionsBundle {
            settings: ScreenSpaceReflectionsSettings {
                perceptual_roughness_threshold: 0.85, // Customize as needed
                thickness: 0.01,
                linear_steps: 128,
                linear_march_exponent: 2.0,
                bisection_steps: 8,
                use_secant: true,
            },
            ..default()
        },
        Fxaa::default(),
    ));

    // add the delayed setup flag
    commands.insert_resource(SetupCompleted(false));

    // add a ground
    // ground_setup(&mut commands, &mut meshes, &mut materials);
}

fn setup_ui(mut commands: Commands) {
    #[cfg(target_os = "macos")]
    let instructions = "WASD / QE\nControl-Click + Drag\nClick + Drag\nScrolling\nSpace\nR";
    #[cfg(not(target_os = "macos"))]
    let instructions = "WASD / QE\nMiddle-Click + Drag\nClick + Drag\nScroll Wheel\nSpace\nR";

    commands
        .spawn((NodeBundle {
            style: Style {
                position_type: PositionType::Absolute,
                bottom: Val::Px(5.0),
                right: Val::Px(5.0),
                padding: UiRect::new(Val::Px(15.0), Val::Px(15.0), Val::Px(10.0), Val::Px(10.0)),
                column_gap: Val::Px(10.0),

                flex_direction: FlexDirection::Row,
                justify_content: JustifyContent::SpaceBetween,
                border: UiRect::all(Val::Px(2.0)),
                ..default()
            },
            background_color: Color::srgba(0.25, 0.41, 0.88, 0.7).into(),
            border_color: Color::srgba(0.8, 0.8, 0.8, 0.7).into(),
            border_radius: BorderRadius::all(Val::Px(10.0)),
            ..default()
        },))
        .with_children(|parent| {
            // Left column
            parent.spawn(TextBundle::from_section(
                "Move\nNavigation\nInteract\nZoom\nPause/Resume\nReset",
                TextStyle {
                    font_size: 12.0,
                    color: Color::srgb(1.0, 0.8, 0.2), // Golden color
                    ..default()
                },
            ));

            // Right column
            parent.spawn(TextBundle::from_section(
                instructions,
                TextStyle {
                    font_size: 12.0,
                    color: Color::WHITE,
                    ..default()
                },
            ));
        });
    commands.spawn(PerfUiCompleteBundle::default());
}

// Space to start / stop the simulation
fn toggle_simulation_state(
    mut state: ResMut<SimulationState>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
) {
    if keyboard_input.just_pressed(KeyCode::Space) {
        if *state == SimulationState::Running {
            *state = SimulationState::Paused;
        } else {
            *state = SimulationState::Running;
        }
    }
}

// Pause / Unpause the physics time.
fn update_physics(state: Res<SimulationState>, mut time: ResMut<Time<Physics>>) {
    if *state == SimulationState::Paused {
        time.pause();
        return;
    }
    time.unpause();
}

fn camera_control_system(
    control: Res<CameraControl>,
    keys: Res<ButtonInput<KeyCode>>,
    mut scroll_evr: EventReader<MouseWheel>,
    mut mouse_motion: EventReader<MouseMotion>,
    mut query: Query<&mut Transform, With<Camera>>,
    time: Res<Time>,
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
    if mouse_button_input.pressed(MouseButton::Middle) {
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

    #[cfg(target_os = "macos")]
    let mouse_button = MouseButton::Right;
    #[cfg(not(target_os = "macos"))]
    let mouse_button = MouseButton::Middle;

    if mouse_button_input.pressed(mouse_button) {
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
