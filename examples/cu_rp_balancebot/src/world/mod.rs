use avian3d::prelude::*;
use bevy::core_pipeline::fxaa::Fxaa;
use bevy::core_pipeline::Skybox;
use bevy::input::{
    keyboard::KeyCode,
    mouse::{MouseButton, MouseMotion, MouseWheel},
};
use bevy::pbr::{DefaultOpaqueRendererMethod, ScreenSpaceReflections};
use bevy::prelude::*;
use cached_path::{Cache, Error as CacheError, ProgressBar};
#[cfg(feature = "perf-ui")]
use iyes_perf_ui::prelude::{PerfUiAllEntries, PerfUiPlugin};
use std::path::{Path, PathBuf}; // Import PathBuf

use std::{fs, io};

pub const BALANCEBOT: &str = "balancebot.glb";
pub const SKYBOX: &str = "skybox.ktx2";
pub const DIFFUSE_MAP: &str = "diffuse_map.ktx2";

const TABLE_HEIGHT: f32 = 0.724;
const RAIL_WIDTH: f32 = 0.55; // 55cm
const RAIL_HEIGHT: f32 = 0.02;
const RAIL_DEPTH: f32 = 0.06;

const CART_WIDTH: f32 = 0.040;
const CART_HEIGHT: f32 = 0.045; // The mid rail is 1cm above the bottom rail, and the cart is 35mm tall.
const CART_DEPTH: f32 = RAIL_DEPTH;

const ROD_WIDTH: f32 = 0.007; // 7mm
const ROD_HEIGHT: f32 = 0.50; // 50cm
const ROD_DEPTH: f32 = ROD_WIDTH;

const AXIS_LENGTH: f32 = 0.02;

const STEEL_DENSITY: f32 = 7800.0; // kg/m^3
const ALUMINUM_DENSITY: f32 = 2700.0; // kg/m^3

#[allow(dead_code)]
const ROD_VOLUME: f32 = ROD_WIDTH * ROD_HEIGHT * ROD_DEPTH;

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

#[derive(Component)]
pub struct Cart;

#[derive(Component)]
pub struct Rod;

pub fn build_world(app: &mut App) -> &mut App {
    let app = app
        .add_plugins(MeshPickingPlugin)
        .add_plugins(PhysicsPlugins::default().with_length_unit(1000.0))
        // we want Bevy to measure these values for us:
        .add_plugins(bevy::diagnostic::FrameTimeDiagnosticsPlugin::default())
        .add_plugins(bevy::diagnostic::EntityCountDiagnosticsPlugin)
        .insert_resource(DefaultOpaqueRendererMethod::deferred())
        .insert_resource(SimulationState::Running)
        .insert_resource(CameraControl {
            rotate_sensitivity: 0.05,
            zoom_sensitivity: 3.5,
            move_sensitivity: 0.01,
        })
        .insert_resource(Gravity::default())
        .insert_resource(Time::<Physics>::default())
        .add_systems(Startup, setup_scene)
        .add_systems(Startup, setup_ui)
        .add_systems(Update, setup_entities) // Wait for the cart entity to be loaded
        .add_systems(Update, toggle_simulation_state)
        .add_systems(Update, camera_control_system)
        .add_systems(Update, update_physics)
        .add_systems(Update, global_cart_drag_listener)
        .add_systems(PostUpdate, reset_sim);

    #[cfg(feature = "perf-ui")]
    app.add_plugins(PerfUiPlugin);

    app
}

fn ground_setup(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let plane_mesh = meshes.add(Plane3d::default().mesh().size(2.0, 2.0));
    // Chessboard Plane
    let black_material = materials.add(StandardMaterial {
        base_color: Color::BLACK,
        reflectance: 0.4,
        perceptual_roughness: 0.4,
        ..default()
    });

    let white_material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        reflectance: 0.4,
        perceptual_roughness: 0.4,
        ..default()
    });

    for x in -3..4 {
        for z in -3..4 {
            commands.spawn((
                Mesh3d(plane_mesh.clone()),
                MeshMaterial3d(if (x + z) % 2 == 0 {
                    black_material.clone()
                } else {
                    white_material.clone()
                }),
                Transform::from_xyz(x as f32 * 2.0, -TABLE_HEIGHT, z as f32 * 2.0),
            ));
        }
    }
}

fn create_symlink(src: &str, dst: &str) -> io::Result<()> {
    let dst_path = Path::new(dst);

    if dst_path.exists() {
        fs::remove_file(dst_path)?;
    }

    #[cfg(unix)]
    {
        std::os::unix::fs::symlink(src, dst)
    }

    #[cfg(windows)]
    {
        std::os::windows::fs::symlink_file(src, dst)
    }
}

/// Tries to get the asset path using the online cache first.
/// If that fails due to a network error, falls back to the offline cache.
fn get_asset_path(
    online_cache: &Cache,
    offline_cache: &Cache,
    asset_url: &str,
    asset_name: &str, // For logging purposes
) -> Result<PathBuf, CacheError> {
    match online_cache.cached_path(asset_url) {
        Ok(path) => Ok(path),
        Err(err) => {
            // Check if the error is network-related
            if matches!(
                err,
                CacheError::HttpError(_) | CacheError::IoError(_) | CacheError::ResourceNotFound(_)
            ) {
                warn!(
                    "Failed to fetch latest '{}' from network ({}). Attempting to use cached version.",
                    asset_name, err
                );
                // Fallback to offline cache
                offline_cache.cached_path(asset_url)
            } else {
                // Not a network error, propagate the original error
                Err(err)
            }
        }
    }
}
pub const BASE_ASSETS_URL: &str = "https://cdn.copper-robotics.com/";

fn setup_scene(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Precache where the user executes the binary

    let online_cache = Cache::builder()
        .progress_bar(Some(ProgressBar::Full))
        .build()
        .expect("Failed to create the online file cache.");

    let offline_cache = Cache::builder()
        .progress_bar(Some(ProgressBar::Full))
        .offline(true) // Force offline mode
        .build()
        .expect("Failed to create the offline file cache.");
    let balance_bot_url = format!("{BASE_ASSETS_URL}{BALANCEBOT}");
    let balance_bot_hashed =
        get_asset_path(&online_cache, &offline_cache, &balance_bot_url, BALANCEBOT)
            .expect("Failed to get balancebot.glb (online or cached).");
    let balance_bot_path = balance_bot_hashed.parent().unwrap().join(BALANCEBOT);

    create_symlink(
        balance_bot_hashed.to_str().unwrap(),
        balance_bot_path.to_str().unwrap(),
    )
    .expect("Failed to create symlink to balancebot.glb.");

    let skybox_url = format!("{BASE_ASSETS_URL}{SKYBOX}");
    let skybox_path_hashed = get_asset_path(&online_cache, &offline_cache, &skybox_url, SKYBOX)
        .expect("Failed to get skybox.ktx2 (online or cached).");

    let skybox_path = skybox_path_hashed.parent().unwrap().join(SKYBOX);
    create_symlink(
        skybox_path_hashed.to_str().unwrap(),
        skybox_path.to_str().unwrap(),
    )
    .expect("Failed to create symlink to skybox.ktx2.");

    let diffuse_map_url = format!("{BASE_ASSETS_URL}{DIFFUSE_MAP}");
    let diffuse_map_path_hashed =
        get_asset_path(&online_cache, &offline_cache, &diffuse_map_url, DIFFUSE_MAP)
            .expect("Failed to get diffuse_map.ktx2 (online or cached).");
    let diffuse_map_path = diffuse_map_path_hashed.parent().unwrap().join(DIFFUSE_MAP);
    create_symlink(
        diffuse_map_path_hashed.to_str().unwrap(),
        diffuse_map_path.to_str().unwrap(),
    )
    .expect("Failed to create symlink to diffuse_map.ktx2.");

    // Load the resources
    let scene_handle = asset_server.load(
        GltfAssetLabel::Scene(0).from_asset(format!("{}#scene0", balance_bot_path.display())),
    );
    let skybox_handle = asset_server.load(skybox_path);
    let diffuse_map_handle = asset_server.load(diffuse_map_path);
    let specular_map_handle = skybox_handle.clone(); // some quirk

    // Fiat Lux
    commands.insert_resource(AmbientLight {
        color: Color::srgb_u8(210, 220, 240),
        brightness: 1.0,
        affects_lightmapped_meshes: true,
    });

    // load the scene
    commands.spawn((SceneRoot(scene_handle),));

    // Spawn the camera
    commands.spawn((
        Camera3d::default(),
        Msaa::Off,
        Skybox {
            image: skybox_handle,
            brightness: 1000.0,
            ..default()
        },
        EnvironmentMapLight {
            diffuse_map: diffuse_map_handle,
            specular_map: specular_map_handle,
            intensity: 900.0,
            ..default()
        },
        ScreenSpaceReflections {
            perceptual_roughness_threshold: 0.85, // Customize as needed
            thickness: 0.01,
            linear_steps: 128,
            linear_march_exponent: 2.0,
            bisection_steps: 8,
            use_secant: true,
        },
        Fxaa::default(),
        Transform::from_xyz(-1.0, 0.1, 1.5).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // add the delayed setup flag
    commands.insert_resource(SetupCompleted(false));

    // add a ground
    ground_setup(&mut commands, &mut meshes, &mut materials);
}

fn setup_ui(mut commands: Commands) {
    #[cfg(target_os = "macos")]
    let instructions = "WASD / QE\nControl-Click + Drag\nClick + Drag\nScrolling\nSpace\nR";
    #[cfg(not(target_os = "macos"))]
    let instructions = "WASD / QE\nMiddle-Click + Drag\nClick + Drag\nScroll Wheel\nSpace\nR";

    commands
        .spawn((
            Node {
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
            BackgroundColor(Color::srgba(0.25, 0.41, 0.88, 0.7)),
            BorderColor(Color::srgba(0.8, 0.8, 0.8, 0.7)),
            BorderRadius::all(Val::Px(10.0)),
        ))
        .with_children(|parent| {
            // Left column
            parent.spawn((
                Text::new("Move\nNavigation\nInteract\nZoom\nPause/Resume\nReset"),
                TextFont {
                    font_size: 12.0,
                    ..default()
                },
                TextColor(Color::srgba(0.25, 0.25, 0.75, 1.0)), // Golden color
            ));

            // Right column
            parent.spawn((
                Text::new(instructions),
                TextFont {
                    font_size: 12.0,
                    ..default()
                },
                TextColor(Color::WHITE),
            ));
        });

    #[cfg(feature = "perf-ui")]
    commands.spawn(PerfUiAllEntries::default());
}

// This needs to match an object / parent object name in the GLTF file (in blender this is the object name).
const CART_GLTF_ASSET_NAME: &str = "Cart";

fn try_to_find_cart_entity(query: Query<(Entity, &Name), Without<Cart>>) -> Option<Entity> {
    if let Some((cart_entity, _)) = query
        .iter()
        .find(|(_, name)| name.as_str() == CART_GLTF_ASSET_NAME)
    {
        return Some(cart_entity);
    }
    None
}

// This is a global drag listener that will move the cart when dragged.
// It is a bit of a hack, but it tries to find back the cart entity parent from any of the possible clickable children
fn global_cart_drag_listener(
    mut drag_events: EventReader<Pointer<Drag>>,
    parents: Query<(&ChildOf, Option<&Cart>)>,
    mut transforms: Query<&mut Transform, With<Cart>>,
    camera_query: Query<&Transform, (With<Camera>, Without<Cart>)>, // Add Without<Cart> to prevent conflicts
) {
    for drag in drag_events.read() {
        if drag.button != PointerButton::Primary {
            continue;
        }
        let mut entity = drag.target;
        while let Ok((parent, maybe_cart)) = parents.get(entity) {
            if maybe_cart.is_some() {
                break;
            }
            entity = parent.parent();
        }

        if let Ok(mut root_transform) = transforms.get_mut(entity) {
            let direction_multiplier = camera_query.iter().next().map_or(1.0, |camera_transform| {
                -camera_transform.forward().z.signum()
            });
            root_transform.translation.x += direction_multiplier * drag.delta.x / 500.0;
        }
    }
}

#[derive(Resource)]
struct SetupCompleted(bool);

#[derive(Bundle)]
struct CartBundle {
    external_force: ExternalForce,
    cart: Cart,
    mass_props: MassPropertiesBundle,
    dominance: Dominance,
    rigid_body: RigidBody,
    locked_axes: LockedAxes,
}

fn setup_entities(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    query: Query<(Entity, &Name), Without<Cart>>,
    mut setup_completed: ResMut<SetupCompleted>,
) {
    let SetupCompleted(completed) = *setup_completed;
    if completed {
        return;
    }

    // The cart entity will be loaded from the GLTF file, we need to wait for it to be loaded
    let cart_entity = match try_to_find_cart_entity(query) {
        Some(entity) => entity,
        None => return,
    };

    let cart_collider_model = Collider::cuboid(CART_WIDTH, CART_HEIGHT, CART_DEPTH);
    let cart_mass_props = MassPropertiesBundle::from_shape(&cart_collider_model, ALUMINUM_DENSITY); // It is a mix of emptiness and motor and steel.. overall some aluminum?

    commands.entity(cart_entity).insert(CartBundle {
        external_force: ExternalForce::new(Vec3::ZERO),
        cart: Cart,
        mass_props: cart_mass_props,
        dominance: Dominance(5),
        rigid_body: RigidBody::Dynamic,
        locked_axes: LockedAxes::new()
            .lock_translation_z()
            .lock_translation_y()
            .lock_rotation_x()
            .lock_rotation_y()
            .lock_rotation_z(),
    });

    let rail_entity = commands
        .spawn((
            RigidBody::Static, // The rail doesn't move
                               // CollisionLayers::new(0b01, 0b01),
        ))
        .id();

    // Connect the cart to the rail
    commands.spawn(
        PrismaticJoint::new(rail_entity, cart_entity)
            .with_free_axis(Vec3::X) // Allow movement along the X-axis
            .with_compliance(1e-9)
            .with_linear_velocity_damping(100.0)
            .with_angular_velocity_damping(10.0)
            .with_limits(-RAIL_WIDTH / 2.0, RAIL_WIDTH / 2.0)
            .with_local_anchor_1(Vec3::new(0.0, RAIL_HEIGHT / 2.0, 0.0)) // Rail top edge
            .with_local_anchor_2(Vec3::new(0.0, -CART_HEIGHT / 2.0, 0.0)),
    );

    let rod_collider_model = Collider::capsule(ROD_WIDTH / 2.0, ROD_HEIGHT);
    let rod_mass_props = MassPropertiesBundle::from_shape(&rod_collider_model, STEEL_DENSITY); // overall some aluminum?

    let rod_entity = commands
        .spawn((
            Name::new("Rod"),
            Rod,
            Mesh3d(meshes.add(Cylinder::new(ROD_WIDTH / 2.0, ROD_HEIGHT))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::WHITE,
                metallic: 0.8,
                perceptual_roughness: 0.1,
                ..default()
            })),
            Transform::from_xyz(
                0.0,
                ROD_HEIGHT / 2.0 /*+ RAIL_HEIGHT as f32*/ + CART_HEIGHT,
                CART_DEPTH / 2.0 + ROD_DEPTH / 2.0 + AXIS_LENGTH,
            ), // Start higher than the cart
            rod_mass_props,
            RigidBody::Dynamic,
            LockedAxes::new()
                .lock_translation_z()
                .lock_rotation_y()
                .lock_rotation_x(),
        ))
        .observe(on_drag_transform)
        .id();
    commands.spawn(
        RevoluteJoint::new(cart_entity, rod_entity)
            .with_compliance(1e-16)
            .with_linear_velocity_damping(10.0)
            .with_angular_velocity_damping(10.0)
            .with_aligned_axis(Vec3::Z) // Align the axis of rotation along the Z-axis
            .with_local_anchor_1(Vec3::new(
                0.0,
                CART_HEIGHT / 2.0,
                CART_DEPTH / 2.0 + ROD_DEPTH / 2.0 + AXIS_LENGTH,
            )) // aim at the center of the rod at the bottom
            .with_local_anchor_2(Vec3::new(0.0, -ROD_HEIGHT / 2.0, 0.0)), // Anchor on the rod (bottom)
    );

    // Light
    commands.spawn((
        PointLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(2.0, 4.0, 2.0),
    ));

    setup_completed.0 = true; // Mark as completed
}

fn on_drag_transform(drag: Trigger<Pointer<Drag>>, mut transforms: Query<&mut Transform>) {
    if drag.button != PointerButton::Primary {
        return;
    }
    if let Ok(mut transform) = transforms.get_mut(drag.target) {
        let pivot_world =
            transform.translation + transform.rotation * Vec3::new(0.0, -ROD_HEIGHT / 2.0, 0.0);
        transform.rotate_around(pivot_world, Quat::from_rotation_z(-drag.delta.x / 50.0));
    }
}

#[allow(clippy::type_complexity)]
fn reset_sim(
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<(
        Option<&Rod>,
        Option<&Cart>,
        Option<&mut Transform>, // Ensure transform is mutable
        Option<&mut ExternalForce>,
        Option<&mut LinearVelocity>,
        Option<&mut AngularVelocity>,
    )>,
) {
    if keys.just_pressed(KeyCode::KeyR) {
        for (
            rod_component,
            cart_component,
            transform,
            ext_force,
            linear_velocity,
            angular_velocity,
        ) in query.iter_mut()
        {
            if let Some(mut transform) = transform {
                if rod_component.is_some() {
                    transform.translation = Vec3::new(
                        0.0,
                        ROD_HEIGHT / 2.0 + CART_HEIGHT,
                        CART_DEPTH / 2.0 + ROD_DEPTH / 2.0 + AXIS_LENGTH,
                    );

                    transform.rotation = Quat::IDENTITY;
                }

                if cart_component.is_some() {
                    transform.translation.x = 0.0;
                }
            }
            if let Some(mut ext_force) = ext_force {
                ext_force.clear();
            }

            if let Some(mut velocity) = linear_velocity {
                *velocity = LinearVelocity::ZERO;
            }

            if let Some(mut angular_velocity) = angular_velocity {
                *angular_velocity = AngularVelocity::ZERO;
            }
        }
    }
}

/// Winged some type of orbital camera to explore around the robot.
fn camera_control_system(
    control: Res<CameraControl>,
    keys: Res<ButtonInput<KeyCode>>,
    mut scroll_evr: EventReader<MouseWheel>,
    mut mouse_motion: EventReader<MouseMotion>,
    mut query: Query<&mut Transform, With<Camera>>,
    time: Res<Time>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
) {
    let mut camera_transform = query.single_mut().expect("Failed to get camera transform");
    let focal_point = Vec3::ZERO; // Define the point to orbit around (usually the center of the scene)

    // Calculate the direction vector from the camera to the focal point
    let direction = camera_transform.translation - focal_point;
    let radius = direction.length(); // Distance from the focal point

    // Zoom with scroll
    for ev in scroll_evr.read() {
        let forward = camera_transform.forward(); // Store forward vector in a variable
        let zoom_amount = ev.y * control.zoom_sensitivity * time.delta_secs();
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
fn update_physics(state: Res<SimulationState>, mut time: ResMut<Time<Virtual>>) {
    if *state == SimulationState::Paused {
        time.pause();
        return;
    }
    time.unpause();
}
