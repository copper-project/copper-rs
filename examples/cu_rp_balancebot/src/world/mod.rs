use crate::motor_model;
use avian3d::math::Vector;
use avian3d::physics_transform::{PreSolveDeltaPosition, PreSolveDeltaRotation};
use avian3d::prelude::*;
use bevy::color::palettes::css::RED;
use bevy::core_pipeline::Skybox;
use bevy::input::{
    keyboard::KeyCode,
    mouse::{MouseButton, MouseMotion, MouseWheel},
};
use bevy::pbr::{DefaultOpaqueRendererMethod, ScreenSpaceReflections};
use bevy::prelude::*;
use bevy::ui::IsDefaultUiCamera;
use bevy_anti_alias::fxaa::Fxaa;
use cached_path::{Cache, Error as CacheError, ProgressBar};
use std::path::{Path, PathBuf}; // Import PathBuf

use std::{fs, io};

pub const BALANCEBOT: &str = "balancebot.glb";
pub const SKYBOX: &str = "skybox.ktx2";
pub const DIFFUSE_MAP: &str = "diffuse_map.ktx2";

const TABLE_HEIGHT: f32 = 0.724;
const RAIL_WIDTH: f32 = 0.55; // 55cm
const RAIL_HEIGHT: f32 = 0.02;
const RAIL_DEPTH: f32 = motor_model::CART_DEPTH;

const CART_WIDTH: f32 = motor_model::CART_WIDTH;
const CART_HEIGHT: f32 = motor_model::CART_HEIGHT; // The mid rail is 1cm above the bottom rail, and the cart is 35mm tall.
const CART_DEPTH: f32 = RAIL_DEPTH;

const ROD_WIDTH: f32 = motor_model::ROD_WIDTH; // 7mm
const ROD_HEIGHT: f32 = motor_model::ROD_HEIGHT; // 50cm
const ROD_DEPTH: f32 = motor_model::ROD_DEPTH;

const AXIS_LENGTH: f32 = 0.02;

const STEEL_DENSITY: f32 = motor_model::STEEL_DENSITY; // kg/m^3
const ALUMINUM_DENSITY: f32 = motor_model::ALUMINUM_DENSITY; // kg/m^3

#[allow(dead_code)]
const ROD_VOLUME: f32 = ROD_WIDTH * ROD_HEIGHT * ROD_DEPTH;

#[derive(Resource)]
struct DragControl {
    pixels_per_newton: f32,
    max_force: f32, // newtons
}
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

#[derive(Resource, Default)]
pub struct DragState {
    pub override_motor: bool,
    /// Currently active drag: (entity, force)
    pub active_drag: Option<(Entity, Vector)>,
}

#[derive(Component)]
pub struct Cart;

#[derive(Component)]
pub struct Rod;

#[derive(Component, Debug, Clone, Copy, Default)]
pub struct AppliedForce(pub Vector);

fn assert_reflected_types(type_registry: Res<AppTypeRegistry>) {
    let registry = type_registry.read();
    assert!(
        registry.get(std::any::TypeId::of::<Transform>()).is_some(),
        "Transform must be registered for scenes"
    );
    assert!(
        registry
            .get(std::any::TypeId::of::<GlobalTransform>())
            .is_some(),
        "GlobalTransform must be registered for scenes"
    );
    assert!(
        registry
            .get(std::any::TypeId::of::<TransformTreeChanged>())
            .is_some(),
        "TransformTreeChanged must be registered for scenes"
    );
    assert!(
        registry.get(std::any::TypeId::of::<Visibility>()).is_some(),
        "Visibility must be registered for scenes"
    );
    assert!(
        registry.get(std::any::TypeId::of::<Children>()).is_some(),
        "Children must be registered for scenes"
    );
    assert!(
        registry.get(std::any::TypeId::of::<ChildOf>()).is_some(),
        "ChildOf must be registered for scenes"
    );
}

pub fn build_world(app: &mut App, headless: bool) -> &mut App {
    app.init_resource::<AppTypeRegistry>();
    let app = app
        .add_plugins(PhysicsPlugins::default().with_length_unit(1000.0))
        // we want Bevy to measure these values for us:
        .add_plugins(bevy::diagnostic::FrameTimeDiagnosticsPlugin::default())
        .add_plugins(bevy::diagnostic::EntityCountDiagnosticsPlugin::default())
        .insert_resource(DefaultOpaqueRendererMethod::deferred())
        .insert_resource(SimulationState::Running)
        .insert_resource(CameraControl {
            rotate_sensitivity: 0.05,
            zoom_sensitivity: 3.5,
            move_sensitivity: 0.05,
        })
        .insert_resource(DragControl {
            pixels_per_newton: 100.,
            max_force: 10.,
        })
        .insert_resource(DragState::default())
        .insert_resource(Gravity::default())
        .insert_resource(Time::<Physics>::default())
        .add_systems(Startup, setup_scene)
        .add_systems(Startup, setup_ui)
        .add_systems(Update, setup_entities) // Wait for the cart entity to be loaded
        .add_systems(Update, update_physics)
        .add_systems(
            FixedPostUpdate,
            lock_cart_rotation.in_set(PhysicsSystems::Last),
        );

    // these will make a headless app crash, so only add them if we aren't headless
    if !headless {
        app.add_plugins(MeshPickingPlugin);
        app.add_systems(Update, toggle_simulation_state)
            .add_systems(Update, camera_control_system)
            .add_systems(Update, external_force_display)
            .add_systems(
                FixedPostUpdate,
                apply_drag_force.in_set(PhysicsSystems::Prepare),
            )
            .add_systems(PostUpdate, reset_sim);
    }

    // Ensure scene-spawned transforms are registered in the type registry.
    app.register_type::<Transform>();
    app.register_type::<GlobalTransform>();
    app.register_type::<TransformTreeChanged>();
    app.register_type::<Visibility>();
    app.register_type::<InheritedVisibility>();
    app.register_type::<ViewVisibility>();
    app.register_type::<Name>();
    app.register_type::<Children>();
    app.register_type::<ChildOf>();
    app.register_type::<Handle<Mesh>>();
    app.register_type::<Handle<StandardMaterial>>();
    app.register_type::<Handle<Image>>();
    app.register_type::<Handle<Gltf>>();
    app.register_type::<Handle<Scene>>();
    app.add_systems(Startup, assert_reflected_types);

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
        IsDefaultUiCamera,
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
    let instructions = "WASD / QE\nControl-Click + Drag\nClick + Drag\nScrolling\nSpace\nR\nF";
    #[cfg(not(target_os = "macos"))]
    let instructions = "WASD / QE\nMiddle-Click + Drag\nClick + Drag\nScroll Wheel\nSpace\nR\nF";

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
            BorderColor::all(Color::srgba(0.8, 0.8, 0.8, 0.7)),
            BorderRadius::all(Val::Px(10.0)),
        ))
        .with_children(|parent| {
            // Left column
            parent.spawn((
                Text::new("Move\nNavigation\nInteract\nZoom\nPause/Resume\nReset\nShow Forces"),
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
}

// This needs to match an object / parent object name in the GLTF file (in blender this is the object name).
const CART_GLTF_ASSET_NAME: &str = "Cart";

fn try_to_find_cart_entity(
    query: &Query<(Entity, &Name, &Transform), Without<Cart>>,
) -> Option<(Entity, Quat)> {
    query
        .iter()
        .find(|(_, name, _)| name.as_str() == CART_GLTF_ASSET_NAME)
        .map(|(cart_entity, _, transform)| (cart_entity, transform.rotation))
}

#[derive(Resource)]
struct SetupCompleted(bool);

#[derive(Bundle)]
struct CartBundle {
    applied_force: AppliedForce,
    cart: Cart,
    constant_force: ConstantForce,
    mass_props: MassPropertiesBundle,
    dominance: Dominance,
    rigid_body: RigidBody,
    locked_axes: LockedAxes,
}

#[derive(Component, Debug, Clone, Copy)]
struct CartRotationLock(Quat);

fn setup_entities(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    query: Query<(Entity, &Name, &Transform), Without<Cart>>,
    mut setup_completed: ResMut<SetupCompleted>,
) {
    let SetupCompleted(completed) = *setup_completed;
    if completed {
        return;
    }

    // The cart entity will be loaded from the GLTF file, we need to wait for it to be loaded
    let (cart_entity, cart_rotation) = match try_to_find_cart_entity(&query) {
        Some(cart) => cart,
        None => return,
    };

    let cart_collider_model = Collider::cuboid(CART_WIDTH, CART_HEIGHT, CART_DEPTH);
    let cart_mass_props = MassPropertiesBundle::from_shape(&cart_collider_model, ALUMINUM_DENSITY); // It is a mix of emptiness and motor and steel.. overall some aluminum?

    commands.entity(cart_entity).insert((
        CartBundle {
            applied_force: AppliedForce::default(),
            cart: Cart,
            constant_force: ConstantForce::default(),
            mass_props: cart_mass_props,
            dominance: Dominance(5),
            rigid_body: RigidBody::Dynamic,
            locked_axes: LockedAxes::new()
                .lock_translation_z()
                .lock_translation_y()
                .lock_rotation_x()
                .lock_rotation_y()
                .lock_rotation_z(),
        },
        CartRotationLock(cart_rotation),
    ));

    // let the cart be dragged too
    commands
        .entity(cart_entity)
        .observe(on_drag_start)
        .observe(on_drag)
        .observe(on_drag_end);

    let rail_entity = commands
        .spawn((
            RigidBody::Static, // The rail doesn't move
                               // CollisionLayers::new(0b01, 0b01),
        ))
        .id();

    // Connect the cart to the rail
    commands.spawn((
        PrismaticJoint::new(rail_entity, cart_entity)
            .with_slider_axis(Vec3::X) // Allow movement along the X-axis
            .with_align_compliance(0.0)
            .with_angle_compliance(0.0)
            .with_limit_compliance(0.0)
            .with_limits(-RAIL_WIDTH / 2.0, RAIL_WIDTH / 2.0)
            .with_local_anchor1(Vec3::new(0.0, RAIL_HEIGHT / 2.0, 0.0)) // Rail top edge
            .with_local_anchor2(Vec3::new(0.0, -CART_HEIGHT / 2.0, 0.0)),
        JointDamping {
            linear: 100.0,
            angular: 10.0,
        },
    ));

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
            AppliedForce::default(),
        ))
        .observe(on_drag_start)
        .observe(on_drag)
        .observe(on_drag_end)
        .id();
    commands.spawn((
        RevoluteJoint::new(cart_entity, rod_entity)
            .with_point_compliance(0.0)
            .with_align_compliance(0.0)
            .with_limit_compliance(0.0)
            .with_hinge_axis(Vec3::Z) // Align the axis of rotation along the Z-axis
            .with_local_anchor1(Vec3::new(
                0.0,
                CART_HEIGHT / 2.0,
                CART_DEPTH / 2.0 + ROD_DEPTH / 2.0 + AXIS_LENGTH,
            )) // aim at the center of the rod at the bottom
            .with_local_anchor2(Vec3::new(0.0, -ROD_HEIGHT / 2.0, 0.0)), // Anchor on the rod (bottom)
        JointDamping {
            linear: 10.0,
            angular: 10.0,
        },
    ));

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

fn get_rigid_body_entity(
    mut drag_target: Entity,
    parents: &Query<(&ChildOf, Option<&RigidBody>)>,
) -> Entity {
    // get a rigid body entity (the drag event may target a descendant of them)
    while let Ok((parent, maybe_rigid_body)) = parents.get(drag_target) {
        if maybe_rigid_body.is_some() {
            break;
        }
        drag_target = parent.parent();
    }

    // this will return the highest ancestor if no rigid body is found, which is fine
    drag_target
}

/// Called immediately when drag starts (before any mouse movement)
fn on_drag_start(
    drag: On<Pointer<DragStart>>,
    parents: Query<(&ChildOf, Option<&RigidBody>)>,
    cart: Query<(), With<Cart>>,
    mut drag_state: ResMut<DragState>,
) {
    if drag.button != PointerButton::Primary {
        return;
    }

    let original_target = drag.entity;
    let target_entity = get_rigid_body_entity(original_target, &parents);
    let is_cart = cart.get(target_entity).is_ok();

    if is_cart {
        drag_state.override_motor = true;
    }

    // Initialize drag with zero force - on_drag will update with actual force
    drag_state.active_drag = Some((target_entity, Vector::ZERO));
}

fn on_drag(
    drag: On<Pointer<Drag>>,
    camera_query: Option<Single<&Transform, With<Camera>>>,
    parents: Query<(&ChildOf, Option<&RigidBody>)>,
    cart: Query<(), With<Cart>>,
    mut applied_forces: Query<&mut AppliedForce>,
    mut drag_state: ResMut<DragState>,
    drag_control: Res<DragControl>,
) {
    if drag.button != PointerButton::Primary {
        return;
    }
    let camera_transform = if let Some(camera_query) = camera_query {
        camera_query.into_inner()
    } else {
        return;
    };

    // Use drag.entity (the Pointer<E>.entity field) to get the originally hit entity
    let original_target = drag.entity;
    let target_entity = get_rigid_body_entity(original_target, &parents);
    let is_cart = cart.get(target_entity).is_ok();

    if is_cart {
        drag_state.override_motor = true;
    }

    // calculate world X-direction drag from screenspace drag
    // drag.delta.y should basically never contribute (as long as camera isn't rolled), but scaling by camera_transform.right() will feel more natural when dragging from a steep visual angle
    let drag_delta_world =
        drag.distance.x * camera_transform.right() + drag.distance.y * camera_transform.down();

    // apply a force to that object based on the world length and direction of the mouse drag
    let applied_force_vec: Vector = (drag_delta_world / drag_control.pixels_per_newton)
        .clamp_length_max(drag_control.max_force);

    // Store the drag state - a system will apply force continuously
    drag_state.active_drag = Some((target_entity, applied_force_vec));

    if let Ok(mut recorded_force) = applied_forces.get_mut(target_entity) {
        recorded_force.0 = applied_force_vec;
    }
}

fn on_drag_end(
    drag: On<Pointer<DragEnd>>,
    parents: Query<(&ChildOf, Option<&RigidBody>)>,
    cart: Query<(), With<Cart>>,
    mut applied_forces: Query<&mut AppliedForce>,
    mut drag_state: ResMut<DragState>,
) {
    if drag.button != PointerButton::Primary {
        return;
    }

    let target_entity = get_rigid_body_entity(drag.entity, &parents);
    let is_cart = cart.get(target_entity).is_ok();

    if is_cart {
        drag_state.override_motor = false;
    }

    // Clear the active drag
    drag_state.active_drag = None;

    if let Ok(mut recorded_force) = applied_forces.get_mut(target_entity) {
        recorded_force.0 = Vector::ZERO;
    }
}

/// System that applies drag force continuously while dragging
fn apply_drag_force(drag_state: Res<DragState>, mut forces: Query<Forces>) {
    if let Some((entity, force_vec)) = drag_state.active_drag
        && let Ok(mut force) = forces.get_mut(entity)
    {
        force.apply_force(force_vec);
    }
}

pub fn external_force_display(
    external_force: Query<(Entity, &Position, &AppliedForce)>,
    cart: Query<(), With<Cart>>,
    rod: Query<(), With<Rod>>,
    mut gizmos: Gizmos,
    keys: Res<ButtonInput<KeyCode>>,
    mut should_display: Local<bool>,
) {
    if keys.just_pressed(KeyCode::KeyF) {
        *should_display = !*should_display;
    }
    if *should_display {
        external_force
            .iter()
            .filter(|(entity, _, _)| {
                // only display external forces for the cart or the rod
                cart.get(*entity).is_ok() || rod.get(*entity).is_ok()
            })
            .for_each(|(_, position, external_force)| {
                gizmos.arrow(
                    **position,
                    **position + (external_force.0).clamp_length_max(10.),
                    RED,
                );
            });
    }
}

#[allow(clippy::type_complexity)]
fn reset_sim(
    keys: Res<ButtonInput<KeyCode>>,
    mut drag_state: ResMut<DragState>,
    mut queries: ParamSet<(
        Query<(
            Entity,
            Option<&Rod>,
            Option<&Cart>,
            Option<&mut Transform>,
            Option<&mut Position>,
            Option<&mut Rotation>,
            Option<&mut PreSolveDeltaPosition>,
            Option<&mut PreSolveDeltaRotation>,
            Option<&mut AppliedForce>,
            Option<&mut ConstantForce>,
            Option<&mut LinearVelocity>,
            Option<&mut AngularVelocity>,
        )>,
        Query<Forces>,
    )>,
) {
    if keys.just_pressed(KeyCode::KeyR) {
        drag_state.override_motor = false;
        let mut entities_with_forces = Vec::new();

        {
            let mut query = queries.p0();
            for (
                entity,
                rod_component,
                cart_component,
                transform,
                position,
                rotation,
                presolve_position,
                presolve_rotation,
                ext_force,
                constant_force,
                linear_velocity,
                angular_velocity,
            ) in query.iter_mut()
            {
                let is_rod: bool = rod_component.is_some();
                let is_cart: bool = cart_component.is_some();

                // Calculate the target translations once so we can apply them to both Transform and Position.
                let rod_translation: Vector = Vector::new(
                    0.0,
                    ROD_HEIGHT / 2.0 + CART_HEIGHT,
                    CART_DEPTH / 2.0 + ROD_DEPTH / 2.0 + AXIS_LENGTH,
                );
                let mut cart_translation: Vector = transform
                    .as_ref()
                    .map(|t| Vector::new(t.translation.x, t.translation.y, t.translation.z))
                    .or_else(|| position.as_ref().map(|p| p.0))
                    .unwrap_or(Vector::ZERO);
                cart_translation.x = 0.0;

                if let Some(mut transform) = transform {
                    if is_rod {
                        transform.translation =
                            Vec3::new(rod_translation.x, rod_translation.y, rod_translation.z);
                        transform.rotation = Quat::IDENTITY;
                    }

                    if is_cart {
                        transform.translation =
                            Vec3::new(cart_translation.x, cart_translation.y, cart_translation.z);
                    }
                }
                if let Some(mut position) = position {
                    if is_rod {
                        *position = Position::new(rod_translation);
                    }
                    if is_cart {
                        *position = Position::new(cart_translation);
                    }
                }
                if let Some(mut rotation) = rotation
                    && is_rod
                {
                    *rotation = Rotation::IDENTITY;
                }
                if let Some(mut presolve_position) = presolve_position {
                    **presolve_position = Vector::ZERO;
                }
                if let Some(mut presolve_rotation) = presolve_rotation {
                    **presolve_rotation = Rotation::IDENTITY;
                }
                if let Some(mut ext_force) = ext_force {
                    ext_force.0 = Vector::ZERO;
                }
                if let Some(mut constant_force) = constant_force {
                    constant_force.0 = Vector::ZERO;
                }

                // Defer force resets until after this query borrow is dropped.
                entities_with_forces.push(entity);

                if let Some(mut velocity) = linear_velocity {
                    *velocity = LinearVelocity::ZERO;
                }

                if let Some(mut angular_velocity) = angular_velocity {
                    *angular_velocity = AngularVelocity::ZERO;
                }
            }
        }

        let mut forces = queries.p1();
        for entity in entities_with_forces {
            if let Ok(mut force) = forces.get_mut(entity) {
                force.reset_accumulated_linear_acceleration();
                force.reset_accumulated_angular_acceleration();
            }
        }
    }
}

/// Winged some type of orbital camera to explore around the robot.
fn camera_control_system(
    camera_control: Res<CameraControl>,
    keys: Res<ButtonInput<KeyCode>>,
    mut scroll_evr: MessageReader<MouseWheel>,
    mut mouse_motion: MessageReader<MouseMotion>,
    mut query: Query<&mut Transform, With<Camera>>,
    // use real time to scale camera movement in case physics time is paused
    time: Res<Time<Real>>,
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
        let zoom_amount = ev.y * camera_control.zoom_sensitivity * time.delta_secs();
        camera_transform.translation += forward * zoom_amount;
    }

    // Rotate camera around the focal point with right mouse button + drag
    if mouse_button_input.pressed(MouseButton::Middle) {
        for ev in mouse_motion.read() {
            let yaw = Quat::from_rotation_y(-ev.delta.x * camera_control.rotate_sensitivity);
            let pitch = Quat::from_rotation_x(-ev.delta.y * camera_control.rotate_sensitivity);

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
            camera_transform.translation += right * -ev.delta.x * camera_control.move_sensitivity;
            camera_transform.translation += up * ev.delta.y * camera_control.move_sensitivity;
        }
    }

    let forward = if keys.pressed(KeyCode::KeyW) {
        camera_transform.forward() * camera_control.move_sensitivity
    } else if keys.pressed(KeyCode::KeyS) {
        camera_transform.back() * camera_control.move_sensitivity
    } else {
        Vec3::ZERO
    };

    let strafe = if keys.pressed(KeyCode::KeyA) {
        camera_transform.left() * camera_control.move_sensitivity
    } else if keys.pressed(KeyCode::KeyD) {
        camera_transform.right() * camera_control.move_sensitivity
    } else {
        Vec3::ZERO
    };

    let vertical = if keys.pressed(KeyCode::KeyQ) {
        Vec3::Y * camera_control.move_sensitivity
    } else if keys.pressed(KeyCode::KeyE) {
        Vec3::NEG_Y * camera_control.move_sensitivity
    } else {
        Vec3::ZERO
    };

    camera_transform.translation += forward + strafe + vertical;

    // make camera look at cart again after any movement
    camera_transform.look_at(Vec3::ZERO, Vec3::Y);
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

// Clamp cart rotation to its initial orientation so it only slides on the rail.
#[allow(clippy::type_complexity)]
fn lock_cart_rotation(
    mut carts: Query<
        (
            &CartRotationLock,
            Option<&mut Rotation>,
            Option<&mut Transform>,
            Option<&mut AngularVelocity>,
        ),
        With<Cart>,
    >,
) {
    for (lock, rotation, transform, angular_velocity) in carts.iter_mut() {
        if let Some(mut rotation) = rotation {
            *rotation = Rotation(lock.0);
        }
        if let Some(mut transform) = transform {
            transform.rotation = lock.0;
        }
        if let Some(mut angular_velocity) = angular_velocity {
            *angular_velocity = AngularVelocity::ZERO;
        }
    }
}
