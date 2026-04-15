pub mod tasks;
mod windowing;

use bevy::app::AppExit;
use bevy::asset::RenderAssetUsages;
use bevy::camera::ClearColorConfig;
use bevy::camera::RenderTarget;
use bevy::prelude::*;
use bevy::render::render_resource::{TextureDimension, TextureFormat, TextureUsages};
use bevy::ui::Node as UiNode;
use cu_bevymon::{
    CuBevyMonFocus, CuBevyMonPlugin, CuBevyMonSplitLayoutConfig, CuBevyMonSplitStyle,
    CuBevyMonSurface, CuBevyMonTexture, MonitorUiOptions, spawn_split_layout,
};
use cu29::prelude::*;
use cu29::prelude::{debug, error, info};
#[cfg(not(target_arch = "wasm32"))]
use std::fs;
#[cfg(not(target_arch = "wasm32"))]
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

const SIM_PANEL_PERCENT: f32 = 62.0;
const MONITOR_PANEL_PERCENT: f32 = 38.0;
const MONITOR_PANEL_INSET_PX: f32 = 4.0;
const CUBE_MOVE_SPEED: f32 = 3.6;
const CAMERA_ORBIT_SPEED: f32 = 1.7;
const CAMERA_ZOOM_SPEED: f32 = 0.65;
const COPPER_TICK_HZ: f32 = 10.0;
#[cfg(not(target_arch = "wasm32"))]
const LOG_SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);

#[copper_runtime(config = "copperconfig.ron")]
struct BevyMonDemoApp {}

#[cfg(target_arch = "wasm32")]
type DemoSectionStorage = NoopSectionStorage;
#[cfg(target_arch = "wasm32")]
type DemoUnifiedLogger = NoopLogger;

#[cfg(not(target_arch = "wasm32"))]
type DemoSectionStorage = memmap::MmapSectionStorage;
#[cfg(not(target_arch = "wasm32"))]
type DemoUnifiedLogger = UnifiedLoggerWrite;

#[derive(Component)]
struct SimCamera;

#[derive(Component)]
struct SplitSceneCamera;

#[derive(Component)]
struct OrbitingCube;

#[derive(Component)]
struct AccentRing;

#[derive(Component)]
struct DemoHudText;

#[derive(Resource, Default)]
struct LayoutSpawned(bool);

#[derive(Resource)]
struct SplitUiCamera(Entity);

#[derive(Resource)]
struct CopperDriver {
    copper_app: BevyMonDemoApp,
    iteration_timer: Timer,
    started: bool,
}

#[derive(Resource)]
struct DemoCameraRig {
    yaw: f32,
    pitch: f32,
    distance: f32,
}

impl Default for DemoCameraRig {
    fn default() -> Self {
        Self {
            yaw: 0.95,
            pitch: -0.32,
            distance: 8.6,
        }
    }
}

fn main() {
    let mut copper = build_copper_driver();
    let monitor_model = copper.copper_app.copper_runtime_mut().monitor.model();

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(primary_window()),
        ..default()
    }));
    #[cfg(not(target_arch = "wasm32"))]
    app.add_systems(Update, windowing::set_copper_window_icon);
    app.add_plugins(
        CuBevyMonPlugin::new(monitor_model)
            .with_initial_focus(CuBevyMonSurface::Sim)
            .with_font_size(24)
            .with_options(MonitorUiOptions {
                show_quit_hint: false,
            }),
    )
    .insert_resource(ClearColor(Color::srgb(0.04, 0.05, 0.07)))
    .insert_resource(copper)
    .init_resource::<LayoutSpawned>()
    .init_resource::<DemoCameraRig>()
    .add_systems(
        Startup,
        (setup_scene, setup_ui_camera, start_copper_runtime),
    )
    .add_systems(
        Update,
        (
            spawn_demo_layout,
            drive_sim_controls,
            handle_sim_zoom,
            sync_demo_camera,
            animate_scene,
            update_demo_hud,
            run_copper_iteration,
        ),
    )
    .add_systems(PostUpdate, stop_copper_on_exit);
    app.run();
}

fn build_copper_driver() -> CopperDriver {
    let unified_logger = build_unified_logger().expect("Failed to create demo logger.");

    #[cfg(target_arch = "wasm32")]
    debug!("Using no-op unified logger for wasm BevyMon demo.");
    #[cfg(not(target_arch = "wasm32"))]
    debug!("Logger created for Copper BevyMon demo.");

    debug!("Creating Copper BevyMon demo application.");

    let copper_app = BevyMonDemoApp::builder()
        .with_logger::<DemoSectionStorage, DemoUnifiedLogger>(unified_logger)
        .build()
        .expect("Failed to create Copper runtime.");

    CopperDriver {
        copper_app,
        iteration_timer: Timer::from_seconds(1.0 / COPPER_TICK_HZ, TimerMode::Repeating),
        started: false,
    }
}

fn start_copper_runtime(mut copper: ResMut<CopperDriver>) {
    <BevyMonDemoApp as CuApplication<DemoSectionStorage, DemoUnifiedLogger>>::start_all_tasks(
        &mut copper.copper_app,
    )
    .expect("Failed to start Copper demo tasks.");
    copper.started = true;
    info!("Copper BevyMon demo runtime started.");
}

#[cfg(not(target_arch = "wasm32"))]
fn primary_window() -> Window {
    Window {
        title: "Copper BevyMon Demo".into(),
        name: Some("io.github.copper-project.bevymon-demo".into()),
        resolution: (1600, 900).into(),
        ..default()
    }
}

#[cfg(target_arch = "wasm32")]
fn primary_window() -> Window {
    Window {
        title: "Copper BevyMon Demo".into(),
        resolution: (1600, 900).into(),
        canvas: Some("#bevy".into()),
        fit_canvas_to_parent: true,
        ..default()
    }
}

#[cfg(target_arch = "wasm32")]
fn build_unified_logger() -> CuResult<Arc<Mutex<DemoUnifiedLogger>>> {
    Ok(Arc::new(Mutex::new(NoopLogger::new())))
}

#[cfg(not(target_arch = "wasm32"))]
fn build_unified_logger() -> CuResult<Arc<Mutex<DemoUnifiedLogger>>> {
    let logger_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("logs/bevymon_demo.copper");
    if let Some(parent) = logger_path.parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }

    let logger = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_base_name(&logger_path)
        .preallocated_size(LOG_SLAB_SIZE.unwrap_or(10 * 1024 * 1024))
        .build()
        .map_err(|err| {
            CuError::new_with_cause("Failed to create unified logger for BevyMon demo", err)
        })?;
    let logger = match logger {
        UnifiedLogger::Write(logger) => logger,
        UnifiedLogger::Read(_) => {
            return Err(CuError::from(
                "UnifiedLoggerBuilder did not create a write-capable logger",
            ));
        }
    };
    Ok(Arc::new(Mutex::new(logger)))
}

fn setup_scene(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut image = Image::new_uninit(
        default(),
        TextureDimension::D2,
        TextureFormat::Bgra8UnormSrgb,
        RenderAssetUsages::all(),
    );
    image.texture_descriptor.usage =
        TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST | TextureUsages::RENDER_ATTACHMENT;
    let split_target = images.add(image);

    commands.spawn((
        Camera3d::default(),
        Camera {
            order: 0,
            ..default()
        },
        RenderTarget::Image(split_target.into()),
        Transform::from_xyz(5.0, 4.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
        SimCamera,
        SplitSceneCamera,
    ));

    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 320.0,
        affects_lightmapped_meshes: true,
    });

    commands.spawn((
        DirectionalLight {
            illuminance: 16_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(8.0, 10.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(18.0, 18.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.12, 0.13, 0.17),
            perceptual_roughness: 0.95,
            ..default()
        })),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.25, 1.25, 1.25))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.93, 0.55, 0.27),
            perceptual_roughness: 0.68,
            ..default()
        })),
        Transform::from_xyz(0.0, 1.0, 0.0),
        OrbitingCube,
    ));

    commands.spawn((
        Mesh3d(meshes.add(Torus::new(1.9, 0.08))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.20, 0.66, 0.86),
            emissive: LinearRgba::new(0.03, 0.08, 0.14, 0.0),
            ..default()
        })),
        Transform::from_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        AccentRing,
    ));
}

fn setup_ui_camera(mut commands: Commands) {
    let camera = commands
        .spawn((
            Camera2d,
            Camera {
                order: 1,
                clear_color: ClearColorConfig::None,
                ..default()
            },
        ))
        .id();
    commands.insert_resource(SplitUiCamera(camera));
}

fn spawn_demo_layout(
    mut commands: Commands,
    texture: Option<Res<CuBevyMonTexture>>,
    ui_camera: Option<Res<SplitUiCamera>>,
    scene_cameras: Query<Entity, With<SplitSceneCamera>>,
    mut spawned: ResMut<LayoutSpawned>,
) {
    if spawned.0 {
        return;
    }
    let Some(texture) = texture else {
        return;
    };
    let Some(ui_camera) = ui_camera else {
        return;
    };
    let Ok(scene_camera) = scene_cameras.single() else {
        return;
    };

    let layout = spawn_split_layout(
        &mut commands,
        texture.0.clone(),
        CuBevyMonSplitLayoutConfig::new(scene_camera)
            .with_ui_camera(ui_camera.0)
            .with_style(CuBevyMonSplitStyle {
                sim_panel_percent: SIM_PANEL_PERCENT,
                monitor_panel_percent: MONITOR_PANEL_PERCENT,
                monitor_panel_inset_px: MONITOR_PANEL_INSET_PX,
                unfocused_border_color: Color::srgb(0.24, 0.29, 0.35),
                ..default()
            }),
    );

    commands.entity(layout.sim_panel).with_children(|panel| {
        panel
            .spawn((
                UiNode {
                    position_type: PositionType::Absolute,
                    left: Val::Px(18.0),
                    top: Val::Px(18.0),
                    padding: UiRect::axes(Val::Px(14.0), Val::Px(10.0)),
                    max_width: Val::Px(330.0),
                    border: UiRect::all(Val::Px(1.0)),
                    ..default()
                },
                BackgroundColor(Color::srgba(0.05, 0.07, 0.10, 0.82)),
                BorderColor::all(Color::srgba(0.35, 0.42, 0.49, 0.92)),
            ))
            .with_children(|card| {
                card.spawn((
                    DemoHudText,
                    Text::new(""),
                    TextFont {
                        font_size: 14.0,
                        ..default()
                    },
                    TextColor(Color::WHITE),
                ));
            });
    });

    spawned.0 = true;
}

fn drive_sim_controls(
    time: Res<Time>,
    focus: Res<CuBevyMonFocus>,
    keys: Res<ButtonInput<KeyCode>>,
    mut rig: ResMut<DemoCameraRig>,
    mut cubes: Query<&mut Transform, With<OrbitingCube>>,
) {
    if focus.0 != CuBevyMonSurface::Sim {
        return;
    }

    let mut translation = Vec3::ZERO;
    if keys.pressed(KeyCode::KeyW) {
        translation.z -= 1.0;
    }
    if keys.pressed(KeyCode::KeyS) {
        translation.z += 1.0;
    }
    if keys.pressed(KeyCode::KeyA) {
        translation.x -= 1.0;
    }
    if keys.pressed(KeyCode::KeyD) {
        translation.x += 1.0;
    }
    if keys.pressed(KeyCode::KeyQ) {
        translation.y += 1.0;
    }
    if keys.pressed(KeyCode::KeyE) {
        translation.y -= 1.0;
    }

    let delta = time.delta_secs();
    for mut cube in &mut cubes {
        if translation.length_squared() > 0.0 {
            cube.translation += translation.normalize() * CUBE_MOVE_SPEED * delta;
            cube.translation.x = cube.translation.x.clamp(-5.0, 5.0);
            cube.translation.y = cube.translation.y.clamp(0.7, 3.0);
            cube.translation.z = cube.translation.z.clamp(-5.0, 5.0);
        }
    }

    if keys.pressed(KeyCode::ArrowLeft) || keys.pressed(KeyCode::KeyH) {
        rig.yaw += CAMERA_ORBIT_SPEED * delta;
    }
    if keys.pressed(KeyCode::ArrowRight) {
        rig.yaw -= CAMERA_ORBIT_SPEED * delta;
    }
    if keys.pressed(KeyCode::ArrowUp) {
        rig.pitch = (rig.pitch + CAMERA_ORBIT_SPEED * 0.7 * delta).clamp(-1.1, 0.3);
    }
    if keys.pressed(KeyCode::ArrowDown) {
        rig.pitch = (rig.pitch - CAMERA_ORBIT_SPEED * 0.7 * delta).clamp(-1.1, 0.3);
    }
}

fn handle_sim_zoom(
    focus: Res<CuBevyMonFocus>,
    mut wheel_events: MessageReader<bevy::input::mouse::MouseWheel>,
    mut rig: ResMut<DemoCameraRig>,
) {
    if focus.0 != CuBevyMonSurface::Sim {
        return;
    }

    for event in wheel_events.read() {
        rig.distance = (rig.distance - event.y * CAMERA_ZOOM_SPEED).clamp(4.0, 16.0);
    }
}

fn sync_demo_camera(
    rig: Res<DemoCameraRig>,
    cubes: Query<&Transform, With<OrbitingCube>>,
    mut cameras: Query<&mut Transform, (With<SimCamera>, Without<OrbitingCube>)>,
) {
    let Some(target_transform) = cubes.iter().next() else {
        return;
    };
    let target = target_transform.translation + Vec3::new(0.0, 0.45, 0.0);
    let flat = rig.distance * rig.pitch.cos();
    let offset = Vec3::new(
        flat * rig.yaw.cos(),
        rig.distance * rig.pitch.sin(),
        flat * rig.yaw.sin(),
    );

    for mut camera in &mut cameras {
        camera.translation = target + offset;
        camera.look_at(target, Vec3::Y);
    }
}

fn animate_scene(
    time: Res<Time>,
    mut cubes: Query<&mut Transform, With<OrbitingCube>>,
    mut rings: Query<&mut Transform, (With<AccentRing>, Without<OrbitingCube>)>,
) {
    for mut cube in &mut cubes {
        cube.rotate_y(time.delta_secs() * 1.5);
        cube.rotate_x(time.delta_secs() * 0.55);
    }

    for mut ring in &mut rings {
        ring.rotate_z(time.delta_secs() * 0.4);
        ring.rotate_y(time.delta_secs() * 0.85);
    }
}

fn update_demo_hud(focus: Res<CuBevyMonFocus>, mut texts: Query<&mut Text, With<DemoHudText>>) {
    let (focus_label, footer) = match focus.0 {
        CuBevyMonSurface::Sim => (
            "SIM",
            "WASD move cube, Q/E raise or lower it.\nArrow keys orbit camera, wheel zooms, and H also yaws left.\nClick the monitor to hand input to the real Copper runtime UI.",
        ),
        CuBevyMonSurface::Monitor => (
            "MONITOR",
            "The right panel owns keyboard and mouse now.\nUse the normal cu_tuimon clicks, tabs, scrolling, and keys.\nThe LOG tab is now fed by a generated Copper app, not a handwritten model.",
        ),
    };

    let text = format!(
        "Focus: {focus_label}\n\nLeft: a small fake Bevy scene.\nRight: cu_bevymon driven by #[copper_runtime] and copperconfig.ron.\nThe Copper graph emits real statuses, copperlists, and structured logs.\n\n{footer}"
    );

    for mut ui_text in &mut texts {
        *ui_text = Text::new(text.clone());
    }
}

fn run_copper_iteration(
    time: Res<Time>,
    mut copper: ResMut<CopperDriver>,
    mut exit_writer: MessageWriter<AppExit>,
) {
    if !copper.started {
        return;
    }

    if !copper.iteration_timer.tick(time.delta()).just_finished() {
        return;
    }

    if let Err(err) =
        <BevyMonDemoApp as CuApplication<DemoSectionStorage, DemoUnifiedLogger>>::run_one_iteration(
            &mut copper.copper_app,
        )
    {
        error!(
            "Copper demo stopped after runtime error: {}",
            err.to_string()
        );
        exit_writer.write(AppExit::Success);
        copper.started = false;
    }
}

fn stop_copper_on_exit(mut exit_events: MessageReader<AppExit>, mut copper: ResMut<CopperDriver>) {
    if exit_events.read().next().is_some() {
        if !copper.started {
            return;
        }

        info!("Stopping Copper BevyMon demo runtime.");
        <BevyMonDemoApp as CuApplication<DemoSectionStorage, DemoUnifiedLogger>>::stop_all_tasks(
            &mut copper.copper_app,
        )
        .expect("Failed to stop Copper demo tasks.");
        let _ = copper.copper_app.log_shutdown_completed();
        copper.started = false;
    }
}
