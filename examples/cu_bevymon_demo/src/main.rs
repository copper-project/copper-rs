use bevy::prelude::*;
use cu_bevymon::{
    CuBevyMonFocus, CuBevyMonFocusBorder, CuBevyMonModel, CuBevyMonPanel, CuBevyMonPlugin,
    CuBevyMonSurface, CuBevyMonSurfaceNode, CuBevyMonTexture, CuBevyMonViewportSurface,
    MonitorModel, MonitorUiOptions,
};
use cu29::clock::CuDuration;
use cu29::monitoring::{
    ComponentId, ComponentType, CopperListInfo, CopperListIoStats, MonitorComponentMetadata,
    MonitorConnection, MonitorNode, MonitorTopology,
};
use std::time::Duration;

const SIM_PANEL_PERCENT: f32 = 62.0;
const MONITOR_PANEL_PERCENT: f32 = 38.0;
const CUBE_MOVE_SPEED: f32 = 3.6;
const CAMERA_ORBIT_SPEED: f32 = 1.7;
const CAMERA_ZOOM_SPEED: f32 = 0.65;

const COMPONENTS: &[MonitorComponentMetadata] = &[
    MonitorComponentMetadata::new("clock", ComponentType::Source, Some("demo::ClockSource")),
    MonitorComponentMetadata::new("planner", ComponentType::Task, Some("demo::PlannerTask")),
    MonitorComponentMetadata::new(
        "controller",
        ComponentType::Task,
        Some("demo::ControllerTask"),
    ),
    MonitorComponentMetadata::new("viz", ComponentType::Bridge, Some("demo::VizBridge")),
    MonitorComponentMetadata::new("actuator", ComponentType::Sink, Some("demo::MotorSink")),
];

#[derive(Component)]
struct SimCamera;

#[derive(Component)]
struct OrbitingCube;

#[derive(Component)]
struct AccentRing;

#[derive(Component)]
struct DemoHudText;

#[derive(Resource, Default)]
struct LayoutSpawned(bool);

#[derive(Resource, Default)]
struct DemoCopperListId(u64);

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
    let model = MonitorModel::from_parts(
        COMPONENTS,
        CopperListInfo::new(1_536, COMPONENTS.len()),
        demo_topology(),
    );

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Copper BevyMon Demo".into(),
                resolution: (1600, 900).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(
            CuBevyMonPlugin::new(model)
                .with_initial_focus(CuBevyMonSurface::Sim)
                .with_options(MonitorUiOptions {
                    system_info:
                        "Windowed Ratatui backend via bevy_ratatui\nClick a panel to move focus"
                            .to_string(),
                    show_quit_hint: false,
                }),
        )
        .insert_resource(ClearColor(Color::srgb(0.04, 0.05, 0.07)))
        .init_resource::<LayoutSpawned>()
        .init_resource::<DemoCopperListId>()
        .init_resource::<DemoCameraRig>()
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                spawn_demo_layout,
                drive_sim_controls,
                handle_sim_zoom,
                sync_demo_camera,
                animate_scene,
                update_demo_hud,
                drive_demo_monitor,
            ),
        )
        .run();
}

fn demo_topology() -> MonitorTopology {
    MonitorTopology {
        nodes: vec![
            MonitorNode {
                id: "clock".into(),
                type_name: Some("demo::ClockSource".into()),
                kind: ComponentType::Source,
                inputs: vec![],
                outputs: vec!["tick".into()],
            },
            MonitorNode {
                id: "planner".into(),
                type_name: Some("demo::PlannerTask".into()),
                kind: ComponentType::Task,
                inputs: vec!["in".into()],
                outputs: vec!["traj".into()],
            },
            MonitorNode {
                id: "controller".into(),
                type_name: Some("demo::ControllerTask".into()),
                kind: ComponentType::Task,
                inputs: vec!["in".into()],
                outputs: vec!["cmd".into()],
            },
            MonitorNode {
                id: "viz".into(),
                type_name: Some("demo::VizBridge".into()),
                kind: ComponentType::Bridge,
                inputs: vec!["state".into()],
                outputs: vec!["mesh".into()],
            },
            MonitorNode {
                id: "actuator".into(),
                type_name: Some("demo::MotorSink".into()),
                kind: ComponentType::Sink,
                inputs: vec!["in".into()],
                outputs: vec![],
            },
        ],
        connections: vec![
            MonitorConnection {
                src: "clock".into(),
                src_port: Some("tick".into()),
                dst: "planner".into(),
                dst_port: Some("in".into()),
                msg: "Tick".into(),
            },
            MonitorConnection {
                src: "planner".into(),
                src_port: Some("traj".into()),
                dst: "controller".into(),
                dst_port: Some("in".into()),
                msg: "Trajectory".into(),
            },
            MonitorConnection {
                src: "controller".into(),
                src_port: Some("cmd".into()),
                dst: "actuator".into(),
                dst_port: Some("in".into()),
                msg: "MotorCommand".into(),
            },
            MonitorConnection {
                src: "controller".into(),
                src_port: Some("cmd".into()),
                dst: "viz".into(),
                dst_port: Some("state".into()),
                msg: "VizState".into(),
            },
        ],
    }
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Camera {
            order: 0,
            ..default()
        },
        Transform::from_xyz(5.0, 4.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
        CuBevyMonViewportSurface(CuBevyMonSurface::Sim),
        SimCamera,
    ));
    commands.spawn((
        Camera2d,
        Camera {
            order: 1,
            ..default()
        },
        IsDefaultUiCamera,
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

fn spawn_demo_layout(
    mut commands: Commands,
    texture: Option<Res<CuBevyMonTexture>>,
    mut spawned: ResMut<LayoutSpawned>,
) {
    if spawned.0 {
        return;
    }
    let Some(texture) = texture else {
        return;
    };

    commands
        .spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                padding: UiRect::all(Val::Px(12.0)),
                column_gap: Val::Px(12.0),
                ..default()
            },
            BackgroundColor(Color::NONE),
        ))
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
                        width: Val::Percent(SIM_PANEL_PERCENT),
                        height: Val::Percent(100.0),
                        position_type: PositionType::Relative,
                        border: UiRect::all(Val::Px(1.0)),
                        ..default()
                    },
                    BackgroundColor(Color::NONE),
                    BorderColor::all(Color::srgb(0.24, 0.29, 0.35)),
                    CuBevyMonSurfaceNode(CuBevyMonSurface::Sim),
                    CuBevyMonFocusBorder::new(CuBevyMonSurface::Sim),
                ))
                .with_children(|panel| {
                    panel
                        .spawn((
                            Node {
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

            parent
                .spawn((
                    Node {
                        width: Val::Percent(MONITOR_PANEL_PERCENT),
                        height: Val::Percent(100.0),
                        border: UiRect::all(Val::Px(1.0)),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.04, 0.05, 0.08, 0.98)),
                    BorderColor::all(Color::srgb(0.24, 0.29, 0.35)),
                    CuBevyMonSurfaceNode(CuBevyMonSurface::Monitor),
                    CuBevyMonFocusBorder::new(CuBevyMonSurface::Monitor),
                ))
                .with_children(|panel| {
                    panel.spawn((
                        ImageNode::new(texture.0.clone()),
                        Node {
                            width: Val::Percent(100.0),
                            height: Val::Percent(100.0),
                            ..default()
                        },
                        BackgroundColor(Color::BLACK),
                        CuBevyMonPanel,
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
            "WASD move cube, Q/E raise or lower it.\nArrow keys orbit camera, wheel zooms, and H also yaws left.\nClick the monitor to hand input to cu_tuimon and prove that H switches back to TUI scroll.",
        ),
        CuBevyMonSurface::Monitor => (
            "MONITOR",
            "The right panel owns keyboard and mouse now.\nUse the normal cu_tuimon clicks, tabs, scrolling, and keys.\nClick back into the sim to recover controls.",
        ),
    };

    let text = format!(
        "Focus: {focus_label}\n\nThis is the reusable side-by-side layout path.\nBoth panels stay visible; only input focus moves.\n\n{footer}"
    );

    for mut ui_text in &mut texts {
        *ui_text = Text::new(text.clone());
    }
}

fn drive_demo_monitor(
    time: Res<Time>,
    model: Res<CuBevyMonModel>,
    mut copperlist_id: ResMut<DemoCopperListId>,
) {
    copperlist_id.0 = copperlist_id.0.wrapping_add(1);
    let t = time.elapsed_secs();
    let base_micros = |offset: f32, scale: f32| -> u64 {
        (250.0 + ((t + offset).sin() * scale + scale) * 220.0).max(50.0) as u64
    };

    let planner = CuDuration::from(Duration::from_micros(base_micros(0.2, 1.8)));
    let controller = CuDuration::from(Duration::from_micros(base_micros(1.1, 1.3)));
    let viz = CuDuration::from(Duration::from_micros(base_micros(2.0, 0.8)));
    let actuator = CuDuration::from(Duration::from_micros(base_micros(2.8, 0.6)));

    model.0.record_component_latency(
        ComponentId::new(0),
        CuDuration::from(Duration::from_micros(90)),
    );
    model
        .0
        .record_component_latency(ComponentId::new(1), planner);
    model
        .0
        .record_component_latency(ComponentId::new(2), controller);
    model.0.record_component_latency(ComponentId::new(3), viz);
    model
        .0
        .record_component_latency(ComponentId::new(4), actuator);
    model
        .0
        .record_end_to_end_latency(CuDuration::from(Duration::from_micros(
            90 + planner.as_micros() as u64
                + controller.as_micros() as u64
                + viz.as_micros() as u64
                + actuator.as_micros() as u64,
        )));

    model
        .0
        .set_component_status(ComponentId::new(0), format!("tick {}", copperlist_id.0));
    model.0.set_component_status(
        ComponentId::new(1),
        format!("target x={:+.2}", (t * 0.6).sin() * 1.8),
    );
    model.0.set_component_status(
        ComponentId::new(2),
        format!("cmd τ={:+.2}", (t * 1.1).cos() * 0.75),
    );
    model.0.set_component_status(
        ComponentId::new(3),
        format!("mesh phase {:.2}", t.rem_euclid(std::f32::consts::TAU)),
    );
    model
        .0
        .set_component_status(ComponentId::new(4), "armed".to_string());

    if (t * 0.25).sin() > 0.96 {
        model
            .0
            .set_component_error(ComponentId::new(2), "torque clamp".to_string());
    }

    model.0.update_copperlist_rate(copperlist_id.0);
    model.0.observe_copperlist_io(CopperListIoStats {
        raw_culist_bytes: 1_536,
        handle_bytes: 256 + (t.sin().abs() * 96.0) as u64,
        encoded_culist_bytes: 744 + (t.cos().abs() * 128.0) as u64,
        keyframe_bytes: if copperlist_id.0 % 90 == 0 { 384 } else { 0 },
        structured_log_bytes_total: copperlist_id.0.saturating_mul(320),
        culistid: copperlist_id.0,
    });

    model.0.upsert_pool_stat(
        "telemetry",
        88 - ((t.sin().abs() * 18.0) as usize),
        128,
        8 * 1024,
    );
    model.0.upsert_pool_stat(
        "viz_cache",
        18 - ((t.cos().abs() * 7.0) as usize),
        32,
        64 * 1024,
    );
}
