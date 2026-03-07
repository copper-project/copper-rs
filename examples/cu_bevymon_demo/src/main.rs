use bevy::camera::Viewport;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use cu_bevymon::{
    CuBevyMonModel, CuBevyMonPanel, CuBevyMonPlugin, CuBevyMonTexture, MonitorModel,
    MonitorUiOptions,
};
use cu29::clock::CuDuration;
use cu29::monitoring::{
    ComponentId, ComponentType, CopperListInfo, CopperListIoStats, MonitorComponentMetadata,
    MonitorConnection, MonitorNode, MonitorTopology,
};
use std::time::Duration;

const MONITOR_PANEL_RATIO: f32 = 0.38;

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

#[derive(Resource, Default)]
struct LayoutSpawned(bool);

#[derive(Resource, Default)]
struct DemoCopperListId(u64);

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
            CuBevyMonPlugin::new(model).with_options(MonitorUiOptions {
                system_info:
                    "Windowed Ratatui backend via bevy_ratatui\nSplit-view Bevy monitor demo"
                        .to_string(),
                show_quit_hint: false,
            }),
        )
        .insert_resource(ClearColor(Color::srgb(0.04, 0.05, 0.07)))
        .init_resource::<LayoutSpawned>()
        .init_resource::<DemoCopperListId>()
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                spawn_monitor_panel,
                update_sim_viewport,
                animate_scene,
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
        Transform::from_xyz(5.0, 4.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
        SimCamera,
    ));
    commands.spawn((Camera2d, IsDefaultUiCamera));

    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 350.0,
        affects_lightmapped_meshes: true,
    });

    commands.spawn((
        DirectionalLight {
            illuminance: 15_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(6.0, 10.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(16.0, 16.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.10, 0.12, 0.16),
            perceptual_roughness: 0.96,
            ..default()
        })),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.2, 1.2, 1.2))),
        MeshMaterial3d(materials.add(Color::srgb(0.93, 0.46, 0.20))),
        Transform::from_xyz(0.0, 1.0, 0.0),
        OrbitingCube,
    ));

    commands.spawn((
        Mesh3d(meshes.add(Torus::new(1.8, 0.08))),
        MeshMaterial3d(materials.add(Color::srgb(0.16, 0.55, 0.76))),
        Transform::from_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
    ));
}

fn spawn_monitor_panel(
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
                justify_content: JustifyContent::FlexEnd,
                ..default()
            },
            BackgroundColor(Color::NONE),
        ))
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
                        width: Val::Percent(MONITOR_PANEL_RATIO * 100.0),
                        height: Val::Percent(100.0),
                        padding: UiRect::all(Val::Px(14.0)),
                        border: UiRect::left(Val::Px(2.0)),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.05, 0.06, 0.08, 0.97)),
                    BorderColor::all(Color::srgb(0.24, 0.29, 0.35)),
                ))
                .with_children(|panel| {
                    panel.spawn((
                        ImageNode::new(texture.0.clone()),
                        Node {
                            width: Val::Percent(100.0),
                            height: Val::Percent(100.0),
                            ..default()
                        },
                        CuBevyMonPanel,
                    ));
                });
        });

    spawned.0 = true;
}

fn update_sim_viewport(
    window: Single<&Window, With<PrimaryWindow>>,
    mut cameras: Query<&mut Camera, With<SimCamera>>,
) {
    let viewport_width = ((1.0 - MONITOR_PANEL_RATIO) * window.physical_width() as f32)
        .round()
        .max(1.0) as u32;
    let viewport_height = window.physical_height();
    let viewport = Some(Viewport {
        physical_position: UVec2::ZERO,
        physical_size: UVec2::new(viewport_width, viewport_height),
        ..default()
    });

    for mut camera in &mut cameras {
        camera.viewport = viewport.clone();
    }
}

fn animate_scene(time: Res<Time>, mut cubes: Query<&mut Transform, With<OrbitingCube>>) {
    let t = time.elapsed_secs();
    for mut transform in &mut cubes {
        transform.translation.x = t.cos() * 2.2;
        transform.translation.z = t.sin() * 1.4;
        transform.translation.y = 1.0 + (t * 1.6).sin() * 0.25;
        transform.rotate_y(time.delta_secs() * 1.7);
        transform.rotate_x(time.delta_secs() * 0.9);
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
