#![cfg(feature = "sim")]

extern crate alloc;

mod messages;
mod sim_support;
mod tasks;

use avian3d::prelude::*;
use bevy::app::AppExit;
use bevy::core_pipeline::Skybox;
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::prelude::{
    App, AssetServer, ButtonInput, Camera, Camera3d, Color, Commands, Component, DefaultPlugins,
    Dir3, DirectionalLight, FixedUpdate, GlobalAmbientLight, GlobalTransform, GltfAssetLabel,
    KeyCode, MessageReader, MessageWriter, MinimalPlugins, Name, PerspectiveProjection,
    PluginGroup, PostUpdate, Projection, Quat, Query, Res, ResMut, Resource, SceneRoot, Startup,
    Time, Transform, Update, Vec3, Window, WindowPlugin, With, Without, default,
};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;

use cu_crsf::messages::RcChannelsPayload;
use cu_sensor_payloads::{BarometerPayload, ImuPayload, MagnetometerPayload};

use std::fs;
use std::path::{Path, PathBuf};

#[copper_runtime(config = "copperconfig.ron", sim_mode = true, ignore_resources = true)]
struct FlightControllerSim {}

#[derive(Clone)]
struct SimVehicleState {
    position: Vec3,
    rotation: Quat,
    body_accel_fc: [f32; 3],
    body_gyro_fc: [f32; 3],
}

impl Default for SimVehicleState {
    fn default() -> Self {
        Self {
            position: Vec3::new(0.0, 1.0, 0.0),
            rotation: Quat::IDENTITY,
            body_accel_fc: [0.0, 0.0, 9.81],
            body_gyro_fc: [0.0; 3],
        }
    }
}

#[derive(Resource)]
struct CopperState {
    _ctx: CopperContext,
    app: FlightControllerSim,
}

#[derive(Resource, Default, Clone)]
struct SimMotorCommands {
    dshot: [u16; 4],
}

#[derive(Resource, Default, Clone)]
struct SimState {
    vehicle: SimVehicleState,
}

#[derive(Resource, Clone)]
struct SimRcInput {
    roll: f32,
    pitch: f32,
    yaw: f32,
    throttle: f32,
    armed: bool,
    mode: messages::FlightMode,
}

impl Default for SimRcInput {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.0,
            armed: false,
            mode: messages::FlightMode::Angle,
        }
    }
}

#[derive(Resource, Default)]
struct SimKinematics {
    prev_linear_velocity: Option<Vec3>,
}

#[derive(Debug)]
struct QuadcopterForceTorque {
    force: Vec3,
    torque: Vec3,
}

enum RotationDirection {
    CounterClockWise,
    ClockWise,
}

struct PropellerInfo {
    position: Vec3,
    direction: Dir3,
    thrust_constant: f32,
    drag_constant: f32,
    rotation_direction: RotationDirection,
}

#[derive(Component)]
struct Multicopter {
    propellers: Vec<PropellerInfo>,
}

impl Multicopter {
    fn new(propellers: Vec<PropellerInfo>) -> Self {
        assert!(
            !propellers.is_empty(),
            "multicopter must have at least one propeller"
        );
        Self { propellers }
    }

    fn force_torque(
        &self,
        state: &GlobalTransform,
        control_inputs: &[f32],
    ) -> Result<QuadcopterForceTorque, String> {
        if control_inputs.len() != self.propellers.len() {
            return Err("incorrect control input length".to_string());
        }

        let mut thrust_local = Vec3::ZERO;
        let mut torque_local = Vec3::ZERO;

        for (propeller, omega) in self.propellers.iter().zip(control_inputs.iter().copied()) {
            let force = propeller.direction * (propeller.thrust_constant * omega.powi(2));
            thrust_local += force;

            let drag_sign = match propeller.rotation_direction {
                RotationDirection::CounterClockWise => 1.0,
                RotationDirection::ClockWise => -1.0,
            };

            let torque = propeller.position.cross(force)
                + propeller.drag_constant * omega.powi(2) * drag_sign * propeller.direction;
            torque_local += torque;
        }

        let rotation = state.rotation();
        let force_world = rotation * thrust_local;
        let torque_world = rotation * torque_local;

        Ok(QuadcopterForceTorque {
            force: force_world,
            torque: torque_world,
        })
    }
}

const THRUST_CONSTANT: f32 = 1.0e-6;
const DRAG_CONSTANT: f32 = 1.0e-7;
const MAX_OMEGA_RAD_S: f32 = 2200.0;
const WORLD_MAG_FIELD_UT: [f32; 3] = [20.0, 0.0, -45.0];

fn propeller_from_position(x: f32, y: f32, z: f32, direction: RotationDirection) -> PropellerInfo {
    PropellerInfo {
        position: Vec3 { x, y, z },
        direction: Dir3::new(Vec3::Y).expect("unit axis"),
        thrust_constant: THRUST_CONSTANT,
        drag_constant: DRAG_CONSTANT,
        rotation_direction: direction,
    }
}

fn quad_x_propellers() -> Vec<PropellerInfo> {
    // Motor order must match cu_flight_controller mixer indices:
    // 0 rear-right (CCW), 1 front-right (CW), 2 rear-left (CW), 3 front-left (CCW).
    vec![
        propeller_from_position(0.08, 0.0, -0.08, RotationDirection::CounterClockWise),
        propeller_from_position(0.08, 0.0, 0.08, RotationDirection::ClockWise),
        propeller_from_position(-0.08, 0.0, -0.08, RotationDirection::ClockWise),
        propeller_from_position(-0.08, 0.0, 0.08, RotationDirection::CounterClockWise),
    ]
}

fn default_callback(_step: default::SimStep) -> SimOverride {
    SimOverride::ExecuteByRuntime
}

fn set_msg_timing<T: CuMsgPayload>(clock: &RobotClock, msg: &mut CuMsg<T>) {
    let now = clock.now();
    msg.tov = Tov::Time(now);
    msg.metadata.process_time.start = now.into();
    msg.metadata.process_time.end = now.into();
}

fn axis_to_rc(value: f32) -> u16 {
    const RC_MIN: f32 = 172.0;
    const RC_MID: f32 = 992.0;
    const RC_MAX: f32 = 1811.0;

    let v = value.clamp(-1.0, 1.0);
    if v >= 0.0 {
        (RC_MID + (RC_MAX - RC_MID) * v).round() as u16
    } else {
        (RC_MID + (RC_MID - RC_MIN) * v).round() as u16
    }
}

fn throttle_to_rc(value: f32) -> u16 {
    const RC_MIN: f32 = 172.0;
    const RC_MAX: f32 = 1811.0;
    (RC_MIN + (RC_MAX - RC_MIN) * value.clamp(0.0, 1.0)).round() as u16
}

fn dshot_to_omega(dshot: u16) -> f32 {
    if dshot < 48 {
        return 0.0;
    }
    let normalized = (dshot as f32 / 2047.0).clamp(0.0, 1.0);
    normalized * MAX_OMEGA_RAD_S
}

fn map_bevy_body_to_fc_axes(v: Vec3) -> [f32; 3] {
    [v.x, v.z, v.y]
}

fn setup_copper(mut commands: Commands) {
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(128 * 1024 * 1024);
    let logger_path = "logs/flight_controller_sim.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("failed to create logs directory");
    }

    let ctx = basic_copper_setup(&PathBuf::from(logger_path), LOG_SLAB_SIZE, true, None)
        .expect("failed to setup logger");

    let mut app = FlightControllerSimBuilder::new()
        .with_context(&ctx)
        .with_sim_callback(&mut default_callback)
        .build()
        .expect("failed to create runtime");

    app.start_all_tasks(&mut default_callback)
        .expect("failed to start tasks");

    commands.insert_resource(CopperState { _ctx: ctx, app });
}

fn setup_world(mut commands: Commands, asset_server: Res<AssetServer>) {
    let skybox_handle = asset_server.load("skybox.ktx2");

    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 1.0 / 5.0,
        affects_lightmapped_meshes: true,
    });

    commands.spawn((
        Name::new("camera"),
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: 90.0_f32.to_radians(),
            ..default()
        }),
        Skybox {
            image: skybox_handle,
            brightness: 1000.0,
            ..default()
        },
        Transform::from_xyz(-2.0, 1.6, -2.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Name::new("sun"),
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_translation(Vec3::new(3.0, 10.0, 1.0)).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Name::new("quadcopter"),
        SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("quadcopter.glb"))),
        RigidBody::Dynamic,
        Transform::from_xyz(0.0, 1.0, 0.0),
        Collider::cuboid(0.14, 0.05, 0.14),
        Multicopter::new(quad_x_propellers()),
        Mass(0.7),
        AngularInertia::new(Vec3::new(0.012, 0.02, 0.012)),
        AngularDamping(0.4),
        LinearDamping(0.2),
        SweptCcd::NON_LINEAR,
    ));

    commands.spawn((
        Name::new("level"),
        SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("level.glb"))),
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMesh),
        RigidBody::Static,
    ));
}

fn update_rc_input(mut rc_input: ResMut<SimRcInput>, keyboard: Res<ButtonInput<KeyCode>>) {
    let roll =
        (keyboard.pressed(KeyCode::KeyA) as i8 - keyboard.pressed(KeyCode::KeyD) as i8) as f32;
    let pitch =
        (keyboard.pressed(KeyCode::KeyS) as i8 - keyboard.pressed(KeyCode::KeyW) as i8) as f32;
    let yaw =
        (keyboard.pressed(KeyCode::KeyQ) as i8 - keyboard.pressed(KeyCode::KeyE) as i8) as f32;

    rc_input.roll = roll * 0.6;
    rc_input.pitch = pitch * 0.6;
    rc_input.yaw = yaw * 0.6;

    if keyboard.just_pressed(KeyCode::Digit1) {
        rc_input.mode = messages::FlightMode::Acro;
    }
    if keyboard.just_pressed(KeyCode::Digit2) {
        rc_input.mode = messages::FlightMode::Angle;
    }
    if keyboard.just_pressed(KeyCode::Digit3) {
        rc_input.mode = messages::FlightMode::PositionHold;
    }

    if keyboard.just_pressed(KeyCode::KeyT) {
        rc_input.armed = !rc_input.armed;
        info!("sim rc: armed={} mode={:?}", rc_input.armed, rc_input.mode);
    }
}

fn adjust_throttle(
    mut rc_input: ResMut<SimRcInput>,
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
) {
    let dt = time.delta_secs();
    let mut throttle = rc_input.throttle;

    if keyboard.pressed(KeyCode::Space) {
        throttle += dt * 0.5;
    }
    if keyboard.pressed(KeyCode::ControlLeft) {
        throttle -= dt * 0.5;
    }

    rc_input.throttle = throttle.clamp(0.0, 1.0);
}

fn reset_vehicle(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<
        (&mut Transform, &mut LinearVelocity, &mut AngularVelocity),
        With<Multicopter>,
    >,
    mut motors: ResMut<SimMotorCommands>,
    mut kin: ResMut<SimKinematics>,
) {
    if !keyboard.just_pressed(KeyCode::KeyR) {
        return;
    }

    if let Ok((mut transform, mut lin_vel, mut ang_vel)) = query.single_mut() {
        *transform = Transform::from_xyz(0.0, 1.0, 0.0);
        *lin_vel = LinearVelocity(Vec3::ZERO);
        *ang_vel = AngularVelocity(Vec3::ZERO);
    }

    motors.dshot = [0; 4];
    kin.prev_linear_velocity = None;
}

fn sync_vehicle_state(
    query: Query<(&GlobalTransform, &LinearVelocity, &AngularVelocity), With<Multicopter>>,
    gravity: Res<Gravity>,
    physics_time: Res<Time<Physics>>,
    mut state: ResMut<SimState>,
    mut kin: ResMut<SimKinematics>,
) {
    let Ok((transform, linear_velocity, angular_velocity)) = query.single() else {
        return;
    };

    let dt = physics_time.delta_secs().max(1.0e-4);
    let lv = linear_velocity.0;
    let prev = kin.prev_linear_velocity.unwrap_or(lv);
    kin.prev_linear_velocity = Some(lv);

    let accel_world = (lv - prev) / dt;
    let specific_force_world = accel_world - gravity.0;

    let rotation = transform.rotation();
    let accel_body = rotation.inverse() * specific_force_world;
    let gyro_body = rotation.inverse() * angular_velocity.0;

    state.vehicle = SimVehicleState {
        position: transform.translation(),
        rotation,
        body_accel_fc: map_bevy_body_to_fc_axes(accel_body),
        body_gyro_fc: map_bevy_body_to_fc_axes(gyro_body),
    };
}

fn run_copper(
    mut copper: ResMut<CopperState>,
    sim_state: Res<SimState>,
    rc_input: Res<SimRcInput>,
    mut motor_commands: ResMut<SimMotorCommands>,
    mut exit_writer: MessageWriter<AppExit>,
) {
    let vehicle = sim_state.vehicle.clone();
    let rc = rc_input.clone();
    let clock = copper._ctx.clock.clone();
    let dshot = &mut motor_commands.dshot;

    let mut sim_callback = move |step: default::SimStep| -> SimOverride {
        match step {
            default::SimStep::Bmi088(CuTaskCallbackState::Process(_, output)) => {
                set_msg_timing(&clock, output);
                output.set_payload(ImuPayload::from_raw(
                    vehicle.body_accel_fc,
                    vehicle.body_gyro_fc,
                    29.0,
                ));
                SimOverride::ExecutedBySim
            }
            default::SimStep::Dps310(CuTaskCallbackState::Process(_, output)) => {
                let altitude_m = vehicle.position.y.max(-100.0);
                let pressure_pa = 101_325.0 * (1.0 - altitude_m / 44_330.0).powf(5.255);
                set_msg_timing(&clock, output);
                output.set_payload(BarometerPayload::from_raw(pressure_pa, 25.0));
                SimOverride::ExecutedBySim
            }
            default::SimStep::Ist8310(CuTaskCallbackState::Process(_, output)) => {
                let world_mag = Vec3::from_array(WORLD_MAG_FIELD_UT);
                let body_mag = vehicle.rotation.inverse() * world_mag;
                set_msg_timing(&clock, output);
                output.set_payload(MagnetometerPayload::from_raw(map_bevy_body_to_fc_axes(
                    body_mag,
                )));
                SimOverride::ExecutedBySim
            }
            default::SimStep::RcRxRcRx { msg, .. } => {
                let mut payload = RcChannelsPayload::default();
                let channels = &mut payload.inner_mut().0;
                channels[0] = axis_to_rc(rc.roll);
                channels[1] = axis_to_rc(rc.pitch);
                channels[2] = throttle_to_rc(rc.throttle);
                channels[3] = axis_to_rc(rc.yaw);
                channels[4] = if rc.armed { 1750 } else { 172 };
                channels[5] = match rc.mode {
                    messages::FlightMode::Acro => 300,
                    messages::FlightMode::Angle => 992,
                    messages::FlightMode::PositionHold => 1700,
                };

                set_msg_timing(&clock, msg);
                msg.set_payload(payload);
                SimOverride::ExecutedBySim
            }
            default::SimStep::BdshotTxEsc0Tx { msg, .. } => {
                dshot[0] = msg.payload().map_or(0, |c| c.throttle);
                SimOverride::ExecuteByRuntime
            }
            default::SimStep::BdshotTxEsc1Tx { msg, .. } => {
                dshot[1] = msg.payload().map_or(0, |c| c.throttle);
                SimOverride::ExecuteByRuntime
            }
            default::SimStep::BdshotTxEsc2Tx { msg, .. } => {
                dshot[2] = msg.payload().map_or(0, |c| c.throttle);
                SimOverride::ExecuteByRuntime
            }
            default::SimStep::BdshotTxEsc3Tx { msg, .. } => {
                dshot[3] = msg.payload().map_or(0, |c| c.throttle);
                SimOverride::ExecuteByRuntime
            }
            _ => SimOverride::ExecuteByRuntime,
        }
    };

    if let Err(err) = copper.app.run_one_iteration(&mut sim_callback) {
        error!("sim loop stopped: {}", err);
        exit_writer.write(AppExit::Success);
    }
}

fn apply_multicopter_dynamics(
    mut query: Query<(&Multicopter, &GlobalTransform, Forces), With<Multicopter>>,
    motors: Res<SimMotorCommands>,
    physics_time: Res<Time<Physics>>,
) {
    let Ok((multicopter, transform, mut forces)) = query.single_mut() else {
        return;
    };

    let dt = physics_time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    let omega = motors.dshot.map(dshot_to_omega);
    let Ok(force_torque) = multicopter.force_torque(transform, &omega) else {
        return;
    };

    forces.apply_linear_impulse(dt * force_torque.force);
    forces.apply_angular_impulse(dt * force_torque.torque);
}

fn camera_follow_quadcopter(
    quadcopter_query: Query<&GlobalTransform, With<Multicopter>>,
    mut camera_query: Query<&mut Transform, (With<Camera>, Without<Multicopter>)>,
) {
    let Ok(quad_tf) = quadcopter_query.single() else {
        return;
    };
    let Ok(mut camera_tf) = camera_query.single_mut() else {
        return;
    };

    let camera_position = quad_tf.translation() + -2.0 * quad_tf.forward() + 1.0 * quad_tf.up();
    camera_tf.translation = camera_position;
    *camera_tf = camera_tf.looking_to(quad_tf.forward(), quad_tf.up());
}

fn track_sim_led_state() {
    let _on = sim_support::sim_activity_led_is_on();
}

fn stop_copper_on_exit(mut exit_events: MessageReader<AppExit>, mut copper: ResMut<CopperState>) {
    for _ in exit_events.read() {
        let _ = copper.app.stop_all_tasks(&mut default_callback);
        let _ = copper.app.log_shutdown_completed();
    }
}

fn register_scene_reflect_types(app: &mut App) {
    app.register_type::<bevy::prelude::Transform>();
    app.register_type::<bevy::prelude::GlobalTransform>();
    app.register_type::<bevy::prelude::TransformTreeChanged>();
    app.register_type::<bevy::prelude::Visibility>();
    app.register_type::<bevy::prelude::InheritedVisibility>();
    app.register_type::<bevy::prelude::ViewVisibility>();
    app.register_type::<bevy::prelude::Name>();
    app.register_type::<bevy::prelude::Children>();
    app.register_type::<bevy::prelude::ChildOf>();
    app.register_type::<bevy::prelude::Mesh3d>();
    app.register_type::<bevy::prelude::MeshMaterial3d<bevy::prelude::StandardMaterial>>();
    app.register_type::<bevy::camera::primitives::Aabb>();
    app.register_type::<bevy::prelude::Handle<bevy::prelude::Mesh>>();
    app.register_type::<bevy::prelude::Handle<bevy::prelude::StandardMaterial>>();
    app.register_type::<bevy::prelude::Handle<bevy::prelude::Image>>();
    app.register_type::<bevy::prelude::Handle<bevy::prelude::Gltf>>();
    app.register_type::<bevy::prelude::Handle<bevy::prelude::Scene>>();
    app.register_type::<bevy::gltf::GltfExtras>();
    app.register_type::<bevy::gltf::GltfSceneExtras>();
    app.register_type::<bevy::gltf::GltfMeshExtras>();
    app.register_type::<bevy::gltf::GltfMeshName>();
    app.register_type::<bevy::gltf::GltfMaterialExtras>();
    app.register_type::<bevy::gltf::GltfMaterialName>();
}

pub fn make_world(headless: bool) -> App {
    let mut app = App::new();
    app.insert_resource(SimState::default())
        .insert_resource(SimMotorCommands::default())
        .insert_resource(SimRcInput::default())
        .insert_resource(SimKinematics::default());

    if headless {
        app.add_plugins(MinimalPlugins);
        app.add_systems(Startup, setup_copper);
        app.add_systems(Update, run_copper);
        app.add_systems(PostUpdate, stop_copper_on_exit);
        return app;
    }

    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "Copper Flight Controller Sim".into(),
            ..default()
        }),
        ..default()
    }))
    .add_plugins(PhysicsPlugins::default())
    .insert_resource(Gravity(Vec3::new(0.0, -9.81, 0.0)))
    .insert_resource(Time::<Physics>::default())
    .add_systems(Startup, (setup_world, setup_copper))
    .add_systems(Update, (update_rc_input, adjust_throttle, reset_vehicle))
    .add_systems(
        Update,
        (camera_follow_quadcopter, track_sim_led_state).chain(),
    )
    .add_systems(
        FixedUpdate,
        (sync_vehicle_state, run_copper, apply_multicopter_dynamics).chain(),
    )
    .add_systems(PostUpdate, stop_copper_on_exit);

    register_scene_reflect_types(&mut app);

    app
}

fn main() {
    let mut app = make_world(false);
    app.run();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_world_starts() {
        let mut app = make_world(true);
        app.update();
    }
}
