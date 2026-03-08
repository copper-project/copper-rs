#[cfg(feature = "sim")]
extern crate alloc;

#[cfg(feature = "sim")]
mod messages;
#[cfg(feature = "sim")]
#[path = "sim/rc_joystick.rs"]
mod rc_joystick;
#[cfg(feature = "sim")]
mod sim_support;
#[cfg(feature = "sim")]
mod tasks;

use avian3d::prelude::*;
use bevy::app::AppExit;
#[cfg(all(feature = "bevymon", target_arch = "wasm32"))]
use bevy::asset::AssetMetaCheck;
use bevy::asset::RenderAssetUsages;
use bevy::asset::UnapprovedPathMode;
#[cfg(feature = "bevymon")]
use bevy::camera::ClearColorConfig;
use bevy::camera::RenderTarget;
use bevy::core_pipeline::Skybox;
use bevy::ecs::change_detection::DetectChanges;
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::image::TextureFormatPixelInfo;
use bevy::prelude::{
    App, AssetPlugin, AssetServer, Assets, ButtonInput, Camera, Camera3d, Color, Commands,
    Component, ComputedNode, DefaultPlugins, Dir3, DirectionalLight, Entity, EnvironmentMapLight,
    FixedUpdate, GlobalAmbientLight, GlobalTransform, GltfAssetLabel, Handle, Image, ImageNode,
    IsDefaultUiCamera, KeyCode, MessageReader, MessageWriter, MinimalPlugins, Name, Node,
    PerspectiveProjection, Pickable, PluginGroup, PositionType, PostUpdate, Projection, Quat,
    Query, Res, ResMut, Resource, Scene, SceneRoot, Startup, Text, TextColor, TextFont,
    TextureAtlasLayout, Time, Transform, UVec2, UiRect, Update, Val, Vec2, Vec3, Visibility,
    Window, WindowPlugin, With, Without, default,
};
use bevy::render::render_resource::{TextureDimension, TextureFormat, TextureUsages};
#[cfg(not(target_arch = "wasm32"))]
use cached_path::{Cache, Error as CacheError, ProgressBar};
#[cfg(feature = "bevymon")]
use cu_bevymon::{
    CuBevyMonFocus, CuBevyMonPlugin, CuBevyMonSplitLayoutConfig, CuBevyMonSplitStyle,
    CuBevyMonSurface, CuBevyMonTexture, MonitorModel, MonitorUiOptions, spawn_split_layout,
};
use cu29::prelude::*;
#[cfg(all(not(target_arch = "wasm32"), feature = "sim"))]
use cu29_helpers::basic_copper_setup;

#[cfg(not(target_arch = "wasm32"))]
use crate::rc_joystick::{RcAxisBindings, RcFrame, RcJoystick};
use cu_crsf::messages::RcChannelsPayload;
use cu_msp_bridge::MspRequestBatch;
use cu_msp_lib::structs::{
    MSP_DP_CLEAR_SCREEN, MSP_DP_DRAW_SCREEN, MSP_DP_WRITE_STRING, MspDisplayPort, MspRequest,
};
use cu_sensor_payloads::{BarometerPayload, ImuPayload, MagnetometerPayload};

#[cfg(not(target_arch = "wasm32"))]
use std::fs;
#[cfg(not(target_arch = "wasm32"))]
use std::io;
#[cfg(not(target_arch = "wasm32"))]
use std::path::{Path, PathBuf};
#[cfg(feature = "bevymon")]
use std::sync::{Arc, Mutex};

#[copper_runtime(config = "copperconfig.ron", sim_mode = true, ignore_resources = true)]
struct FlightControllerSim {}

#[derive(Clone)]
struct SimVehicleState {
    position: Vec3,
    velocity_world: Vec3,
    rotation: Quat,
    body_accel_fc: [f32; 3],
    body_gyro_fc: [f32; 3],
}

impl Default for SimVehicleState {
    fn default() -> Self {
        Self {
            position: SIM_SPAWN_POSITION,
            velocity_world: Vec3::ZERO,
            rotation: spawn_rotation(),
            body_accel_fc: [0.0, 0.0, 9.81],
            body_gyro_fc: [0.0; 3],
        }
    }
}

#[derive(Resource)]
struct CopperState<T: Send + Sync + 'static> {
    _runtime_state: T,
    clock: RobotClock,
    app: gnss::FlightControllerSim,
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

#[derive(Resource)]
struct PendingQuadcopterSpawn {
    quadcopter_scene: Handle<Scene>,
    city_scene: Handle<Scene>,
}

#[derive(Resource, Clone, Copy, Debug, Default, PartialEq, Eq)]
enum RcInputSource {
    #[default]
    Keyboard,
    Joystick,
}

#[derive(Resource, Default)]
struct SimJoystickState {
    #[cfg(not(target_arch = "wasm32"))]
    reader: Option<RcJoystick>,
}

#[derive(Resource, Clone, Copy)]
struct WorldLayout {
    split_monitor: bool,
}

#[derive(Resource, Default)]
struct SceneLoadState {
    ready: bool,
}

#[derive(Resource)]
struct SimHudRoot(Entity);

#[derive(Resource, Default)]
struct SimHudSpawnState {
    loading: bool,
    help: bool,
    osd: bool,
}

#[cfg(feature = "bevymon")]
#[derive(Resource, Default)]
struct LayoutSpawned(bool);

#[cfg(feature = "bevymon")]
#[derive(Resource)]
struct SplitUiCamera(Entity);

#[derive(Resource, Clone, Copy, Debug, Default, PartialEq, Eq)]
enum CameraView {
    #[default]
    FirstPerson,
    ThirdPerson,
}

#[derive(Component)]
struct SimSceneCamera;

#[derive(Component)]
struct SplitSceneCamera;

#[derive(Component)]
struct SceneLoadingOverlay;

const OSD_COLS: usize = 53;
const OSD_ROWS: usize = 16;
const OSD_BLANK_SYMBOL: u8 = 0x20;
const OSD_FONT_ATLAS_COLS: u32 = 16;
const OSD_FONT_ATLAS_ROWS: u32 = 16;
const OSD_FONT_ATLAS_GLYPHS: usize = (OSD_FONT_ATLAS_COLS * OSD_FONT_ATLAS_ROWS) as usize;
const OSD_GLYPH_WIDTH_PX: u32 = 12;
const OSD_GLYPH_HEIGHT_PX: u32 = 18;
const OSD_GLYPH_PADDING_PX: u32 = 1;
const OSD_FONT_ATLAS_PATH: &str = "osd/vtx_font.png";
const OSD_CANVAS_WIDTH_PX: u32 = OSD_COLS as u32 * OSD_GLYPH_WIDTH_PX;
const OSD_CANVAS_HEIGHT_PX: u32 = OSD_ROWS as u32 * OSD_GLYPH_HEIGHT_PX;

#[derive(Resource, Clone)]
struct SimOsdOverlay {
    cols: usize,
    rows: usize,
    cells: Vec<u8>,
}

impl Default for SimOsdOverlay {
    fn default() -> Self {
        Self {
            cols: OSD_COLS,
            rows: OSD_ROWS,
            cells: vec![OSD_BLANK_SYMBOL; OSD_COLS * OSD_ROWS],
        }
    }
}

impl SimOsdOverlay {
    fn clear(&mut self) {
        for c in &mut self.cells {
            *c = OSD_BLANK_SYMBOL;
        }
    }

    fn apply_batch(&mut self, batch: &MspRequestBatch) {
        for request in batch.iter() {
            let MspRequest::MspDisplayPort(displayport) = request else {
                continue;
            };
            let _ = self.apply_displayport(displayport);
        }
    }

    fn apply_displayport(&mut self, displayport: &MspDisplayPort) -> bool {
        let payload = displayport.as_bytes();
        let Some(cmd) = payload.first().copied() else {
            return false;
        };

        match cmd {
            MSP_DP_CLEAR_SCREEN => {
                self.clear();
                true
            }
            MSP_DP_WRITE_STRING => {
                if payload.len() < 4 {
                    return false;
                }
                let row = payload[1] as usize;
                let col = payload[2] as usize;
                self.write_bytes(row, col, &payload[4..])
            }
            MSP_DP_DRAW_SCREEN => true,
            _ => false,
        }
    }

    fn write_bytes(&mut self, row: usize, col: usize, bytes: &[u8]) -> bool {
        if row >= self.rows || col >= self.cols || bytes.is_empty() {
            return false;
        }
        let mut written = false;
        for (i, byte) in bytes.iter().enumerate() {
            let x = col + i;
            if x >= self.cols {
                break;
            }
            let idx = row * self.cols + x;
            self.cells[idx] = *byte;
            written = true;
        }
        written
    }
}

#[derive(Component)]
struct OsdOverlayRoot;

#[derive(Component)]
struct OsdCanvasFrame;

#[derive(Component)]
struct OsdCanvasNode;

#[derive(Resource)]
struct OsdCanvasAssets {
    canvas: Handle<Image>,
    atlas: Handle<Image>,
    atlas_layout: Handle<TextureAtlasLayout>,
}

#[derive(Resource, Default)]
struct OsdRasterSource {
    atlas_width: usize,
    bytes_per_pixel: usize,
    rects: Vec<bevy::math::URect>,
    pixels: Vec<u8>,
    ready: bool,
}

#[derive(Component)]
struct SimHelpValuesText;

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
// Bevy world frame uses X east, Y up, Z south in this scene.
// Keep declination at 0 in the simulated magnetic field itself so
// `declination_deg` in `MagneticTrueHeading` can be tested independently.
const WORLD_MAG_FIELD_UT: [f32; 3] = [0.0, -45.0, -20.0];
#[cfg(not(target_arch = "wasm32"))]
const BASE_ASSETS_URL: &str = "https://cdn.copper-robotics.com/";
const SKYBOX: &str = "skybox.ktx2";
const SPECULAR_MAP: &str = "specular_map.ktx2";
const QUADCOPTER: &str = "quadcopter.glb";
const CITY: &str = "city-fixed.glb";
// Measured from `gltf-transform inspect city.glb` in source model units.
const LOCAL_CITY_BBOX_MIN_UNITS: Vec3 = Vec3::new(-30_614.165, -648.2196, -4_185.883);
const LOCAL_CITY_BBOX_MAX_UNITS: Vec3 = Vec3::new(18_754.953, 11_102.407, 35_871.875);
// Source appears to be authored in centimeters; convert units to meters.
const LOCAL_CITY_SCALE: f32 = 0.01;
const SIM_SPAWN_POSITION: Vec3 = Vec3::new(-10.0, 1.0, 20.0);
const SIM_SPAWN_YAW_DEG: f32 = 180.0;
#[cfg(not(target_arch = "wasm32"))]
const ARM_SWITCH_NAMES: &[&str] = &["sf", "se", "arm", "btn1"];
// With the corrected 0.44 kg sim mass and 10% airmode idle, hover is about 0.48.
// Keep keyboard idle slightly below hover so release-to-descend works again.
const KEYBOARD_HOVER_THROTTLE_LOW: f32 = 0.47;
const KEYBOARD_HOVER_THROTTLE_HIGH: f32 = 0.50;

fn spawn_rotation() -> Quat {
    Quat::from_rotation_y(SIM_SPAWN_YAW_DEG.to_radians())
}

fn spawn_pose_components() -> (
    Transform,
    Position,
    Rotation,
    LinearVelocity,
    AngularVelocity,
) {
    (
        Transform::from_translation(SIM_SPAWN_POSITION).with_rotation(spawn_rotation()),
        Position::from_xyz(
            SIM_SPAWN_POSITION.x,
            SIM_SPAWN_POSITION.y,
            SIM_SPAWN_POSITION.z,
        ),
        Rotation(spawn_rotation()),
        LinearVelocity(Vec3::ZERO),
        AngularVelocity(Vec3::ZERO),
    )
}

#[cfg(not(target_arch = "wasm32"))]
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

#[cfg(not(target_arch = "wasm32"))]
fn get_asset_path(
    online_cache: &Cache,
    offline_cache: &Cache,
    asset_url: &str,
    asset_name: &str,
) -> Result<PathBuf, CacheError> {
    match online_cache.cached_path(asset_url) {
        Ok(path) => Ok(path),
        Err(err) => {
            if matches!(
                err,
                CacheError::HttpError(_) | CacheError::IoError(_) | CacheError::ResourceNotFound(_)
            ) {
                eprintln!(
                    "Failed to fetch latest '{}' from network ({}). Attempting to use cached version.",
                    asset_name, err
                );
                offline_cache.cached_path(asset_url)
            } else {
                Err(err)
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn precached_asset_path(
    online_cache: &Cache,
    offline_cache: &Cache,
    asset_name: &str,
) -> Result<PathBuf, CacheError> {
    let asset_url = format!("{BASE_ASSETS_URL}{asset_name}");
    let hashed_path = get_asset_path(online_cache, offline_cache, &asset_url, asset_name)?;
    let plain_path = hashed_path.parent().unwrap().join(asset_name);
    create_symlink(hashed_path.to_str().unwrap(), plain_path.to_str().unwrap())
        .expect("failed to create cached-asset symlink");
    Ok(plain_path)
}

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

fn default_callback(_step: gnss::SimStep) -> SimOverride {
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

/// Map Bevy body polar vectors (X right, Y up, Z forward) to FC AHRS body frame
/// (X forward, Y right, Z down; aerospace/NED).
fn map_bevy_body_to_fc_polar(v: Vec3) -> [f32; 3] {
    [v.z, v.x, -v.y]
}

/// Map Bevy body axial vectors (angular velocity) into FC body frame.
/// Because the frame transform changes handedness, axial vectors pick up an extra sign.
fn map_bevy_body_to_fc_axial(v: Vec3) -> [f32; 3] {
    [-v.z, -v.x, v.y]
}

/// Map Bevy body magnetic vectors into FC body frame.
/// Magnetometer is a polar vector (not axial), so it uses the same mapping as accel.
fn map_bevy_body_to_fc_magnetometer(v: Vec3) -> [f32; 3] {
    map_bevy_body_to_fc_polar(v)
}

#[cfg(feature = "bevymon")]
#[cfg(target_arch = "wasm32")]
type BevyMonSectionStorage = NoopSectionStorage;
#[cfg(feature = "bevymon")]
#[cfg(target_arch = "wasm32")]
type BevyMonUnifiedLogger = NoopLogger;

#[cfg(feature = "bevymon")]
#[cfg(not(target_arch = "wasm32"))]
type BevyMonSectionStorage = memmap::MmapSectionStorage;
#[cfg(feature = "bevymon")]
#[cfg(not(target_arch = "wasm32"))]
type BevyMonUnifiedLogger = UnifiedLoggerWrite;

#[cfg(all(not(target_arch = "wasm32"), feature = "sim"))]
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
    let clock = ctx.clock.clone();

    let mut app = gnss::FlightControllerSimBuilder::new()
        .with_context(&ctx)
        .with_sim_callback(&mut default_callback)
        .build()
        .expect("failed to create runtime");

    app.start_all_tasks(&mut default_callback)
        .expect("failed to start tasks");

    commands.insert_resource(CopperState {
        _runtime_state: ctx,
        clock,
        app,
    });
}

#[cfg(feature = "bevymon")]
fn build_bevymon_copper() -> (MonitorModel, CopperState<LoggerRuntime>) {
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(128 * 1024 * 1024);
    const STRUCTURED_LOG_SECTION_SIZE: usize = 4096 * 10;

    let clock = RobotClock::default();
    let unified_logger = build_unified_logger(LOG_SLAB_SIZE).expect("failed to create logger");
    let logger_runtime =
        init_logger_runtime(&clock, unified_logger.clone(), STRUCTURED_LOG_SECTION_SIZE)
            .expect("failed to initialize structured logger");

    let mut sim_callback = default_callback;
    let mut app = <gnss::FlightControllerSim as CuSimApplication<
        BevyMonSectionStorage,
        BevyMonUnifiedLogger,
    >>::new(clock.clone(), unified_logger, None, &mut sim_callback)
    .expect("failed to create runtime");

    app.start_all_tasks(&mut sim_callback)
        .expect("failed to start tasks");

    let monitor_model = app.copper_runtime_mut().monitor.model();
    (
        monitor_model,
        CopperState {
            _runtime_state: logger_runtime,
            clock,
            app,
        },
    )
}

#[cfg(feature = "bevymon")]
fn init_logger_runtime(
    clock: &RobotClock,
    unified_logger: Arc<Mutex<BevyMonUnifiedLogger>>,
    structured_log_section_size: usize,
) -> CuResult<LoggerRuntime> {
    let structured_stream = stream_write::<CuLogEntry, BevyMonSectionStorage>(
        unified_logger,
        UnifiedLogType::StructuredLogLine,
        structured_log_section_size,
    )?;
    Ok(LoggerRuntime::init(
        clock.clone(),
        structured_stream,
        None::<NullLog>,
    ))
}

#[cfg(feature = "bevymon")]
#[cfg(target_arch = "wasm32")]
fn build_unified_logger(
    _log_slab_size: Option<usize>,
) -> CuResult<Arc<Mutex<BevyMonUnifiedLogger>>> {
    Ok(Arc::new(Mutex::new(NoopLogger::new())))
}

#[cfg(feature = "bevymon")]
#[cfg(not(target_arch = "wasm32"))]
fn build_unified_logger(
    log_slab_size: Option<usize>,
) -> CuResult<Arc<Mutex<BevyMonUnifiedLogger>>> {
    let logger_path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("logs/flight_controller_sim.copper");
    if let Some(parent) = logger_path.parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("failed to create logs directory");
    }

    let logger = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_base_name(&logger_path)
        .preallocated_size(log_slab_size.unwrap_or(10 * 1024 * 1024))
        .build()
        .map_err(|err| CuError::new_with_cause("failed to create flight controller logger", err))?;
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

fn setup_world(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut images: ResMut<Assets<Image>>,
    layout: Res<WorldLayout>,
) {
    #[cfg(not(target_arch = "wasm32"))]
    let online_cache = Cache::builder()
        .progress_bar(Some(ProgressBar::Full))
        .build()
        .expect("failed to create online file cache");

    #[cfg(not(target_arch = "wasm32"))]
    let offline_cache = Cache::builder()
        .progress_bar(Some(ProgressBar::Full))
        .offline(true)
        .build()
        .expect("failed to create offline file cache");

    #[cfg(not(target_arch = "wasm32"))]
    let quadcopter_path = precached_asset_path(&online_cache, &offline_cache, QUADCOPTER)
        .expect("failed to get quadcopter.glb (online or cached)");
    #[cfg(not(target_arch = "wasm32"))]
    let skybox_path = precached_asset_path(&online_cache, &offline_cache, SKYBOX)
        .expect("failed to get skybox.ktx2 (online or cached)");
    #[cfg(not(target_arch = "wasm32"))]
    let specular_map_path = precached_asset_path(&online_cache, &offline_cache, SPECULAR_MAP)
        .expect("failed to get specular_map.ktx2 (online or cached)");
    #[cfg(not(target_arch = "wasm32"))]
    let city_path = precached_asset_path(&online_cache, &offline_cache, CITY)
        .expect("failed to get city.glb (online or cached)");
    #[cfg(target_arch = "wasm32")]
    let quadcopter_path = QUADCOPTER;
    #[cfg(target_arch = "wasm32")]
    let skybox_path = SKYBOX;
    #[cfg(target_arch = "wasm32")]
    let specular_map_path = SPECULAR_MAP;
    #[cfg(target_arch = "wasm32")]
    let city_path = CITY;

    let city_size_units = LOCAL_CITY_BBOX_MAX_UNITS - LOCAL_CITY_BBOX_MIN_UNITS;
    let city_size_m = city_size_units * LOCAL_CITY_SCALE;
    let city_translation = Vec3::ZERO;
    let city_scale = Vec3::splat(LOCAL_CITY_SCALE);
    #[cfg(not(target_arch = "wasm32"))]
    let city_path_str = city_path.to_string_lossy().into_owned();
    #[cfg(target_arch = "wasm32")]
    let city_path_str = city_path.to_string();
    info!(
        "sim world: loading city {} (bbox {}x{}x{} units, scaled to {}x{}x{} m) with translation ({}, {}, {})",
        city_path_str,
        city_size_units.x,
        city_size_units.y,
        city_size_units.z,
        city_size_m.x,
        city_size_m.y,
        city_size_m.z,
        city_translation.x,
        city_translation.y,
        city_translation.z
    );

    let skybox_handle = asset_server.load(skybox_path);
    let specular_map_handle = asset_server.load(specular_map_path);

    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 1.0 / 5.0,
        affects_lightmapped_meshes: true,
    });

    let split_target = if layout.split_monitor {
        let mut image = Image::new_uninit(
            default(),
            TextureDimension::D2,
            TextureFormat::Bgra8UnormSrgb,
            RenderAssetUsages::all(),
        );
        image.texture_descriptor.usage = TextureUsages::TEXTURE_BINDING
            | TextureUsages::COPY_DST
            | TextureUsages::RENDER_ATTACHMENT;
        Some(images.add(image))
    } else {
        None
    };

    let mut camera = commands.spawn((
        Name::new("camera"),
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: 90.0_f32.to_radians(),
            ..default()
        }),
        Skybox {
            image: skybox_handle.clone(),
            brightness: 1000.0,
            ..default()
        },
        EnvironmentMapLight {
            diffuse_map: skybox_handle.clone(),
            specular_map: specular_map_handle,
            intensity: 900.0,
            ..default()
        },
        Transform::from_xyz(-2.0, 1.6, -2.0).looking_at(Vec3::ZERO, Vec3::Y),
        SimSceneCamera,
    ));
    if layout.split_monitor {
        camera.insert((
            Camera {
                order: 0,
                ..default()
            },
            RenderTarget::Image(split_target.expect("split viewport target missing").into()),
            SplitSceneCamera,
        ));
    } else {
        camera.insert(IsDefaultUiCamera);
    }

    commands.spawn((
        Name::new("sun"),
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_translation(Vec3::new(3.0, 10.0, 1.0)).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    #[cfg(not(target_arch = "wasm32"))]
    let quadcopter_scene_path = format!("{}#scene0", quadcopter_path.display());
    #[cfg(target_arch = "wasm32")]
    let quadcopter_scene_path = format!("{quadcopter_path}#scene0");
    #[cfg(not(target_arch = "wasm32"))]
    let city_scene_path = format!("{}#scene0", city_path.display());
    #[cfg(target_arch = "wasm32")]
    let city_scene_path = format!("{city_path}#scene0");

    let quadcopter_scene =
        asset_server.load(GltfAssetLabel::Scene(0).from_asset(quadcopter_scene_path));
    let city_scene = asset_server.load(GltfAssetLabel::Scene(0).from_asset(city_scene_path));
    commands.insert_resource(PendingQuadcopterSpawn {
        quadcopter_scene: quadcopter_scene.clone(),
        city_scene: city_scene.clone(),
    });

    commands.spawn((
        Name::new("city"),
        SceneRoot(city_scene),
        Transform {
            translation: city_translation,
            scale: city_scale,
            ..default()
        },
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMesh),
        RigidBody::Static,
    ));
}

fn spawn_quadcopter_when_world_ready(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    pending_spawn: Option<Res<PendingQuadcopterSpawn>>,
    colliders: Query<(), (With<Collider>, Without<Multicopter>)>,
    mut load_state: ResMut<SceneLoadState>,
) {
    let Some(pending_spawn) = pending_spawn else {
        return;
    };

    if !asset_server.is_loaded_with_dependencies(pending_spawn.city_scene.id())
        || !asset_server.is_loaded_with_dependencies(pending_spawn.quadcopter_scene.id())
        || colliders.is_empty()
    {
        return;
    }

    let (transform, position, rotation, lin_vel, ang_vel) = spawn_pose_components();
    commands
        .spawn((
            Name::new("quadcopter"),
            SceneRoot(pending_spawn.quadcopter_scene.clone()),
            RigidBody::Dynamic,
            transform,
            position,
            rotation,
            lin_vel,
            ang_vel,
            Collider::cuboid(0.14, 0.05, 0.14),
            Multicopter::new(quad_x_propellers()),
            Mass(0.44),
            AngularInertia::new(Vec3::new(0.012, 0.02, 0.012)),
            AngularDamping(0.4),
            LinearDamping(0.2),
            SweptCcd::NON_LINEAR,
        ))
        .insert(Visibility::Visible);
    commands.remove_resource::<PendingQuadcopterSpawn>();
    load_state.ready = true;
}

fn setup_full_window_hud_root(mut commands: Commands, layout: Res<WorldLayout>) {
    if layout.split_monitor {
        return;
    }

    let root = commands
        .spawn((
            Name::new("sim-ui-root"),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(0.0),
                left: Val::Px(0.0),
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                ..default()
            },
            Pickable::IGNORE,
        ))
        .id();
    commands.insert_resource(SimHudRoot(root));
}

fn spawn_osd_overlay(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut texture_atlases: ResMut<Assets<TextureAtlasLayout>>,
    mut images: ResMut<Assets<Image>>,
    hud_root: Option<Res<SimHudRoot>>,
    mut spawned: ResMut<SimHudSpawnState>,
) {
    if spawned.osd {
        return;
    }
    let Some(hud_root) = hud_root else {
        return;
    };

    let atlas_handle = asset_server.load(OSD_FONT_ATLAS_PATH);
    let atlas_layout = texture_atlases.add(TextureAtlasLayout::from_grid(
        UVec2::new(OSD_GLYPH_WIDTH_PX, OSD_GLYPH_HEIGHT_PX),
        OSD_FONT_ATLAS_COLS,
        OSD_FONT_ATLAS_ROWS,
        Some(UVec2::splat(OSD_GLYPH_PADDING_PX)),
        None,
    ));
    let mut canvas_image = Image::new_fill(
        bevy::render::render_resource::Extent3d {
            width: OSD_CANVAS_WIDTH_PX,
            height: OSD_CANVAS_HEIGHT_PX,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &[0, 0, 0, 0],
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    );
    canvas_image.sampler = bevy::image::ImageSampler::nearest();
    let canvas_handle = images.add(canvas_image);

    commands.insert_resource(OsdCanvasAssets {
        canvas: canvas_handle.clone(),
        atlas: atlas_handle.clone(),
        atlas_layout: atlas_layout.clone(),
    });

    commands.entity(hud_root.0).with_children(|parent| {
        parent
            .spawn((
                Name::new("fpv-osd"),
                OsdOverlayRoot,
                Node {
                    position_type: PositionType::Absolute,
                    top: Val::Px(0.0),
                    left: Val::Px(0.0),
                    width: Val::Percent(100.0),
                    height: Val::Percent(100.0),
                    ..default()
                },
                Pickable::IGNORE,
                Visibility::Visible,
            ))
            .with_children(|canvas| {
                canvas
                    .spawn((
                        OsdCanvasFrame,
                        Node {
                            position_type: PositionType::Absolute,
                            top: Val::Px(0.0),
                            left: Val::Px(0.0),
                            width: Val::Px(OSD_CANVAS_WIDTH_PX as f32),
                            height: Val::Px(OSD_CANVAS_HEIGHT_PX as f32),
                            ..default()
                        },
                        Pickable::IGNORE,
                    ))
                    .with_children(|frame| {
                        frame.spawn((
                            OsdCanvasNode,
                            Node {
                                width: Val::Percent(100.0),
                                height: Val::Percent(100.0),
                                ..default()
                            },
                            Pickable::IGNORE,
                            ImageNode::new(canvas_handle.clone()),
                        ));
                    });
            });
    });
    spawned.osd = true;
}

fn spawn_loading_overlay(
    mut commands: Commands,
    hud_root: Option<Res<SimHudRoot>>,
    mut spawned: ResMut<SimHudSpawnState>,
) {
    if spawned.loading {
        return;
    }
    let Some(hud_root) = hud_root else {
        return;
    };

    commands.entity(hud_root.0).with_children(|parent| {
        parent
            .spawn((
                Node {
                    position_type: PositionType::Absolute,
                    top: Val::Px(18.0),
                    left: Val::Px(0.0),
                    right: Val::Px(0.0),
                    justify_content: bevy::ui::JustifyContent::Center,
                    ..default()
                },
                Pickable::IGNORE,
            ))
            .with_children(|loading| {
                loading
                    .spawn((
                        Node {
                            padding: UiRect::new(
                                Val::Px(18.0),
                                Val::Px(18.0),
                                Val::Px(10.0),
                                Val::Px(10.0),
                            ),
                            border: UiRect::all(Val::Px(2.0)),
                            border_radius: bevy::ui::BorderRadius::all(Val::Px(12.0)),
                            ..default()
                        },
                        Pickable::IGNORE,
                        SceneLoadingOverlay,
                        bevy::ui::BackgroundColor(Color::srgba(0.03, 0.05, 0.09, 0.92)),
                        bevy::ui::BorderColor::all(Color::srgba(0.58, 0.74, 0.96, 0.95)),
                    ))
                    .with_children(|cartouche| {
                        cartouche.spawn((
                            Pickable::IGNORE,
                            Text::new("Assets loading..."),
                            TextFont {
                                font_size: 18.0,
                                ..default()
                            },
                            TextColor(Color::srgb(0.93, 0.96, 1.0)),
                        ));
                    });
            });
    });
    spawned.loading = true;
}

fn spawn_help_overlay(
    mut commands: Commands,
    hud_root: Option<Res<SimHudRoot>>,
    mut spawned: ResMut<SimHudSpawnState>,
) {
    if spawned.help {
        return;
    }
    let Some(hud_root) = hud_root else {
        return;
    };

    commands.entity(hud_root.0).with_children(|parent| {
        parent
            .spawn((
                Name::new("sim-help"),
                Node {
                    position_type: PositionType::Absolute,
                    bottom: Val::Px(5.0),
                    right: Val::Px(5.0),
                    padding: UiRect::new(
                        Val::Px(15.0),
                        Val::Px(15.0),
                        Val::Px(10.0),
                        Val::Px(10.0),
                    ),
                    column_gap: Val::Px(10.0),
                    flex_direction: bevy::ui::FlexDirection::Row,
                    justify_content: bevy::ui::JustifyContent::SpaceBetween,
                    border: UiRect::all(Val::Px(2.0)),
                    border_radius: bevy::ui::BorderRadius::all(Val::Px(10.0)),
                    ..default()
                },
                Pickable::IGNORE,
                bevy::ui::BackgroundColor(Color::srgba(0.25, 0.41, 0.88, 0.7)),
                bevy::ui::BorderColor::all(Color::srgba(0.8, 0.8, 0.8, 0.7)),
            ))
            .with_children(|help| {
                help.spawn((
                    Pickable::IGNORE,
                    Text::new("View\nRC Link\nArm\nMode\nThrottle\nRoll/Pitch\nYaw\nReset"),
                    TextFont {
                        font_size: 12.0,
                        ..default()
                    },
                    TextColor(Color::srgba(0.25, 0.25, 0.75, 1.0)),
                ));

                help.spawn((
                    Pickable::IGNORE,
                    SimHelpValuesText,
                    Text::new("FPV (V)\nChecking RC link..."),
                    TextFont {
                        font_size: 12.0,
                        ..default()
                    },
                    TextColor(Color::WHITE),
                ));
            });
    });
    spawned.help = true;
}

#[cfg(not(target_arch = "wasm32"))]
fn setup_joystick(
    mut rc_input: ResMut<SimRcInput>,
    mut rc_source: ResMut<RcInputSource>,
    mut joystick_state: ResMut<SimJoystickState>,
) {
    let preferred = std::env::var("CU_SIM_JOYSTICK").ok();
    match RcJoystick::open(preferred.as_deref()) {
        Ok(joystick) => {
            apply_joystick_frame(&joystick.current_frame(), &mut rc_input);
            *rc_source = RcInputSource::Joystick;
            info!(
                "sim rc: joystick source active: {} (set CU_SIM_JOYSTICK=<name> to target a device)",
                joystick.device_name()
            );
            joystick_state.reader = Some(joystick);
        }
        Err(err) => {
            *rc_source = RcInputSource::Keyboard;
            rc_input.mode = messages::FlightMode::Angle;
            rc_input.armed = true;
            rc_input.throttle = KEYBOARD_HOVER_THROTTLE_LOW;
            info!(
                "sim rc: joystick unavailable ({}), using keyboard controls (auto-armed, hover throttle) (set CU_SIM_ALLOW_GENERIC_JOYSTICK=1 to allow non-radio joysticks)",
                err.to_string()
            );
            joystick_state.reader = None;
        }
    }
}

#[cfg(target_arch = "wasm32")]
fn setup_joystick(mut rc_input: ResMut<SimRcInput>, mut rc_source: ResMut<RcInputSource>) {
    *rc_source = RcInputSource::Keyboard;
    rc_input.mode = messages::FlightMode::Angle;
    rc_input.armed = true;
    rc_input.throttle = KEYBOARD_HOVER_THROTTLE_LOW;
    info!("sim rc: web build using keyboard controls");
}

#[cfg(not(target_arch = "wasm32"))]
fn poll_joystick(
    mut joystick: ResMut<SimJoystickState>,
    mut rc_input: ResMut<SimRcInput>,
    mut rc_source: ResMut<RcInputSource>,
) {
    let Some(reader) = joystick.reader.as_mut() else {
        return;
    };

    match reader.next_frame() {
        Ok(Some(frame)) => {
            let prev_armed = rc_input.armed;
            let prev_mode = rc_input.mode;
            apply_joystick_frame(&frame, &mut rc_input);
            *rc_source = RcInputSource::Joystick;
            if rc_input.armed != prev_armed || rc_input.mode != prev_mode {
                info!("sim rc: armed={} mode={:?}", rc_input.armed, rc_input.mode);
            }
        }
        Ok(None) => {}
        Err(err) => {
            info!(
                "sim rc: joystick read failed ({}), falling back to keyboard",
                err.to_string()
            );
            joystick.reader = None;
            *rc_source = RcInputSource::Keyboard;
            rc_input.mode = messages::FlightMode::Angle;
            rc_input.armed = true;
            rc_input.throttle = KEYBOARD_HOVER_THROTTLE_LOW;
        }
    }
}

#[cfg(target_arch = "wasm32")]
fn poll_joystick() {}

#[cfg(not(target_arch = "wasm32"))]
fn mode_from_three_pos(value: f32) -> messages::FlightMode {
    if value < -0.33 {
        messages::FlightMode::Acro
    } else if value < 0.33 {
        messages::FlightMode::Angle
    } else {
        messages::FlightMode::PositionHold
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn find_arm_switch(frame: &RcFrame) -> Option<&rc_joystick::SwitchState> {
    for name in ARM_SWITCH_NAMES {
        if let Some(sw) = frame
            .switches
            .iter()
            .find(|s| s.name.eq_ignore_ascii_case(name))
        {
            return Some(sw);
        }
    }

    frame.switches.first()
}

#[cfg(not(target_arch = "wasm32"))]
fn arm_from_switches(frame: &RcFrame) -> Option<bool> {
    find_arm_switch(frame).map(|s| s.on)
}

#[cfg(not(target_arch = "wasm32"))]
fn apply_joystick_frame(frame: &RcFrame, rc_input: &mut SimRcInput) {
    rc_input.roll = frame.roll.clamp(-1.0, 1.0);
    // Match FC stick convention used by keyboard path and RcMapper.
    rc_input.pitch = (-frame.pitch).clamp(-1.0, 1.0);
    rc_input.yaw = (-frame.yaw).clamp(-1.0, 1.0);
    rc_input.throttle = frame.throttle.clamp(0.0, 1.0);

    let armed_from_switch = arm_from_switches(frame);
    let arm_uses_sa_axis = armed_from_switch.is_none();
    rc_input.armed = armed_from_switch.unwrap_or(frame.knob_sa > 0.5);

    let mode_axis = if arm_uses_sa_axis {
        frame.knob_sb
    } else {
        frame.knob_sa
    };
    rc_input.mode = mode_from_three_pos(mode_axis);
}

fn update_rc_input_keyboard(
    mut rc_input: ResMut<SimRcInput>,
    keyboard: Res<ButtonInput<KeyCode>>,
    rc_source: Res<RcInputSource>,
    _layout: Res<WorldLayout>,
    #[cfg(feature = "bevymon")] focus: Option<Res<CuBevyMonFocus>>,
) {
    #[cfg(feature = "bevymon")]
    if _layout.split_monitor && !focus.is_some_and(|focus| focus.0 == CuBevyMonSurface::Sim) {
        return;
    }

    if *rc_source == RcInputSource::Joystick {
        return;
    }

    let roll =
        (keyboard.pressed(KeyCode::KeyD) as i8 - keyboard.pressed(KeyCode::KeyA) as i8) as f32;
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
        if rc_input.armed {
            // Keyboard arming should start in a safe stabilized mode.
            rc_input.mode = messages::FlightMode::Angle;
        }
        info!("sim rc: armed={} mode={:?}", rc_input.armed, rc_input.mode);
    }
}

fn adjust_keyboard_throttle(
    mut rc_input: ResMut<SimRcInput>,
    keyboard: Res<ButtonInput<KeyCode>>,
    rc_source: Res<RcInputSource>,
    _layout: Res<WorldLayout>,
    #[cfg(feature = "bevymon")] focus: Option<Res<CuBevyMonFocus>>,
) {
    #[cfg(feature = "bevymon")]
    if _layout.split_monitor && !focus.is_some_and(|focus| focus.0 == CuBevyMonSurface::Sim) {
        return;
    }

    if *rc_source == RcInputSource::Joystick {
        return;
    }

    // Keyboard mode is a simple "descend a bit / climb a bit" control around hover.
    rc_input.throttle = if keyboard.pressed(KeyCode::Space) {
        KEYBOARD_HOVER_THROTTLE_HIGH
    } else {
        KEYBOARD_HOVER_THROTTLE_LOW
    };
}

fn reset_vehicle(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<
        (
            &mut Transform,
            &mut Position,
            &mut Rotation,
            &mut LinearVelocity,
            &mut AngularVelocity,
        ),
        With<Multicopter>,
    >,
    mut motors: ResMut<SimMotorCommands>,
    mut kin: ResMut<SimKinematics>,
    _layout: Res<WorldLayout>,
    #[cfg(feature = "bevymon")] focus: Option<Res<CuBevyMonFocus>>,
) {
    #[cfg(feature = "bevymon")]
    if _layout.split_monitor && !focus.is_some_and(|focus| focus.0 == CuBevyMonSurface::Sim) {
        return;
    }

    if !keyboard.just_pressed(KeyCode::KeyR) {
        return;
    }

    if let Ok((mut transform, mut position, mut rotation, mut lin_vel, mut ang_vel)) =
        query.single_mut()
    {
        let (spawn_tf, spawn_pos, spawn_rot, spawn_lin_vel, spawn_ang_vel) =
            spawn_pose_components();
        *transform = spawn_tf;
        *position = spawn_pos;
        *rotation = spawn_rot;
        *lin_vel = spawn_lin_vel;
        *ang_vel = spawn_ang_vel;
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
    // cu-ahrs expects accelerometer values in NED-style body axes where
    // level flight is approximately [0, 0, +9.81]. In Bevy world (Y up),
    // this corresponds to gravity minus linear acceleration.
    let accel_measure_world = gravity.0 - accel_world;

    let rotation = transform.rotation();
    let accel_body = rotation.inverse() * accel_measure_world;
    let gyro_body = rotation.inverse() * angular_velocity.0;

    state.vehicle = SimVehicleState {
        position: transform.translation(),
        velocity_world: lv,
        rotation,
        body_accel_fc: map_bevy_body_to_fc_polar(accel_body),
        body_gyro_fc: map_bevy_body_to_fc_axial(gyro_body),
    };
}

fn run_copper<T: Send + Sync + 'static>(
    mut copper: ResMut<CopperState<T>>,
    sim_state: Res<SimState>,
    rc_input: Res<SimRcInput>,
    mut motor_commands: ResMut<SimMotorCommands>,
    mut osd_overlay: ResMut<SimOsdOverlay>,
    mut exit_writer: MessageWriter<AppExit>,
) {
    let vehicle = sim_state.vehicle.clone();
    let rc = rc_input.clone();
    sim_support::sim_battery_set_armed(rc.armed);
    sim_support::sim_battery_set_throttle(rc.throttle);
    sim_support::sim_gnss_set_vehicle_state(
        [vehicle.position.x, vehicle.position.y, vehicle.position.z],
        [
            vehicle.velocity_world.x,
            vehicle.velocity_world.y,
            vehicle.velocity_world.z,
        ],
    );
    let clock = copper.clock.clone();
    let dshot = &mut motor_commands.dshot;

    let mut sim_callback = move |step: gnss::SimStep| -> SimOverride {
        match step {
            gnss::SimStep::Bmi088(CuTaskCallbackState::Process(_, output)) => {
                set_msg_timing(&clock, output);
                output.set_payload(ImuPayload::from_raw(
                    vehicle.body_accel_fc,
                    vehicle.body_gyro_fc,
                    29.0,
                ));
                SimOverride::ExecutedBySim
            }
            gnss::SimStep::Dps310(CuTaskCallbackState::Process(_, output)) => {
                let altitude_m = vehicle.position.y.max(-100.0);
                let pressure_pa = 101_325.0 * (1.0 - altitude_m / 44_330.0).powf(5.255);
                set_msg_timing(&clock, output);
                output.set_payload(BarometerPayload::from_raw(pressure_pa, 25.0));
                SimOverride::ExecutedBySim
            }
            gnss::SimStep::Ist8310(CuTaskCallbackState::Process(_, output)) => {
                let world_mag = Vec3::from_array(WORLD_MAG_FIELD_UT);
                // Convert world vector into body frame using the same world->body convention as IMU.
                let body_mag = vehicle.rotation.inverse() * world_mag;
                set_msg_timing(&clock, output);
                output.set_payload(MagnetometerPayload::from_raw(
                    map_bevy_body_to_fc_magnetometer(body_mag),
                ));
                SimOverride::ExecutedBySim
            }
            gnss::SimStep::RcRxRcRx { msg, .. } => {
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
            gnss::SimStep::BdshotTxEsc0Tx { msg, .. } => {
                dshot[0] = msg.payload().map_or(0, |c| c.throttle);
                SimOverride::ExecuteByRuntime
            }
            gnss::SimStep::BdshotTxEsc1Tx { msg, .. } => {
                dshot[1] = msg.payload().map_or(0, |c| c.throttle);
                SimOverride::ExecuteByRuntime
            }
            gnss::SimStep::BdshotTxEsc2Tx { msg, .. } => {
                dshot[2] = msg.payload().map_or(0, |c| c.throttle);
                SimOverride::ExecuteByRuntime
            }
            gnss::SimStep::BdshotTxEsc3Tx { msg, .. } => {
                dshot[3] = msg.payload().map_or(0, |c| c.throttle);
                SimOverride::ExecuteByRuntime
            }
            gnss::SimStep::VtxMspTxRequests { msg, .. } => {
                if let Some(batch) = msg.payload() {
                    osd_overlay.apply_batch(batch);
                }
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

fn toggle_camera_view(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut camera_view: ResMut<CameraView>,
    _layout: Res<WorldLayout>,
    #[cfg(feature = "bevymon")] focus: Option<Res<CuBevyMonFocus>>,
) {
    #[cfg(feature = "bevymon")]
    if _layout.split_monitor && !focus.is_some_and(|focus| focus.0 == CuBevyMonSurface::Sim) {
        return;
    }

    if !keyboard.just_pressed(KeyCode::KeyV) {
        return;
    }

    *camera_view = match *camera_view {
        CameraView::FirstPerson => CameraView::ThirdPerson,
        CameraView::ThirdPerson => CameraView::FirstPerson,
    };
    let mode = match *camera_view {
        CameraView::FirstPerson => "first_person",
        CameraView::ThirdPerson => "third_person",
    };
    info!("sim camera: {}", mode);
}

fn camera_follow_quadcopter(
    quadcopter_query: Query<&GlobalTransform, With<Multicopter>>,
    mut camera_query: Query<&mut Transform, (With<SimSceneCamera>, Without<Multicopter>)>,
    camera_view: Res<CameraView>,
) {
    let Ok(quad_tf) = quadcopter_query.single() else {
        return;
    };
    let Ok(mut camera_tf) = camera_query.single_mut() else {
        return;
    };

    let camera_position = match *camera_view {
        CameraView::FirstPerson => {
            quad_tf.translation() + 0.10 * quad_tf.up() + 0.16 * quad_tf.forward()
        }
        CameraView::ThirdPerson => {
            quad_tf.translation() + -2.0 * quad_tf.forward() + 1.0 * quad_tf.up()
        }
    };
    camera_tf.translation = camera_position;
    *camera_tf = camera_tf.looking_to(quad_tf.forward(), quad_tf.up());
}

fn update_quadcopter_visibility(
    camera_view: Res<CameraView>,
    mut quad_query: Query<&mut Visibility, With<Multicopter>>,
) {
    let Ok(mut visibility) = quad_query.single_mut() else {
        return;
    };
    *visibility = if *camera_view == CameraView::FirstPerson {
        Visibility::Hidden
    } else {
        Visibility::Visible
    };
}

fn track_sim_led_state() {
    let _on = sim_support::sim_activity_led_is_on();
}

#[cfg(not(target_arch = "wasm32"))]
fn axis_or_unmapped(axis: Option<&String>) -> &str {
    axis.map_or("unmapped", String::as_str)
}

#[cfg(not(target_arch = "wasm32"))]
fn connected_help_values(view_label: &str, joystick: &RcJoystick) -> String {
    let axes: RcAxisBindings = joystick.axis_bindings();
    let frame = joystick.current_frame();
    let arm_switch = find_arm_switch(&frame);
    let device_name = joystick.device_name();

    let arm_line = if let Some(sw) = arm_switch {
        format!("switch {}", sw.name)
    } else {
        format!("knob_sa {} > 0.5", axis_or_unmapped(axes.knob_sa.as_ref()))
    };

    let mode_line = if arm_switch.is_some() {
        format!(
            "knob_sa {} (3-pos)",
            axis_or_unmapped(axes.knob_sa.as_ref())
        )
    } else {
        format!(
            "knob_sb {} (3-pos)",
            axis_or_unmapped(axes.knob_sb.as_ref())
        )
    };

    format!(
        "{view_label}\nConnected ({device_name})\n{arm_line}\n{mode_line}\n{}\n{} / {}\n{}\nR",
        axis_or_unmapped(axes.throttle.as_ref()),
        axis_or_unmapped(axes.roll.as_ref()),
        axis_or_unmapped(axes.pitch.as_ref()),
        axis_or_unmapped(axes.yaw.as_ref()),
    )
}

fn update_help_overlay(
    camera_view: Res<CameraView>,
    rc_source: Res<RcInputSource>,
    _joystick_state: Res<SimJoystickState>,
    mut help_text_query: Query<&mut Text, With<SimHelpValuesText>>,
) {
    let Ok(mut text) = help_text_query.single_mut() else {
        return;
    };

    let view_label = match *camera_view {
        CameraView::FirstPerson => "FPV (V -> 3RD)",
        CameraView::ThirdPerson => "3RD (V -> FPV)",
    };

    let values = match *rc_source {
        RcInputSource::Keyboard => format!(
            "{view_label}\nNot connected (plug RC via USB/BT)\nT\n1=Acro 2=Angle 3=PosHold\nSpace (boost)\nWASD\nQ / E\nR"
        ),
        RcInputSource::Joystick => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                _joystick_state.reader.as_ref().map_or_else(
                    || {
                        format!(
                            "{view_label}\nConnected (USB/BT)\nRC source initializing...\n-\n-\n-\n-\nR"
                        )
                    },
                    |joy| connected_help_values(view_label, joy),
                )
            }
            #[cfg(target_arch = "wasm32")]
            {
                format!(
                    "{view_label}\nWeb build keyboard mode\nT\n1=Acro 2=Angle 3=PosHold\nSpace (boost)\nWASD\nQ / E\nR"
                )
            }
        }
    };

    *text = Text::new(values);
}

fn rasterize_osd_canvas(
    osd_overlay: &SimOsdOverlay,
    raster_source: &OsdRasterSource,
    canvas_image: &mut Image,
) {
    if !raster_source.ready {
        return;
    }
    let Some(canvas_data) = canvas_image.data.as_mut() else {
        return;
    };

    let canvas_width = canvas_image.texture_descriptor.size.width as usize;
    let glyph_w = OSD_GLYPH_WIDTH_PX as usize;
    let glyph_h = OSD_GLYPH_HEIGHT_PX as usize;

    canvas_data.fill(0);

    for row in 0..OSD_ROWS {
        for col in 0..OSD_COLS {
            let idx = row * osd_overlay.cols + col;
            let glyph = osd_overlay
                .cells
                .get(idx)
                .copied()
                .unwrap_or(OSD_BLANK_SYMBOL) as usize;
            let rect = &raster_source.rects[glyph.min(OSD_FONT_ATLAS_GLYPHS.saturating_sub(1))];
            let src_x = rect.min.x as usize;
            let src_y = rect.min.y as usize;
            let dst_x = col * glyph_w;
            let dst_y = row * glyph_h;

            for y in 0..glyph_h {
                let src_offset = ((src_y + y) * raster_source.atlas_width + src_x)
                    * raster_source.bytes_per_pixel;
                let dst_offset =
                    ((dst_y + y) * canvas_width + dst_x) * raster_source.bytes_per_pixel;
                let line_len = glyph_w * raster_source.bytes_per_pixel;
                canvas_data[dst_offset..dst_offset + line_len]
                    .copy_from_slice(&raster_source.pixels[src_offset..src_offset + line_len]);
            }
        }
    }
}

fn prepare_osd_raster_source(
    osd_canvas_assets: Option<Res<OsdCanvasAssets>>,
    atlas_images: Res<Assets<Image>>,
    atlas_layouts: Res<Assets<TextureAtlasLayout>>,
    mut raster_source: ResMut<OsdRasterSource>,
) {
    if raster_source.ready {
        return;
    }
    let Some(osd_canvas_assets) = osd_canvas_assets else {
        return;
    };
    let Some(atlas_image) = atlas_images.get(&osd_canvas_assets.atlas) else {
        return;
    };
    let Some(atlas_layout) = atlas_layouts.get(&osd_canvas_assets.atlas_layout) else {
        return;
    };
    let Some(atlas_pixels) = atlas_image.data.as_ref() else {
        return;
    };
    let Ok(bytes_per_pixel) = atlas_image.texture_descriptor.format.pixel_size() else {
        return;
    };

    raster_source.atlas_width = atlas_image.texture_descriptor.size.width as usize;
    raster_source.bytes_per_pixel = bytes_per_pixel;
    raster_source.rects = atlas_layout.textures.clone();
    raster_source.pixels = atlas_pixels.clone();
    raster_source.ready = true;
}

fn display_viewport_rect(
    camera: &Camera,
    render_target: &RenderTarget,
    root_size: Option<Vec2>,
) -> Option<bevy::math::Rect> {
    match render_target {
        RenderTarget::Image(_) => root_size.map(|size| bevy::math::Rect {
            min: Vec2::ZERO,
            max: size,
        }),
        _ => camera.logical_viewport_rect(),
    }
}

fn logical_node_size(node: &ComputedNode) -> Vec2 {
    node.size() * node.inverse_scale_factor()
}

fn update_osd_overlay(
    (camera_view, osd_overlay): (Res<CameraView>, Res<SimOsdOverlay>),
    osd_canvas_assets: Option<Res<OsdCanvasAssets>>,
    raster_source: Res<OsdRasterSource>,
    mut images: ResMut<Assets<Image>>,
    scene_camera: Query<(&Camera, &RenderTarget), With<SimSceneCamera>>,
    mut root_query: Query<(&ComputedNode, &mut Visibility), With<OsdOverlayRoot>>,
    mut canvas_query: Query<&mut Node, With<OsdCanvasFrame>>,
) {
    let Ok((camera, render_target)) = scene_camera.single() else {
        return;
    };
    let Ok((root_node, mut visibility)) = root_query.single_mut() else {
        return;
    };
    let Some(viewport_rect) =
        display_viewport_rect(camera, render_target, Some(logical_node_size(root_node)))
    else {
        return;
    };
    let Ok(mut canvas_node) = canvas_query.single_mut() else {
        return;
    };

    *visibility = if *camera_view == CameraView::FirstPerson {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };

    let viewport_width = viewport_rect.width();
    let viewport_height = viewport_rect.height();
    let osd_width = OSD_CANVAS_WIDTH_PX as f32;
    let osd_height = OSD_CANVAS_HEIGHT_PX as f32;
    let width_fit_scale = viewport_width / osd_width;
    let width_fit_height = osd_height * width_fit_scale;

    let (target_width, target_height, target_left, target_top) =
        if width_fit_height <= viewport_height {
            (
                viewport_width,
                width_fit_height,
                viewport_rect.min.x,
                viewport_rect.min.y + ((viewport_height - width_fit_height) * 0.5),
            )
        } else {
            let height_fit_scale = viewport_height / osd_height;
            let height_fit_width = osd_width * height_fit_scale;
            (
                height_fit_width,
                viewport_height,
                viewport_rect.min.x + ((viewport_width - height_fit_width) * 0.5),
                viewport_rect.min.y,
            )
        };

    canvas_node.left = Val::Px(target_left.max(viewport_rect.min.x));
    canvas_node.top = Val::Px(target_top.max(viewport_rect.min.y));
    canvas_node.width = Val::Px(target_width.min(viewport_width).ceil());
    canvas_node.height = Val::Px(target_height.min(viewport_height).ceil());

    if !osd_overlay.is_changed() {
        return;
    }

    let Some(osd_canvas_assets) = osd_canvas_assets else {
        return;
    };
    let Some(canvas_image) = images.get_mut(&osd_canvas_assets.canvas) else {
        return;
    };

    rasterize_osd_canvas(&osd_overlay, &raster_source, canvas_image);
}

fn stop_copper_on_exit<T: Send + Sync + 'static>(
    mut exit_events: MessageReader<AppExit>,
    mut copper: ResMut<CopperState<T>>,
) {
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

fn asset_plugin() -> AssetPlugin {
    #[cfg(all(feature = "bevymon", target_arch = "wasm32"))]
    {
        AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            unapproved_path_mode: UnapprovedPathMode::Allow,
            ..default()
        }
    }
    #[cfg(not(all(feature = "bevymon", target_arch = "wasm32")))]
    {
        AssetPlugin {
            unapproved_path_mode: UnapprovedPathMode::Allow,
            ..default()
        }
    }
}

fn primary_window(split_monitor: bool) -> Window {
    let title = if split_monitor {
        "Copper Flight Controller BevyMon"
    } else {
        "Copper Flight Controller Sim"
    };

    #[cfg(target_arch = "wasm32")]
    {
        return Window {
            title: title.into(),
            resolution: (1680, 960).into(),
            canvas: Some("#bevy".into()),
            fit_canvas_to_parent: true,
            ..default()
        };
    }

    #[cfg(not(target_arch = "wasm32"))]
    {
        Window {
            title: title.into(),
            resolution: (1680, 960).into(),
            ..default()
        }
    }
}

#[cfg(feature = "bevymon")]
fn setup_split_ui_camera(mut commands: Commands) {
    let camera = commands
        .spawn((
            bevy::prelude::Camera2d,
            Camera {
                order: 1,
                clear_color: ClearColorConfig::None,
                ..default()
            },
        ))
        .id();
    commands.insert_resource(SplitUiCamera(camera));
}

#[cfg(feature = "bevymon")]
fn spawn_bevymon_layout(
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
                sim_panel_percent: 67.0,
                monitor_panel_percent: 33.0,
                monitor_panel_inset_px: 4.0,
                ..default()
            }),
    );

    let hud_root = commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(0.0),
                left: Val::Px(0.0),
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                ..default()
            },
            Pickable::IGNORE,
        ))
        .id();
    commands.entity(layout.sim_panel).add_child(hud_root);
    commands.insert_resource(SimHudRoot(hud_root));
    spawned.0 = true;
}

fn sync_loading_overlay(
    load_state: Res<SceneLoadState>,
    mut overlays: Query<&mut Visibility, With<SceneLoadingOverlay>>,
) {
    if !load_state.ready {
        return;
    }

    for mut visibility in &mut overlays {
        *visibility = Visibility::Hidden;
    }
}

pub fn build_world(headless: bool, split_monitor: bool) -> App {
    let mut app = App::new();
    app.insert_resource(SimState::default())
        .insert_resource(SimMotorCommands::default())
        .insert_resource(SimRcInput::default())
        .insert_resource(SimKinematics::default())
        .insert_resource(RcInputSource::default())
        .insert_resource(SimJoystickState::default())
        .insert_resource(CameraView::default())
        .insert_resource(SimOsdOverlay::default())
        .init_resource::<OsdRasterSource>()
        .insert_resource(WorldLayout { split_monitor })
        .init_resource::<SceneLoadState>()
        .init_resource::<SimHudSpawnState>();

    if headless {
        app.add_plugins(MinimalPlugins);
        return app;
    }

    app.add_plugins(
        DefaultPlugins
            .set(WindowPlugin {
                primary_window: Some(primary_window(split_monitor)),
                ..default()
            })
            .set(asset_plugin()),
    )
    .add_plugins(PhysicsPlugins::default())
    .insert_resource(Gravity(Vec3::new(0.0, -9.81, 0.0)))
    .insert_resource(Time::<Physics>::default())
    .add_systems(
        Startup,
        (setup_world, setup_full_window_hud_root, setup_joystick),
    )
    .add_systems(
        Update,
        (
            spawn_loading_overlay,
            spawn_help_overlay,
            spawn_osd_overlay,
            sync_loading_overlay,
        ),
    )
    .add_systems(
        Update,
        (
            spawn_quadcopter_when_world_ready,
            poll_joystick,
            update_rc_input_keyboard,
            adjust_keyboard_throttle,
            reset_vehicle,
        ),
    )
    .add_systems(
        Update,
        (
            toggle_camera_view,
            update_quadcopter_visibility,
            camera_follow_quadcopter,
            track_sim_led_state,
            update_help_overlay,
            prepare_osd_raster_source,
            update_osd_overlay,
        )
            .chain(),
    )
    .add_systems(FixedUpdate, sync_vehicle_state);

    register_scene_reflect_types(&mut app);

    app
}

#[cfg(feature = "sim")]
pub fn run_sim() {
    let mut app = build_world(false, false);
    #[cfg(not(target_arch = "wasm32"))]
    {
        app.add_systems(Startup, setup_copper);
        app.add_systems(
            FixedUpdate,
            (run_copper::<CopperContext>, apply_multicopter_dynamics)
                .chain()
                .after(sync_vehicle_state),
        );
        app.add_systems(PostUpdate, stop_copper_on_exit::<CopperContext>);
    }
    app.run();
}

#[cfg(feature = "bevymon")]
pub fn run_bevymon() {
    let (monitor_model, copper) = build_bevymon_copper();

    let mut app = build_world(false, true);
    app.insert_resource(copper)
        .init_resource::<LayoutSpawned>()
        .add_plugins(
            CuBevyMonPlugin::new(monitor_model)
                .with_initial_focus(CuBevyMonSurface::Sim)
                .with_options(MonitorUiOptions {
                    show_quit_hint: false,
                }),
        )
        .add_systems(Startup, setup_split_ui_camera)
        .add_systems(Update, spawn_bevymon_layout)
        .add_systems(
            FixedUpdate,
            (run_copper::<LoggerRuntime>, apply_multicopter_dynamics)
                .chain()
                .after(sync_vehicle_state),
        )
        .add_systems(PostUpdate, stop_copper_on_exit::<LoggerRuntime>);
    app.run();
}

#[cfg(feature = "sim")]
fn main() {
    run_sim();
}

#[cfg(test)]
mod tests {
    use super::*;

    fn heading_from_mag_xy_deg(mag: [f32; 3]) -> f32 {
        let mut heading = libm::atan2f(mag[1], mag[0]).to_degrees();
        if heading < 0.0 {
            heading += 360.0;
        }
        heading
    }

    fn assert_heading_close(actual: f32, expected: f32) {
        let err = (actual - expected + 540.0).rem_euclid(360.0) - 180.0;
        assert!(
            err.abs() < 1.0e-3,
            "heading mismatch: actual={actual} expected={expected} err={err}"
        );
    }

    #[test]
    fn sim_world_starts() {
        let mut app = build_world(true, false);
        app.update();
    }

    #[test]
    fn sim_world_magnetic_field_is_three_dimensional() {
        let world_mag = Vec3::from_array(WORLD_MAG_FIELD_UT);
        assert!(world_mag.x.abs() > 0.0 || world_mag.z.abs() > 0.0);
        assert!(world_mag.y.abs() > 0.0);
    }

    #[test]
    fn sim_magnetometer_heading_tracks_bevy_yaw_convention() {
        let world_mag = Vec3::from_array(WORLD_MAG_FIELD_UT);

        // In Bevy body axes, identity faces world +Z (south), so heading is 180 deg.
        let south_body = Quat::IDENTITY.inverse() * world_mag;
        let south_fc = map_bevy_body_to_fc_magnetometer(south_body);
        assert!(south_fc[2] > 0.0, "expected positive down component");
        assert_heading_close(heading_from_mag_xy_deg(south_fc), 180.0);

        // Positive Bevy yaw rotates counter-clockwise; compass heading decreases clockwise.
        let yaw_90_body = Quat::from_rotation_y(90.0_f32.to_radians()).inverse() * world_mag;
        let yaw_90_fc = map_bevy_body_to_fc_magnetometer(yaw_90_body);
        assert_heading_close(heading_from_mag_xy_deg(yaw_90_fc), 90.0);

        let yaw_180_body = Quat::from_rotation_y(180.0_f32.to_radians()).inverse() * world_mag;
        let yaw_180_fc = map_bevy_body_to_fc_magnetometer(yaw_180_body);
        assert_heading_close(heading_from_mag_xy_deg(yaw_180_fc), 0.0);
    }

    #[test]
    fn sim_gyro_z_maps_right_turn_to_negative_fc_yaw_rate() {
        // The corresponding body yaw rate in Bevy for a right turn is negative Y.
        let gyro_fc = map_bevy_body_to_fc_axial(Vec3::new(0.0, -1.0, 0.0));
        assert!(
            gyro_fc[2] < 0.0,
            "right turn should map to negative FC gyro_z, got {}",
            gyro_fc[2]
        );
    }
}
