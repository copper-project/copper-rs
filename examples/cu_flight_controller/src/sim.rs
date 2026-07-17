#[cfg(feature = "sim")]
extern crate alloc;

#[cfg(all(feature = "forest-world", feature = "urban-world"))]
compile_error!("`forest-world` and `urban-world` are mutually exclusive");

#[cfg(not(any(feature = "forest-world", feature = "urban-world")))]
compile_error!("select exactly one simulator world feature: `forest-world` or `urban-world`");

#[cfg(feature = "sim")]
mod autonomy_bridge;
mod compute_tasks;
#[cfg(feature = "sim")]
mod messages;
#[cfg(feature = "sim")]
#[path = "sim/rc_joystick.rs"]
mod rc_joystick;
#[cfg(feature = "sim")]
mod sim_support;
#[cfg(any(feature = "sim", feature = "bevymon"))]
mod sim_zed;
#[cfg(feature = "sim")]
mod tasks;
#[cfg(not(target_arch = "wasm32"))]
mod windowing;

use avian3d::prelude::*;
use bevy::app::AppExit;
#[cfg(all(feature = "bevymon", target_arch = "wasm32"))]
use bevy::asset::AssetMetaCheck;
use bevy::asset::RenderAssetUsages;
use bevy::asset::UnapprovedPathMode;
#[cfg(feature = "bevymon")]
use bevy::camera::ClearColorConfig;
use bevy::camera::RenderTarget;
use bevy::core_pipeline::{Core3dSystems, Skybox, prepass::DepthPrepass, schedule::Core3d};
use bevy::ecs::change_detection::DetectChanges;
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::ecs::system::SystemParam;
use bevy::image::{
    ImageCompareFunction, ImageSampler, ImageSamplerDescriptor, TextureFormatPixelInfo,
};
#[cfg(feature = "forest-world")]
use bevy::pbr::{DistanceFog, FogFalloff};
#[cfg(all(feature = "sim", test))]
use bevy::prelude::EulerRot;
use bevy::prelude::{
    App, AssetPlugin, AssetServer, Assets, ButtonInput, Camera, Camera3d, Color, Commands,
    Component, ComputedNode, DefaultPlugins, Dir3, DirectionalLight, Entity, EnvironmentMapLight,
    FixedUpdate, FontSize, GlobalAmbientLight, GlobalTransform, GltfAssetLabel, Handle, Image,
    ImageNode, IsDefaultUiCamera, KeyCode, MessageReader, MessageWriter, MinimalPlugins, Msaa,
    Name, Node, On, PerspectiveProjection, Pickable, PluginGroup, PositionType, PostUpdate,
    Projection, Quat, Query, Res, ResMut, Resource, Startup, Text, TextColor, TextFont,
    TextureAtlasLayout, Time, Transform, UVec2, UiRect, Update, Val, Vec2, Vec3, Visibility,
    Window, WindowPlugin, With, Without, WorldAsset, WorldAssetRoot, default,
};
use bevy::render::RenderApp;
use bevy::render::camera::ExtractedCamera;
use bevy::render::extract_resource::{ExtractResource, ExtractResourcePlugin};
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::render_asset::RenderAssets;
use bevy::render::render_resource::{
    Extent3d, Origin3d, TexelCopyTextureInfo, TextureAspect, TextureDimension, TextureFormat,
    TextureUsages,
};
use bevy::render::renderer::{RenderContext, ViewQuery};
use bevy::render::texture::GpuImage;
use bevy::render::view::ViewDepthTexture;
#[cfg(not(target_arch = "wasm32"))]
use cached_path::{Cache, Error as CacheError, ProgressBar};
#[cfg(feature = "bevymon")]
use cu_bevymon::{
    CuBevyMonFocus, CuBevyMonPlugin, CuBevyMonSplitLayoutConfig, CuBevyMonSplitStyle,
    CuBevyMonSurface, CuBevyMonTexture, MonitorModel, MonitorUiOptions, spawn_split_layout,
};
use cu29::prelude::*;
#[cfg(all(feature = "sim", test))]
use cu29::units::si::angle::radian;
#[cfg(all(feature = "sim", test))]
use cu29::units::si::f32::Angle;
#[cfg(feature = "sim")]
use cu29::units::si::velocity::meter_per_second;

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
use std::io::{self, Write};
use std::mem::size_of;
#[cfg(not(target_arch = "wasm32"))]
use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;
#[cfg(feature = "bevymon")]
use std::sync::{Arc, Mutex};

#[cfg(feature = "sim")]
fn replace_reflected_task<A, T>(app: &mut A, task_id: &str, replacement: T) -> CuResult<()>
where
    A: cu29::reflect::ReflectTaskIntrospection,
    T: cu29::reflect::Reflect,
{
    let task = cu29::reflect::ReflectTaskIntrospection::reflect_task_mut(app, task_id)
        .and_then(|task| task.downcast_mut::<T>())
        .ok_or_else(|| CuError::from(format!("sim reset cannot access task {task_id}")))?;
    *task = replacement;
    Ok(())
}

mod mcu_copper {
    use super::*;

    pub mod tasks {
        pub use crate::tasks::*;
    }

    #[copper_runtime(
        config = "flight_controller.ron",
        subsystem = "mcu",
        sim_mode = true,
        ignore_resources = true
    )]
    struct FlightControllerSim {}

    pub(super) struct Runtime {
        app: default::FlightControllerSim,
    }

    impl Runtime {
        #[cfg(all(not(target_arch = "wasm32"), feature = "sim"))]
        pub(super) fn with_log_path(
            clock: &RobotClock,
            logger_path: &Path,
            log_slab_size: Option<usize>,
        ) -> Self {
            let app = default::FlightControllerSim::builder()
                .with_clock(clock.clone())
                .with_log_path(PathBuf::from(logger_path), log_slab_size)
                .expect("failed to create logger")
                .with_sim_callback(&mut default_callback)
                .build()
                .expect("failed to create runtime");
            Self { app }
        }

        #[cfg(test)]
        pub(super) fn without_logger(clock: &RobotClock) -> Self {
            let app = default::FlightControllerSim::builder()
                .with_clock(clock.clone())
                .with_sim_callback(&mut default_callback)
                .build()
                .expect("failed to create runtime");
            Self { app }
        }

        #[cfg(feature = "bevymon")]
        pub(super) fn with_bevymon_logger(
            clock: &RobotClock,
            logger: Arc<Mutex<BevyMonUnifiedLogger>>,
        ) -> Self {
            let app = default::FlightControllerSim::builder()
                .with_clock(clock.clone())
                .with_logger::<BevyMonSectionStorage, BevyMonUnifiedLogger>(logger)
                .with_sim_callback(&mut default_callback)
                .build()
                .expect("failed to create runtime");
            Self { app }
        }

        pub(super) fn start(&mut self) -> CuResult<()> {
            self.app.start_all_tasks(&mut default_callback)
        }

        pub(super) fn stop(&mut self) -> CuResult<()> {
            self.app.stop_all_tasks(&mut default_callback)
        }

        pub(super) fn log_shutdown_completed(&mut self) -> CuResult<()> {
            self.app.log_shutdown_completed()
        }

        #[cfg(feature = "sim")]
        pub(super) fn reset_sim_state(&mut self) -> CuResult<()> {
            replace_reflected_task(&mut self.app, "ahrs", cu_ahrs::CuAhrs::new_filter())?;
            replace_reflected_task(
                &mut self.app,
                "navigation",
                crate::tasks::autonomy::NavigationStateTask::default(),
            )?;
            replace_reflected_task(
                &mut self.app,
                "auto_mission",
                crate::tasks::autonomy::AutoMission::default(),
            )?;
            replace_reflected_task(
                &mut self.app,
                "autonomy_context",
                crate::tasks::autonomy::AutonomyContextTask::default(),
            )?;
            replace_reflected_task(
                &mut self.app,
                "auto_controller",
                crate::tasks::autonomy::AutoVelocityController::default(),
            )
        }

        #[cfg(feature = "bevymon")]
        pub(super) fn monitor_model(&mut self) -> MonitorModel {
            self.app.copper_runtime_mut().monitor.model()
        }

        pub(super) fn run_iteration(
            &mut self,
            clock: &RobotClock,
            vehicle: SimVehicleState,
            rc: SimRcInput,
            motor_commands: &mut SimMotorCommands,
            osd_overlay: &mut SimOsdOverlay,
            _autonomy_link: &mut SimAutonomyLink,
        ) -> CuResult<()> {
            let clock = clock.clone();
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
                        output.set_payload(MagnetometerPayload::from_raw(
                            map_bevy_body_to_fc_magnetometer(body_mag),
                        ));
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
                        channels[6] = if rc.auto { 992 } else { 172 };

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
                    default::SimStep::VtxMspTxRequests { msg, .. } => {
                        if let Some(batch) = msg.payload() {
                            osd_overlay.apply_batch(batch);
                        }
                        SimOverride::ExecuteByRuntime
                    }
                    #[cfg(feature = "sim")]
                    default::SimStep::AutonomyTxContext { msg, .. } => {
                        _autonomy_link.context = msg.clone();
                        SimOverride::ExecutedBySim
                    }
                    #[cfg(feature = "sim")]
                    default::SimStep::AutonomyRxCommand { msg, .. } => {
                        *msg = core::mem::take(&mut _autonomy_link.command);
                        SimOverride::ExecutedBySim
                    }
                    _ => SimOverride::ExecuteByRuntime,
                }
            };

            self.app.run_one_iteration(&mut sim_callback)
        }
    }

    fn default_callback(_step: default::SimStep) -> SimOverride {
        SimOverride::ExecuteByRuntime
    }

    #[cfg(all(test, not(target_arch = "wasm32"), feature = "sim"))]
    pub(super) fn register_distributed_replay(
        builder: cu29::distributed_replay::DistributedReplayBuilder,
    ) -> CuResult<cu29::distributed_replay::DistributedReplayBuilder> {
        builder.register::<FlightControllerSim>("mcu")
    }
}

mod compute_copper {
    use super::*;

    pub mod tasks {
        pub use crate::compute_tasks::*;
    }

    #[copper_runtime(
        config = "flight_controller.ron",
        subsystem = "compute",
        sim_mode = true
    )]
    struct FlightComputeSim {}

    pub(super) struct Runtime {
        app: default::FlightComputeSim,
        zed_store: sim_zed::SimZedFrameStore,
        zed_static_state_sent: bool,
    }

    #[cfg(test)]
    pub(super) type RecordedDataSet = default::CuStampedDataSet;

    impl Runtime {
        #[cfg(all(not(target_arch = "wasm32"), feature = "sim"))]
        pub(super) fn with_log_path(
            clock: &RobotClock,
            zed_store: sim_zed::SimZedFrameStore,
            logger_path: &Path,
            log_slab_size: Option<usize>,
        ) -> Self {
            let app = default::FlightComputeSim::builder()
                .with_clock(clock.clone())
                .with_log_path(PathBuf::from(logger_path), log_slab_size)
                .expect("failed to create compute logger")
                .with_sim_callback(&mut default_callback)
                .build()
                .expect("failed to create compute runtime");
            Self {
                app,
                zed_store,
                zed_static_state_sent: false,
            }
        }

        #[cfg(any(test, feature = "bevymon"))]
        pub(super) fn new(clock: &RobotClock, zed_store: sim_zed::SimZedFrameStore) -> Self {
            let app = default::FlightComputeSim::builder()
                .with_clock(clock.clone())
                .with_sim_callback(&mut default_callback)
                .build()
                .expect("failed to create compute runtime");
            Self {
                app,
                zed_store,
                zed_static_state_sent: false,
            }
        }

        pub(super) fn start(&mut self) -> CuResult<()> {
            self.app.start_all_tasks(&mut default_callback)
        }

        pub(super) fn run_iteration(
            &mut self,
            clock: &RobotClock,
            _autonomy_link: &mut SimAutonomyLink,
        ) -> CuResult<()> {
            let zed_store = &self.zed_store;
            let zed_static_state_sent = &mut self.zed_static_state_sent;
            let mut sim_callback = |step: default::SimStep| -> SimOverride {
                match step {
                    default::SimStep::Zed(CuTaskCallbackState::Process(_, output)) => {
                        sim_zed::write_source_outputs(
                            clock,
                            zed_store,
                            zed_static_state_sent,
                            output,
                        );
                        SimOverride::ExecutedBySim
                    }
                    #[cfg(not(feature = "sim"))]
                    default::SimStep::Vitfly(CuTaskCallbackState::Process(input, _)) => {
                        zed_store.publish_vitfly_depth(input.1.payload());
                        SimOverride::ExecuteByRuntime
                    }
                    #[cfg(feature = "sim")]
                    default::SimStep::Vitfly(CuTaskCallbackState::Process(input, _)) => {
                        zed_store.publish_vitfly_depth(input.0.payload());
                        SimOverride::ExecuteByRuntime
                    }
                    #[cfg(feature = "sim")]
                    default::SimStep::VitflyCommand(CuTaskCallbackState::Process(input, _)) => {
                        if let Some(velocity) = input.1.payload() {
                            zed_store.publish_vitfly_prediction(
                                velocity.map(|axis| axis.get::<meter_per_second>()),
                            );
                        }
                        SimOverride::ExecuteByRuntime
                    }
                    #[cfg(feature = "sim")]
                    default::SimStep::AutonomyRxContext { msg, .. } => {
                        *msg = core::mem::take(&mut _autonomy_link.context);
                        SimOverride::ExecutedBySim
                    }
                    #[cfg(feature = "sim")]
                    default::SimStep::AutonomyTxCommand { msg, .. } => {
                        _autonomy_link.command = msg.clone();
                        if let Some(command) = msg.payload() {
                            // The upstream ViTFly preview draws the normalized
                            // velocity command, not the raw network vector.
                            zed_store.publish_vitfly_prediction([
                                command.north.get::<meter_per_second>(),
                                command.west.get::<meter_per_second>(),
                                command.up.get::<meter_per_second>(),
                            ]);
                        }
                        SimOverride::ExecutedBySim
                    }
                    _ => SimOverride::ExecuteByRuntime,
                }
            };
            self.app.run_one_iteration(&mut sim_callback)
        }

        pub(super) fn set_zed_sensors(&self, sensors: sim_zed::SimZedSensors) {
            self.zed_store.set_sensors(sensors);
        }

        pub(super) fn stop(&mut self) -> CuResult<()> {
            self.app.stop_all_tasks(&mut default_callback)
        }

        pub(super) fn log_shutdown_completed(&mut self) -> CuResult<()> {
            self.app.log_shutdown_completed()
        }

        #[cfg(feature = "sim")]
        pub(super) fn reset_sim_state(&mut self) -> CuResult<()> {
            replace_reflected_task(
                &mut self.app,
                "vitfly_context",
                crate::compute_tasks::VitFlyContextAdapter::default(),
            )?;

            let vitfly =
                cu29::reflect::ReflectTaskIntrospection::reflect_task_mut(&mut self.app, "vitfly")
                    .and_then(|task| task.downcast_mut::<cu_vitfly::VitFlyTask>())
                    .ok_or_else(|| CuError::from("sim reset cannot access task vitfly"))?;
            let config = cu29::bincode::config::standard();
            let encoded =
                cu29::bincode::encode_to_vec(Option::<(Vec<f32>, Vec<f32>)>::None, config)
                    .map_err(|err| {
                        CuError::new_with_cause(
                            "failed to encode empty ViTFly recurrent state",
                            err,
                        )
                    })?;
            let reader = cu29::bincode::de::read::SliceReader::new(&encoded);
            let mut decoder = cu29::bincode::de::DecoderImpl::new(reader, config, ());
            vitfly.thaw(&mut decoder).map_err(|err| {
                CuError::new_with_cause("failed to clear ViTFly recurrent state", err)
            })?;
            self.zed_static_state_sent = false;
            Ok(())
        }
    }

    fn default_callback(_step: default::SimStep) -> SimOverride {
        SimOverride::ExecuteByRuntime
    }

    #[cfg(all(test, not(target_arch = "wasm32"), feature = "sim"))]
    pub(super) fn register_distributed_replay(
        builder: cu29::distributed_replay::DistributedReplayBuilder,
    ) -> CuResult<cu29::distributed_replay::DistributedReplayBuilder> {
        builder.register::<FlightComputeSim>("compute")
    }
}

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
            position: SIM_WORLD.spawn_position,
            velocity_world: Vec3::ZERO,
            rotation: spawn_rotation(),
            body_accel_fc: [0.0, 0.0, 9.81],
            body_gyro_fc: [0.0; 3],
        }
    }
}

#[derive(Resource)]
struct CopperState {
    clock: RobotClock,
    clock_mock: RobotClockMock,
    mcu: mcu_copper::Runtime,
    compute: compute_copper::Runtime,
    autonomy_link: SimAutonomyLink,
}

#[derive(Default)]
#[allow(dead_code)]
struct SimAutonomyLink {
    context: CuMsg<messages::AutonomyContext>,
    command: CuMsg<messages::AutonomyVelocityCommand>,
}

#[derive(Clone, Resource)]
struct SceneAssetPaths {
    quadcopter: String,
    world: String,
    skybox: String,
    specular_map: String,
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
    auto: bool,
}

impl Default for SimRcInput {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.0,
            armed: false,
            mode: messages::FlightMode::Acro,
            auto: false,
        }
    }
}

#[derive(Resource, Default)]
struct SimKinematics {
    prev_linear_velocity: Option<Vec3>,
}

#[derive(Resource)]
struct PendingQuadcopterSpawn {
    quadcopter_scene: Handle<WorldAsset>,
    world_scene: Handle<WorldAsset>,
}

#[derive(Resource, Clone, Copy, Debug, Default, PartialEq, Eq)]
enum RcInputSource {
    #[default]
    Keyboard,
    Joystick,
}

#[derive(Resource, Default)]
struct SimResetInterlock {
    waiting_for_joystick_safe: bool,
}

impl SimResetInterlock {
    fn begin(&mut self, source: RcInputSource, rc_input: &mut SimRcInput) {
        match source {
            RcInputSource::Keyboard => {
                self.waiting_for_joystick_safe = false;
                init_keyboard_rc(rc_input);
            }
            RcInputSource::Joystick => {
                self.waiting_for_joystick_safe = true;
                force_disarmed(rc_input);
            }
        }
    }

    fn apply_joystick_frame(&mut self, rc_input: &mut SimRcInput) {
        if !self.waiting_for_joystick_safe {
            return;
        }

        if !rc_input.armed && !rc_input.auto {
            self.waiting_for_joystick_safe = false;
            return;
        }

        force_disarmed(rc_input);
    }
}

fn force_disarmed(rc_input: &mut SimRcInput) {
    rc_input.roll = 0.0;
    rc_input.pitch = 0.0;
    rc_input.yaw = 0.0;
    rc_input.throttle = 0.0;
    rc_input.armed = false;
    rc_input.auto = false;
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
    zed: bool,
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
struct ZedDepthCamera;

#[derive(Component)]
struct ZedRightCamera;

type ZedCameraFilter = (
    Without<Multicopter>,
    bevy::ecs::query::Or<(With<ZedDepthCamera>, With<ZedRightCamera>)>,
);

#[derive(Clone, Resource)]
struct ZedDepthTexture(Handle<Image>);

impl ExtractResource for ZedDepthTexture {
    type Source = Self;

    fn extract_resource(source: &Self::Source) -> Self {
        source.clone()
    }
}

#[derive(Resource)]
struct ZedDepthPreview {
    image: Handle<Image>,
    last_seq: Option<u64>,
    last_prediction_seq: u64,
}

#[cfg(feature = "vitfly-cuda")]
const ZED_DEPTH_PREVIEW_TITLE: &str = "depth & prediction (GPU)";
#[cfg(all(feature = "sim", not(feature = "vitfly-cuda")))]
const ZED_DEPTH_PREVIEW_TITLE: &str = "depth & prediction (CPU)";
#[cfg(not(feature = "sim"))]
const ZED_DEPTH_PREVIEW_TITLE: &str = "depth";

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
const EARTH_METERS_PER_DEG_LAT: f64 = 111_320.0;
// The FC sensor convention maps Bevy body +Z to forward, so its northward
// magnetic component must be +Z even though the rendered vehicle faces -Z.
// This produces the expected NED field [north=20, east=0, down=45] at the
// identity spawn pose. Declination remains 0 here and is applied separately by
// `MagneticTrueHeading`.
const WORLD_MAG_FIELD_UT: [f32; 3] = [0.0, -45.0, 20.0];
#[cfg(not(target_arch = "wasm32"))]
const BASE_ASSETS_URL: &str = "https://cdn.copper-robotics.com/";
const SKYBOX: &str = "skybox.ktx2";
const SPECULAR_MAP: &str = "specular_map.ktx2";
const QUADCOPTER: &str = "quadcopter.glb";
#[cfg(not(target_arch = "wasm32"))]
const SCENE_ASSET_CACHE_DIR: &str = ".download-cache";

struct SimWorldConfig {
    asset_name: &'static str,
    entity_name: &'static str,
    bbox_min_units: Vec3,
    bbox_max_units: Vec3,
    scale: f32,
    spawn_position: Vec3,
    spawn_yaw_deg: f32,
}

#[cfg(feature = "forest-world")]
const SIM_WORLD: SimWorldConfig = SimWorldConfig {
    asset_name: "forest.glb",
    entity_name: "forest",
    bbox_min_units: Vec3::new(-115.0, -2.5065, -545.0),
    bbox_max_units: Vec3::new(115.0, 28.97935, 90.0),
    scale: 1.0,
    spawn_position: Vec3::new(-10.0, 0.12, 20.0),
    spawn_yaw_deg: 0.0,
};

#[cfg(feature = "urban-world")]
const SIM_WORLD: SimWorldConfig = SimWorldConfig {
    asset_name: "city-fixed.glb",
    entity_name: "city",
    bbox_min_units: Vec3::new(-30_614.165, -648.2196, -4_185.883),
    bbox_max_units: Vec3::new(18_754.953, 11_102.407, 35_871.875),
    scale: 0.01,
    spawn_position: Vec3::new(-10.0, 0.12, 20.0),
    spawn_yaw_deg: 180.0,
};

// Extend beyond the rendered frame and motor-arm centers so scene-geometry
// contacts happen before the model visibly penetrates the environment.
const SIM_QUADCOPTER_COLLIDER_SIZE: Vec3 = Vec3::new(0.18, 0.06, 0.18);
// With the corrected 0.44 kg sim mass and 10% airmode idle, hover is about 0.48.
// Keep keyboard idle slightly below hover so release-to-descend works again.
const KEYBOARD_HOVER_THROTTLE_LOW: f32 = 0.47;
const KEYBOARD_HOVER_THROTTLE_HIGH: f32 = 0.52;

fn spawn_rotation() -> Quat {
    Quat::from_rotation_y(SIM_WORLD.spawn_yaw_deg.to_radians())
}

fn init_keyboard_rc(rc_input: &mut SimRcInput) {
    rc_input.mode = messages::FlightMode::Acro;
    rc_input.auto = false;
    rc_input.armed = false;
    rc_input.throttle = 0.0;
}

fn spawn_pose_components() -> (
    Transform,
    Position,
    Rotation,
    LinearVelocity,
    AngularVelocity,
) {
    (
        Transform::from_translation(SIM_WORLD.spawn_position).with_rotation(spawn_rotation()),
        Position::from_xyz(
            SIM_WORLD.spawn_position.x,
            SIM_WORLD.spawn_position.y,
            SIM_WORLD.spawn_position.z,
        ),
        Rotation(spawn_rotation()),
        LinearVelocity(Vec3::ZERO),
        AngularVelocity(Vec3::ZERO),
    )
}

#[cfg(not(target_arch = "wasm32"))]
fn scene_asset_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets")
}

#[cfg(not(target_arch = "wasm32"))]
fn scene_asset_cache_root() -> PathBuf {
    scene_asset_root().join(SCENE_ASSET_CACHE_DIR)
}

#[cfg(not(target_arch = "wasm32"))]
fn path_string(path: &Path) -> String {
    path.to_string_lossy().into_owned()
}

#[cfg(not(target_arch = "wasm32"))]
fn asset_progress_bar(done: usize, total: usize) -> String {
    const WIDTH: usize = 28;
    let filled = WIDTH * done / total.max(1);
    let empty = WIDTH.saturating_sub(filled);
    format!(
        "[{}{}] {done}/{total}",
        "=".repeat(filled),
        " ".repeat(empty)
    )
}

#[cfg(not(target_arch = "wasm32"))]
fn link_or_copy_cached_asset(src: &Path, dst: &Path) -> io::Result<()> {
    if fs::symlink_metadata(dst).is_ok() {
        fs::remove_file(dst)?;
    }

    #[cfg(unix)]
    {
        match std::os::unix::fs::symlink(src, dst) {
            Ok(()) => Ok(()),
            Err(symlink_err) => fs::copy(src, dst).map(|_| ()).map_err(|copy_err| {
                io::Error::new(
                    copy_err.kind(),
                    format!("failed to symlink ({symlink_err}) or copy ({copy_err})"),
                )
            }),
        }
    }

    #[cfg(windows)]
    {
        match std::os::windows::fs::symlink_file(src, dst) {
            Ok(()) => Ok(()),
            Err(symlink_err) => fs::copy(src, dst).map(|_| ()).map_err(|copy_err| {
                io::Error::new(
                    copy_err.kind(),
                    format!("failed to symlink ({symlink_err}) or copy ({copy_err})"),
                )
            }),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn get_asset_path(
    online_cache: &Cache,
    offline_cache: &Cache,
    asset_url: &str,
    asset_name: &str,
) -> Result<PathBuf, CacheError> {
    match offline_cache.cached_path(asset_url) {
        Ok(path) => Ok(path),
        Err(err) => {
            if matches!(
                err,
                CacheError::NoCachedVersions(_) | CacheError::CacheCorrupted(_)
            ) {
                eprintln!("  {asset_name}: cache miss; downloading from {asset_url}");
                online_cache.cached_path(asset_url)
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
    asset_root: &Path,
    index: usize,
    total: usize,
    asset_name: &str,
) -> Result<String, CacheError> {
    let plain_path = asset_root.join(asset_name);
    if plain_path.is_file() {
        eprintln!(
            "  {} {asset_name}: cached",
            asset_progress_bar(index, total)
        );
        return Ok(path_string(&plain_path));
    }

    if fs::symlink_metadata(&plain_path).is_ok() {
        fs::remove_file(&plain_path)?;
    }

    eprintln!(
        "  {} {asset_name}: resolving",
        asset_progress_bar(index.saturating_sub(1), total)
    );
    let asset_url = format!("{BASE_ASSETS_URL}{asset_name}");
    let hashed_path = get_asset_path(online_cache, offline_cache, &asset_url, asset_name)?;
    link_or_copy_cached_asset(&hashed_path, &plain_path)?;
    eprintln!("  {} {asset_name}: ready", asset_progress_bar(index, total));
    Ok(path_string(&plain_path))
}

#[cfg(not(target_arch = "wasm32"))]
fn prepare_scene_assets() -> SceneAssetPaths {
    let asset_root = scene_asset_root();
    let cache_root = scene_asset_cache_root();
    fs::create_dir_all(&asset_root).expect("failed to create scene asset directory");

    eprintln!(
        "Preparing Copper flight-controller scene assets in {}",
        asset_root.display()
    );
    let _ = io::stderr().flush();

    let online_cache = Cache::builder()
        .dir(cache_root.clone())
        .progress_bar(Some(ProgressBar::Full))
        .build()
        .expect("failed to create online scene asset cache");
    let offline_cache = Cache::builder()
        .dir(cache_root)
        .offline(true)
        .progress_bar(None)
        .build()
        .expect("failed to create offline scene asset cache");

    let scene_assets = [QUADCOPTER, SIM_WORLD.asset_name, SKYBOX, SPECULAR_MAP];
    let total = scene_assets.len();
    let mut paths = Vec::with_capacity(total);
    for (index, asset_name) in scene_assets.iter().enumerate() {
        paths.push(
            precached_asset_path(
                &online_cache,
                &offline_cache,
                &asset_root,
                index + 1,
                total,
                asset_name,
            )
            .unwrap_or_else(|err| panic!("failed to prepare {asset_name}: {err}")),
        );
    }

    eprintln!("Scene assets ready.");
    SceneAssetPaths {
        quadcopter: paths[0].clone(),
        world: paths[1].clone(),
        skybox: paths[2].clone(),
        specular_map: paths[3].clone(),
    }
}

#[cfg(target_arch = "wasm32")]
fn prepare_scene_assets() -> SceneAssetPaths {
    SceneAssetPaths {
        quadcopter: QUADCOPTER.to_string(),
        world: SIM_WORLD.asset_name.to_string(),
        skybox: SKYBOX.to_string(),
        specular_map: SPECULAR_MAP.to_string(),
    }
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

fn set_msg_timing<T: CuMsgPayload>(clock: &RobotClock, msg: &mut CuMsg<T>) {
    let tov = clock.now();
    let perf = cu29::curuntime::perf_now(clock);
    msg.tov = Tov::Time(tov);
    msg.metadata.process_time.start = perf.into();
    msg.metadata.process_time.end = perf.into();
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

#[cfg(all(feature = "sim", test))]
fn vitfly_pose(rotation: Quat) -> cu_ahrs::AhrsPose {
    // Bevy body axes are right/up/forward; the FC convention is
    // forward/right/down. Axial angles therefore map as [-Z, -X, +Y].
    let (yaw_bevy, pitch_bevy, roll_bevy) = rotation.to_euler(EulerRot::YXZ);
    cu_ahrs::AhrsPose {
        roll: Angle::new::<radian>(-roll_bevy),
        pitch: Angle::new::<radian>(-pitch_bevy),
        yaw: Angle::new::<radian>(yaw_bevy),
    }
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
fn setup_copper(mut commands: Commands, zed_store: Res<sim_zed::SimZedFrameStore>) {
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(128 * 1024 * 1024);
    let (mcu_logger_path, compute_logger_path) = sim_log_paths();
    commands.insert_resource(build_sim_copper_state(
        &mcu_logger_path,
        &compute_logger_path,
        LOG_SLAB_SIZE,
        zed_store.clone(),
    ));
}

#[cfg(all(not(target_arch = "wasm32"), feature = "sim"))]
fn sim_log_paths() -> (PathBuf, PathBuf) {
    let log_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("logs");
    (
        log_dir.join("flight_controller_sim.copper"),
        log_dir.join("flight_compute_sim.copper"),
    )
}

#[cfg(all(not(target_arch = "wasm32"), feature = "sim"))]
fn build_sim_copper_state(
    mcu_logger_path: &Path,
    compute_logger_path: &Path,
    log_slab_size: Option<usize>,
    zed_store: sim_zed::SimZedFrameStore,
) -> CopperState {
    for logger_path in [mcu_logger_path, compute_logger_path] {
        if let Some(parent) = logger_path.parent()
            && !parent.exists()
        {
            fs::create_dir_all(parent).expect("failed to create logs directory");
        }
    }

    let (clock, clock_mock) = RobotClock::mock();
    let mut mcu = mcu_copper::Runtime::with_log_path(&clock, mcu_logger_path, log_slab_size);
    let mut compute = compute_copper::Runtime::with_log_path(
        &clock,
        zed_store,
        compute_logger_path,
        log_slab_size,
    );

    mcu.start().expect("failed to start tasks");
    compute.start().expect("failed to start compute tasks");

    CopperState {
        clock,
        clock_mock,
        mcu,
        compute,
        autonomy_link: SimAutonomyLink::default(),
    }
}

#[cfg(feature = "bevymon")]
fn build_bevymon_copper(zed_store: sim_zed::SimZedFrameStore) -> (MonitorModel, CopperState) {
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(128 * 1024 * 1024);

    let (clock, clock_mock) = RobotClock::mock();
    let unified_logger = build_unified_logger(LOG_SLAB_SIZE).expect("failed to create logger");

    let mut mcu = mcu_copper::Runtime::with_bevymon_logger(&clock, unified_logger);
    let mut compute = compute_copper::Runtime::new(&clock, zed_store);

    mcu.start().expect("failed to start tasks");
    compute.start().expect("failed to start compute tasks");

    let monitor_model = mcu.monitor_model();
    (
        monitor_model,
        CopperState {
            clock,
            clock_mock,
            mcu,
            compute,
            autonomy_link: SimAutonomyLink::default(),
        },
    )
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

const ZED_DEPTH_CAMERA_ORDER: isize = -2;
const ZED_RIGHT_CAMERA_ORDER: isize = -1;

fn zed_color_texture() -> Image {
    let mut image = Image::new_fill(
        Extent3d {
            width: sim_zed::ZED_SIM_WIDTH,
            height: sim_zed::ZED_SIM_HEIGHT,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &[0, 0, 0, 255],
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::all(),
    );
    image.texture_descriptor.usage =
        TextureUsages::RENDER_ATTACHMENT | TextureUsages::COPY_SRC | TextureUsages::TEXTURE_BINDING;
    image
}

fn zed_depth_texture() -> Image {
    let mut image = Image::new_uninit(
        Extent3d {
            width: sim_zed::ZED_SIM_WIDTH,
            height: sim_zed::ZED_SIM_HEIGHT,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        TextureFormat::Depth32Float,
        RenderAssetUsages::all(),
    );
    image.texture_descriptor.usage |=
        TextureUsages::COPY_DST | TextureUsages::COPY_SRC | TextureUsages::TEXTURE_BINDING;
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        label: Some("ZED2i simulated depth".to_owned()),
        compare: Some(ImageCompareFunction::Always),
        ..default()
    });
    image
}

fn zed_depth_preview() -> Image {
    Image::new_fill(
        Extent3d {
            width: sim_zed::ZED_SIM_WIDTH,
            height: sim_zed::ZED_SIM_HEIGHT,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &[0, 0, 0, 255],
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::all(),
    )
}

fn copy_zed_depth_texture(
    view: ViewQuery<(&ExtractedCamera, &ViewDepthTexture)>,
    depth_texture: Option<Res<ZedDepthTexture>>,
    image_assets: Res<RenderAssets<GpuImage>>,
    mut ctx: RenderContext,
) {
    let Some(depth_texture) = depth_texture else {
        return;
    };
    let (camera, view_depth) = view.into_inner();
    if camera.order != ZED_DEPTH_CAMERA_ORDER {
        return;
    }
    let Some(destination) = image_assets.get(depth_texture.0.id()) else {
        return;
    };

    ctx.command_encoder().copy_texture_to_texture(
        TexelCopyTextureInfo {
            texture: &view_depth.texture,
            mip_level: 0,
            origin: Origin3d::default(),
            aspect: TextureAspect::DepthOnly,
        },
        TexelCopyTextureInfo {
            texture: &destination.texture,
            mip_level: 0,
            origin: Origin3d::default(),
            aspect: TextureAspect::DepthOnly,
        },
        Extent3d {
            width: sim_zed::ZED_SIM_WIDTH,
            height: sim_zed::ZED_SIM_HEIGHT,
            depth_or_array_layers: 1,
        },
    );
}

fn capture_zed_depth(event: On<ReadbackComplete>, store: Res<sim_zed::SimZedFrameStore>) {
    let pixel_count = (sim_zed::ZED_SIM_WIDTH * sim_zed::ZED_SIM_HEIGHT) as usize;
    if event.data.len() != pixel_count * size_of::<f32>() {
        return;
    }

    let mut depth = Vec::with_capacity(pixel_count);
    let mut confidence = Vec::with_capacity(pixel_count);
    for bytes in event.data.chunks_exact(size_of::<f32>()) {
        let reverse_z = f32::from_ne_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let distance = zed_reverse_z_to_distance(reverse_z);
        let valid = distance.is_finite() && distance <= sim_zed::ZED_SIM_MAX_DEPTH_M;
        depth.push(if valid { distance } else { f32::NAN });
        confidence.push(if valid { 0.0 } else { 100.0 });
    }

    store.set_depth(depth, confidence);
}

fn zed_reverse_z_to_distance(reverse_z: f32) -> f32 {
    if reverse_z <= 0.0 {
        return f32::NAN;
    }
    let near = sim_zed::ZED_SIM_NEAR_M;
    let far = sim_zed::ZED_SIM_MAX_DEPTH_M;
    (near * far) / (near + reverse_z * (far - near))
}

fn capture_zed_left_image(event: On<ReadbackComplete>, store: Res<sim_zed::SimZedFrameStore>) {
    let expected_len = (sim_zed::ZED_SIM_WIDTH * sim_zed::ZED_SIM_HEIGHT * 4) as usize;
    if event.data.len() == expected_len {
        store.set_left_image(event.data.to_vec());
    }
}

fn capture_zed_right_image(event: On<ReadbackComplete>, store: Res<sim_zed::SimZedFrameStore>) {
    let expected_len = (sim_zed::ZED_SIM_WIDTH * sim_zed::ZED_SIM_HEIGHT * 4) as usize;
    if event.data.len() == expected_len {
        store.set_right_image(event.data.to_vec());
    }
}

fn update_zed_depth_preview(
    store: Res<sim_zed::SimZedFrameStore>,
    mut preview: ResMut<ZedDepthPreview>,
    mut images: ResMut<Assets<Image>>,
) {
    let Some(depth) = store.published_depth() else {
        return;
    };
    let (prediction_seq, prediction) = store.vitfly_prediction();
    if preview.last_seq == Some(depth.seq) && preview.last_prediction_seq == prediction_seq {
        return;
    }

    let mut preview_pixels = depth.buffer_handle.with_inner(|values| {
        let mut pixels = Vec::with_capacity(values.len() * 4);
        for &distance in values.iter() {
            pixels.extend_from_slice(&depth_preview_color(distance));
        }
        pixels
    });
    if let Some(velocity) = prediction {
        draw_vitfly_arrow(
            &mut preview_pixels,
            depth.format.width as usize,
            depth.format.height as usize,
            velocity,
        );
    }
    let Some(mut preview_image) = images.get_mut(&preview.image) else {
        return;
    };
    let Some(pixels) = preview_image.data.as_mut() else {
        return;
    };
    if pixels.len() != preview_pixels.len() {
        return;
    }
    pixels.copy_from_slice(&preview_pixels);
    preview.last_seq = Some(depth.seq);
    preview.last_prediction_seq = prediction_seq;
}

fn draw_vitfly_arrow(pixels: &mut [u8], width: usize, height: usize, velocity: [f32; 3]) {
    let [_north, west, up] = velocity;
    if !west.is_finite() || !up.is_finite() || width == 0 || height == 0 {
        return;
    }

    let center = (width as f32 * 0.5, height as f32 * 0.5);
    // Match the original Python preview. Its fixed world frame is aligned with
    // the simulator's initial camera: west points left and up points upward.
    let end = (
        (center.0 - west * width as f32 / 3.0).clamp(0.0, (width - 1) as f32),
        (center.1 - up * height as f32 / 3.0).clamp(0.0, (height - 1) as f32),
    );
    draw_preview_line(pixels, width, height, center, end, 2);

    let direction = Vec2::new(center.0 - end.0, center.1 - end.1);
    if direction.length_squared() < 1.0 {
        return;
    }
    let direction = direction.normalize();
    let perpendicular = Vec2::new(-direction.y, direction.x);
    let tip = Vec2::new(end.0, end.1);
    let base = tip + direction * 12.0;
    draw_preview_line(
        pixels,
        width,
        height,
        (tip.x, tip.y),
        (
            base.x + perpendicular.x * 7.0,
            base.y + perpendicular.y * 7.0,
        ),
        2,
    );
    draw_preview_line(
        pixels,
        width,
        height,
        (tip.x, tip.y),
        (
            base.x - perpendicular.x * 7.0,
            base.y - perpendicular.y * 7.0,
        ),
        2,
    );
}

fn draw_preview_line(
    pixels: &mut [u8],
    width: usize,
    height: usize,
    start: (f32, f32),
    end: (f32, f32),
    radius: i32,
) {
    let delta_x = end.0 - start.0;
    let delta_y = end.1 - start.1;
    let steps = delta_x.abs().max(delta_y.abs()).ceil().max(1.0) as usize;
    for step in 0..=steps {
        let t = step as f32 / steps as f32;
        let x = (start.0 + delta_x * t).round() as i32;
        let y = (start.1 + delta_y * t).round() as i32;
        for offset_y in -radius..=radius {
            for offset_x in -radius..=radius {
                if offset_x * offset_x + offset_y * offset_y > radius * radius {
                    continue;
                }
                let pixel_x = x + offset_x;
                let pixel_y = y + offset_y;
                if pixel_x < 0 || pixel_y < 0 || pixel_x >= width as i32 || pixel_y >= height as i32
                {
                    continue;
                }
                let index = (pixel_y as usize * width + pixel_x as usize) * 4;
                if let Some(pixel) = pixels.get_mut(index..index + 4) {
                    pixel.copy_from_slice(&[255, 32, 32, 255]);
                }
            }
        }
    }
}

fn depth_preview_color(distance: f32) -> [u8; 4] {
    if !distance.is_finite() || distance > sim_zed::ZED_SIM_MAX_DEPTH_M {
        return [0, 0, 0, 255];
    }

    let t = (distance / sim_zed::ZED_SIM_MAX_DEPTH_M).clamp(0.0, 1.0);
    let intensity = ((1.0 - t).sqrt() * 255.0).round() as u8;
    [intensity, intensity, intensity, 255]
}

fn setup_world(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut images: ResMut<Assets<Image>>,
    layout: Res<WorldLayout>,
    asset_paths: Res<SceneAssetPaths>,
) {
    let world_size_units = SIM_WORLD.bbox_max_units - SIM_WORLD.bbox_min_units;
    let world_size_m = world_size_units * SIM_WORLD.scale;
    let world_translation = Vec3::ZERO;
    let world_scale = Vec3::splat(SIM_WORLD.scale);
    info!(
        "sim world: loading {} (bbox {}x{}x{} units, scaled to {}x{}x{} m) with translation ({}, {}, {})",
        asset_paths.world.as_str(),
        world_size_units.x,
        world_size_units.y,
        world_size_units.z,
        world_size_m.x,
        world_size_m.y,
        world_size_m.z,
        world_translation.x,
        world_translation.y,
        world_translation.z
    );

    let skybox_handle = asset_server.load(asset_paths.skybox.clone());
    let specular_map_handle = asset_server.load(asset_paths.specular_map.clone());

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
            image: Some(skybox_handle.clone()),
            brightness: 1000.0,
            ..default()
        },
        EnvironmentMapLight {
            diffuse_map: skybox_handle.clone(),
            specular_map: specular_map_handle.clone(),
            intensity: 900.0,
            ..default()
        },
        Transform::from_xyz(-2.0, 1.6, -2.0).looking_at(Vec3::ZERO, Vec3::Y),
        #[cfg(feature = "forest-world")]
        DistanceFog {
            color: Color::srgba(0.58, 0.68, 0.65, 0.22),
            falloff: FogFalloff::from_visibility(180.0),
            ..default()
        },
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

    let depth_texture = images.add(zed_depth_texture());
    let depth_preview = images.add(zed_depth_preview());
    let left_image = images.add(zed_color_texture());
    let right_image = images.add(zed_color_texture());
    commands.insert_resource(ZedDepthTexture(depth_texture.clone()));
    commands.insert_resource(ZedDepthPreview {
        image: depth_preview,
        last_seq: None,
        last_prediction_seq: 0,
    });
    commands.spawn((
        Name::new("zed2i-left-depth-camera"),
        Camera3d::default(),
        Camera {
            order: ZED_DEPTH_CAMERA_ORDER,
            ..default()
        },
        RenderTarget::Image(left_image.clone().into()),
        Projection::Perspective(PerspectiveProjection {
            fov: sim_zed::ZED_SIM_VERTICAL_FOV_DEG.to_radians(),
            near: sim_zed::ZED_SIM_NEAR_M,
            far: sim_zed::ZED_SIM_MAX_DEPTH_M,
            ..default()
        }),
        Msaa::Off,
        DepthPrepass,
        Transform::from_xyz(0.0, 1.0, 0.0),
        ZedDepthCamera,
    ));
    commands.spawn((
        Name::new("zed2i-right-camera"),
        Camera3d::default(),
        Camera {
            order: ZED_RIGHT_CAMERA_ORDER,
            ..default()
        },
        RenderTarget::Image(right_image.clone().into()),
        Projection::Perspective(PerspectiveProjection {
            fov: sim_zed::ZED_SIM_VERTICAL_FOV_DEG.to_radians(),
            near: sim_zed::ZED_SIM_NEAR_M,
            far: sim_zed::ZED_SIM_MAX_DEPTH_M,
            ..default()
        }),
        Msaa::Off,
        Transform::from_xyz(0.0, 1.0, 0.0),
        ZedRightCamera,
    ));
    commands
        .spawn((
            Name::new("zed2i-depth-readback"),
            Readback::texture(depth_texture),
        ))
        .observe(capture_zed_depth);
    commands
        .spawn((
            Name::new("zed2i-left-readback"),
            Readback::texture(left_image),
        ))
        .observe(capture_zed_left_image);
    commands
        .spawn((
            Name::new("zed2i-right-readback"),
            Readback::texture(right_image),
        ))
        .observe(capture_zed_right_image);

    commands.spawn((
        Name::new("sun"),
        DirectionalLight {
            illuminance: 12_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_translation(Vec3::new(3.0, 10.0, 1.0)).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let quadcopter_scene_path = format!("{}#scene0", asset_paths.quadcopter.as_str());
    let world_scene_path = format!("{}#scene0", asset_paths.world.as_str());

    let quadcopter_scene =
        asset_server.load(GltfAssetLabel::Scene(0).from_asset(quadcopter_scene_path));
    let world_scene = asset_server.load(GltfAssetLabel::Scene(0).from_asset(world_scene_path));
    commands.insert_resource(PendingQuadcopterSpawn {
        quadcopter_scene: quadcopter_scene.clone(),
        world_scene: world_scene.clone(),
    });

    commands.spawn((
        Name::new(SIM_WORLD.entity_name),
        WorldAssetRoot(world_scene),
        Transform {
            translation: world_translation,
            scale: world_scale,
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

    if !asset_server.is_loaded_with_dependencies(pending_spawn.world_scene.id())
        || !asset_server.is_loaded_with_dependencies(pending_spawn.quadcopter_scene.id())
        || colliders.is_empty()
    {
        return;
    }

    let (transform, position, rotation, lin_vel, ang_vel) = spawn_pose_components();
    commands
        .spawn((
            Name::new("quadcopter"),
            WorldAssetRoot(pending_spawn.quadcopter_scene.clone()),
            RigidBody::Dynamic,
            transform,
            position,
            rotation,
            lin_vel,
            ang_vel,
            Collider::cuboid(
                SIM_QUADCOPTER_COLLIDER_SIZE.x,
                SIM_QUADCOPTER_COLLIDER_SIZE.y,
                SIM_QUADCOPTER_COLLIDER_SIZE.z,
            ),
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
                                font_size: FontSize::Px(18.0),
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
                bevy::ui::BackgroundColor(Color::srgba(0.03, 0.05, 0.09, 0.94)),
                bevy::ui::BorderColor::all(Color::srgba(0.58, 0.74, 0.96, 0.95)),
            ))
            .with_children(|help| {
                help.spawn((
                    Pickable::IGNORE,
                    Text::new("View\nRC Link\nArm\nMode\nAuto\nThrottle\nRoll/Pitch\nYaw\nReset"),
                    TextFont {
                        font_size: FontSize::Px(12.0),
                        ..default()
                    },
                    TextColor(Color::srgb(0.78, 0.86, 0.96)),
                ));

                help.spawn((
                    Pickable::IGNORE,
                    SimHelpValuesText,
                    Text::new("FPV (V)\nChecking RC link..."),
                    TextFont {
                        font_size: FontSize::Px(12.0),
                        ..default()
                    },
                    TextColor(Color::WHITE),
                ));
            });
    });
    spawned.help = true;
}

fn spawn_zed_overlay(
    mut commands: Commands,
    hud_root: Option<Res<SimHudRoot>>,
    preview_image: Option<Res<ZedDepthPreview>>,
    mut spawned: ResMut<SimHudSpawnState>,
) {
    if spawned.zed {
        return;
    }
    let (Some(hud_root), Some(preview_image)) = (hud_root, preview_image) else {
        return;
    };

    commands.entity(hud_root.0).with_children(|parent| {
        parent
            .spawn((
                Name::new("zed2i-preview"),
                Node {
                    position_type: PositionType::Absolute,
                    top: Val::Px(5.0),
                    right: Val::Px(5.0),
                    padding: UiRect::all(Val::Px(5.0)),
                    row_gap: Val::Px(4.0),
                    flex_direction: bevy::ui::FlexDirection::Column,
                    border: UiRect::all(Val::Px(2.0)),
                    border_radius: bevy::ui::BorderRadius::all(Val::Px(8.0)),
                    ..default()
                },
                Pickable::IGNORE,
                bevy::ui::BackgroundColor(Color::srgba(0.03, 0.05, 0.09, 0.94)),
                bevy::ui::BorderColor::all(Color::srgba(0.58, 0.74, 0.96, 0.95)),
            ))
            .with_children(|preview| {
                preview.spawn((
                    Pickable::IGNORE,
                    Text::new(ZED_DEPTH_PREVIEW_TITLE),
                    TextFont {
                        font_size: FontSize::Px(12.0),
                        ..default()
                    },
                    TextColor(Color::srgb(0.78, 0.86, 0.96)),
                ));
                preview.spawn((
                    Node {
                        width: Val::Px(sim_zed::ZED_SIM_WIDTH as f32),
                        height: Val::Px(sim_zed::ZED_SIM_HEIGHT as f32),
                        ..default()
                    },
                    Pickable::IGNORE,
                    ImageNode::new(preview_image.image.clone()),
                ));
            });
    });
    spawned.zed = true;
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
            init_keyboard_rc(&mut rc_input);
            info!(
                "sim rc: joystick unavailable ({}), using keyboard controls (disarmed start, Space arms Angle mode) (set CU_SIM_ALLOW_GENERIC_JOYSTICK=1 to allow non-radio joysticks)",
                err.to_string()
            );
            joystick_state.reader = None;
        }
    }
}

#[cfg(target_arch = "wasm32")]
fn setup_joystick(mut rc_input: ResMut<SimRcInput>, mut rc_source: ResMut<RcInputSource>) {
    *rc_source = RcInputSource::Keyboard;
    init_keyboard_rc(&mut rc_input);
    info!("sim rc: web build using keyboard controls (disarmed start, Space arms Angle mode)");
}

#[cfg(not(target_arch = "wasm32"))]
fn poll_joystick(
    mut joystick: ResMut<SimJoystickState>,
    mut rc_input: ResMut<SimRcInput>,
    mut rc_source: ResMut<RcInputSource>,
    mut reset_interlock: ResMut<SimResetInterlock>,
) {
    let Some(reader) = joystick.reader.as_mut() else {
        return;
    };

    match reader.next_frame() {
        Ok(Some(frame)) => {
            let prev_armed = rc_input.armed;
            let prev_mode = rc_input.mode;
            let prev_auto = rc_input.auto;
            apply_joystick_frame(&frame, &mut rc_input);
            reset_interlock.apply_joystick_frame(&mut rc_input);
            *rc_source = RcInputSource::Joystick;
            if rc_input.armed != prev_armed
                || rc_input.mode != prev_mode
                || rc_input.auto != prev_auto
            {
                info!(
                    "sim rc: armed={} mode={:?} auto={}",
                    rc_input.armed, rc_input.mode, rc_input.auto
                );
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
            init_keyboard_rc(&mut rc_input);
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
fn three_pos_is_middle(value: f32) -> bool {
    (-0.33..0.33).contains(&value)
}

#[cfg(not(target_arch = "wasm32"))]
fn apply_joystick_frame(frame: &RcFrame, rc_input: &mut SimRcInput) {
    rc_input.roll = frame.roll.clamp(-1.0, 1.0);
    // Match FC stick convention used by keyboard path and RcMapper.
    rc_input.pitch = (-frame.pitch).clamp(-1.0, 1.0);
    rc_input.yaw = (-frame.yaw).clamp(-1.0, 1.0);
    rc_input.throttle = frame.throttle.clamp(0.0, 1.0);

    rc_input.armed = frame.arm.map_or(frame.knob_sa > 0.5, |arm| arm > 0.5);
    rc_input.mode = mode_from_three_pos(frame.knob_sb);
    rc_input.auto = frame.knob_sc.is_some_and(three_pos_is_middle);
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
    if keyboard.just_pressed(KeyCode::Digit4) {
        rc_input.auto = !rc_input.auto;
        info!("sim rc: auto={}", rc_input.auto);
    }
    if keyboard.just_pressed(KeyCode::Space) && !rc_input.armed {
        rc_input.armed = true;
        rc_input.mode = messages::FlightMode::Angle;
        rc_input.auto = false;
        info!("sim rc: keyboard armed in Angle mode");
    }

    if keyboard.just_pressed(KeyCode::KeyT) {
        rc_input.armed = !rc_input.armed;
        info!(
            "sim rc: armed={} mode={:?} auto={}",
            rc_input.armed, rc_input.mode, rc_input.auto
        );
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

    if !rc_input.armed {
        rc_input.throttle = 0.0;
        return;
    }

    // Keyboard mode is a simple "descend a bit / climb a bit" control around hover.
    rc_input.throttle =
        if keyboard.pressed(KeyCode::Space) && !keyboard.just_pressed(KeyCode::Space) {
            KEYBOARD_HOVER_THROTTLE_HIGH
        } else {
            KEYBOARD_HOVER_THROTTLE_LOW
        };
}

#[derive(SystemParam)]
struct SimResetResources<'w> {
    motors: ResMut<'w, SimMotorCommands>,
    kinematics: ResMut<'w, SimKinematics>,
    sim_state: ResMut<'w, SimState>,
    rc_source: Res<'w, RcInputSource>,
    rc_input: ResMut<'w, SimRcInput>,
    interlock: ResMut<'w, SimResetInterlock>,
    zed_store: Res<'w, sim_zed::SimZedFrameStore>,
    #[cfg(feature = "sim")]
    copper: ResMut<'w, CopperState>,
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
    mut reset: SimResetResources,
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

    reset.motors.dshot = [0; 4];
    reset.kinematics.prev_linear_velocity = None;
    reset.sim_state.vehicle = SimVehicleState::default();
    reset.zed_store.reset_dynamic();
    let rc_source = *reset.rc_source;
    reset.interlock.begin(rc_source, &mut reset.rc_input);

    #[cfg(feature = "sim")]
    {
        reset.copper.autonomy_link = SimAutonomyLink::default();
        if let Err(err) = reset.copper.mcu.reset_sim_state() {
            error!("failed to reset MCU task state: {}", err);
        }
        if let Err(err) = reset.copper.compute.reset_sim_state() {
            error!("failed to reset compute task state: {}", err);
        }
    }

    match rc_source {
        RcInputSource::Keyboard => info!("sim reset: disarmed and cleared controller state"),
        RcInputSource::Joystick => info!(
            "sim reset: disarmed and cleared controller state; set ARM low and AUTO off before rearming"
        ),
    }
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

fn run_copper(
    mut copper: ResMut<CopperState>,
    physics_time: Res<Time<Physics>>,
    sim_state: Res<SimState>,
    rc_input: Res<SimRcInput>,
    mut motor_commands: ResMut<SimMotorCommands>,
    mut osd_overlay: ResMut<SimOsdOverlay>,
    mut exit_writer: MessageWriter<AppExit>,
) {
    let elapsed_ns = physics_time.elapsed().as_nanos() as u64;
    if let Err(err) = run_copper_iteration(
        &mut copper,
        elapsed_ns,
        sim_state.vehicle.clone(),
        rc_input.clone(),
        &mut motor_commands,
        &mut osd_overlay,
    ) {
        error!("sim loop stopped: {}", err);
        exit_writer.write(AppExit::Success);
    }
}

fn run_copper_iteration(
    copper: &mut CopperState,
    elapsed_ns: u64,
    vehicle: SimVehicleState,
    rc: SimRcInput,
    motor_commands: &mut SimMotorCommands,
    osd_overlay: &mut SimOsdOverlay,
) -> CuResult<()> {
    copper.clock_mock.set_value(elapsed_ns);
    sim_battery_set_armed(rc.armed);
    sim_battery_set_throttle(rc.throttle);
    sim_gnss_set_vehicle_state(
        [vehicle.position.x, vehicle.position.y, vehicle.position.z],
        [
            vehicle.velocity_world.x,
            vehicle.velocity_world.y,
            vehicle.velocity_world.z,
        ],
    );
    let world_mag = Vec3::from_array(WORLD_MAG_FIELD_UT);
    let body_mag = vehicle.rotation.inverse() * world_mag;
    let altitude_m = vehicle.position.y.max(-100.0);
    let pressure_pa = 101_325.0 * (1.0 - altitude_m / 44_330.0).powf(5.255);
    copper.compute.set_zed_sensors(sim_zed::SimZedSensors {
        imu: ImuPayload::from_raw(vehicle.body_accel_fc, vehicle.body_gyro_fc, 29.0),
        magnetometer: MagnetometerPayload::from_raw(map_bevy_body_to_fc_magnetometer(body_mag)),
        barometer: BarometerPayload::from_raw(pressure_pa, 25.0),
    });
    copper.mcu.run_iteration(
        &copper.clock,
        vehicle,
        rc,
        motor_commands,
        osd_overlay,
        &mut copper.autonomy_link,
    )?;
    copper
        .compute
        .run_iteration(&copper.clock, &mut copper.autonomy_link)
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

fn zed_camera_follow_quadcopter(
    quadcopter_query: Query<&GlobalTransform, With<Multicopter>>,
    mut cameras: Query<(&mut Transform, Option<&ZedDepthCamera>), ZedCameraFilter>,
) {
    let Ok(quad_tf) = quadcopter_query.single() else {
        return;
    };
    let camera_center = quad_tf.translation() + 0.08 * quad_tf.up() + 0.16 * quad_tf.forward();
    let half_baseline = sim_zed::ZED_SIM_BASELINE_M * 0.5;

    for (mut camera, left) in &mut cameras {
        let horizontal_offset = if left.is_some() {
            -half_baseline
        } else {
            half_baseline
        };
        camera.translation = camera_center + horizontal_offset * quad_tf.right();
        *camera = camera.looking_to(quad_tf.forward(), quad_tf.up());
    }
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

fn sim_activity_led_is_on() -> bool {
    sim_support::sim_activity_led_state().load(Ordering::Relaxed)
}

fn sim_battery_set_throttle(throttle: f32) {
    let clamped = throttle.clamp(0.0, 1.0);
    sim_support::sim_battery_throttle_state().store(clamped.to_bits(), Ordering::Relaxed);
}

fn sim_battery_set_armed(armed: bool) {
    sim_support::sim_battery_armed_state().store(armed, Ordering::Relaxed);
}

fn sim_gnss_set_vehicle_state(position_xyz_m: [f32; 3], velocity_xyz_mps: [f32; 3]) {
    // Bevy world uses +X east, +Y up, and +Z south. The identity quad therefore
    // faces geographic north along Bevy's -Z forward axis.
    let north_m = -(position_xyz_m[2] as f64);
    let east_m = position_xyz_m[0] as f64;
    let up_m = position_xyz_m[1];

    let meters_per_deg_lon = (EARTH_METERS_PER_DEG_LAT
        * sim_support::GNSS_FIXED_LAT_DEG.to_radians().cos().abs())
    .max(1.0);
    let lat_deg = sim_support::GNSS_FIXED_LAT_DEG + (north_m / EARTH_METERS_PER_DEG_LAT);
    let lon_deg = sim_support::GNSS_FIXED_LON_DEG + (east_m / meters_per_deg_lon);

    let velocity_north_mps = -velocity_xyz_mps[2];
    let velocity_east_mps = velocity_xyz_mps[0];
    let velocity_down_mps = -velocity_xyz_mps[1];
    let ground_speed_mps = libm::sqrtf(
        velocity_north_mps * velocity_north_mps + velocity_east_mps * velocity_east_mps,
    )
    .max(0.0);
    let heading_motion_deg = if ground_speed_mps > 1.0e-3 {
        wrap_heading_deg(libm::atan2f(velocity_east_mps, velocity_north_mps).to_degrees())
    } else {
        0.0
    };

    let state = sim_support::sim_gnss_state();
    state
        .lat_deg_bits
        .store(lat_deg.to_bits(), Ordering::Relaxed);
    state
        .lon_deg_bits
        .store(lon_deg.to_bits(), Ordering::Relaxed);
    state.ellipsoid_alt_m_bits.store(
        (sim_support::GNSS_FIXED_ELLIPSOID_ALT_M + up_m).to_bits(),
        Ordering::Relaxed,
    );
    state.msl_alt_m_bits.store(
        (sim_support::GNSS_FIXED_MSL_ALT_M + up_m).to_bits(),
        Ordering::Relaxed,
    );
    state
        .velocity_north_mps_bits
        .store(velocity_north_mps.to_bits(), Ordering::Relaxed);
    state
        .velocity_east_mps_bits
        .store(velocity_east_mps.to_bits(), Ordering::Relaxed);
    state
        .velocity_down_mps_bits
        .store(velocity_down_mps.to_bits(), Ordering::Relaxed);
    state
        .ground_speed_mps_bits
        .store(ground_speed_mps.to_bits(), Ordering::Relaxed);
    state
        .heading_motion_deg_bits
        .store(heading_motion_deg.to_bits(), Ordering::Relaxed);
}

fn wrap_heading_deg(value: f32) -> f32 {
    value.rem_euclid(360.0)
}

fn track_sim_led_state() {
    let _on = sim_activity_led_is_on();
}

#[cfg(not(target_arch = "wasm32"))]
fn axis_or_unmapped(axis: Option<&String>) -> &str {
    axis.map_or("unmapped", String::as_str)
}

#[cfg(not(target_arch = "wasm32"))]
fn connected_help_values(
    view_label: &str,
    joystick: &RcJoystick,
    reset_interlock: &SimResetInterlock,
) -> String {
    let axes: RcAxisBindings = joystick.axis_bindings();
    let frame = joystick.current_frame();
    let device_name = joystick.device_name();

    let arm_line = if frame.arm.is_some() {
        format!("arm {} > 0.5", axis_or_unmapped(axes.arm.as_ref()))
    } else {
        format!("knob_sa {} > 0.5", axis_or_unmapped(axes.knob_sa.as_ref()))
    };

    let mode_line = format!(
        "knob_sb {} (3-pos)",
        axis_or_unmapped(axes.knob_sb.as_ref())
    );
    let auto_line = format!(
        "knob_sc {} (middle auto)",
        axis_or_unmapped(axes.knob_sc.as_ref())
    );

    let reset_line = if reset_interlock.waiting_for_joystick_safe {
        "ARM LOW + AUTO OFF"
    } else {
        "R"
    };
    format!(
        "{view_label}\nConnected ({device_name})\n{arm_line}\n{mode_line}\n{auto_line}\n{}\n{} / {}\n{}\n{reset_line}",
        axis_or_unmapped(axes.throttle.as_ref()),
        axis_or_unmapped(axes.roll.as_ref()),
        axis_or_unmapped(axes.pitch.as_ref()),
        axis_or_unmapped(axes.yaw.as_ref()),
    )
}

fn update_help_overlay(
    camera_view: Res<CameraView>,
    rc_source: Res<RcInputSource>,
    reset_interlock: Res<SimResetInterlock>,
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
            "{view_label}\nNot connected (plug RC via USB/BT)\nT\n1=Acro 2=Angle 3=PosHold\n4=Auto toggle\nSpace (arm Angle / climb)\nWASD\nQ / E\nR"
        ),
        RcInputSource::Joystick => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                _joystick_state.reader.as_ref().map_or_else(
                    || {
                        format!(
                            "{view_label}\nConnected (USB/BT)\nRC source initializing...\n-\n-\n-\n-\n-\nR"
                        )
                    },
                    |joy| connected_help_values(view_label, joy, &reset_interlock),
                )
            }
            #[cfg(target_arch = "wasm32")]
            {
                format!(
                    "{view_label}\nWeb build keyboard mode\nT\n1=Acro 2=Angle 3=PosHold\n4=Auto toggle\nSpace (arm Angle / climb)\nWASD\nQ / E\nR"
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
    let Some(mut canvas_image) = images.get_mut(&osd_canvas_assets.canvas) else {
        return;
    };

    rasterize_osd_canvas(&osd_overlay, &raster_source, &mut canvas_image);
}

fn stop_copper_on_exit(mut exit_events: MessageReader<AppExit>, mut copper: ResMut<CopperState>) {
    for _ in exit_events.read() {
        let _ = copper.mcu.stop();
        let _ = copper.mcu.log_shutdown_completed();
        let _ = copper.compute.stop();
        let _ = copper.compute.log_shutdown_completed();
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
    app.register_type::<bevy::prelude::Handle<bevy::prelude::WorldAsset>>();
    app.register_type::<bevy::gltf::GltfExtras>();
    app.register_type::<bevy::gltf::GltfSceneExtras>();
    app.register_type::<bevy::gltf::GltfSceneName>();
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
    #[cfg(not(target_arch = "wasm32"))]
    let app_id = if split_monitor {
        "io.github.copper-project.flight-controller-bevymon"
    } else {
        "io.github.copper-project.flight-controller-sim"
    };

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
            name: Some(app_id.into()),
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
    build_world_with_assets(
        headless,
        split_monitor,
        None,
        sim_zed::SimZedFrameStore::default(),
    )
}

fn build_world_with_assets(
    headless: bool,
    split_monitor: bool,
    scene_assets: Option<SceneAssetPaths>,
    zed_store: sim_zed::SimZedFrameStore,
) -> App {
    let mut app = App::new();
    app.insert_resource(SimState::default())
        .insert_resource(SimMotorCommands::default())
        .insert_resource(SimRcInput::default())
        .insert_resource(SimKinematics::default())
        .insert_resource(SimResetInterlock::default())
        .insert_resource(RcInputSource::default())
        .insert_resource(SimJoystickState::default())
        .insert_resource(CameraView::default())
        .insert_resource(SimOsdOverlay::default())
        .insert_resource(zed_store)
        .init_resource::<OsdRasterSource>()
        .insert_resource(WorldLayout { split_monitor })
        .init_resource::<SceneLoadState>()
        .init_resource::<SimHudSpawnState>();

    if !headless {
        app.insert_resource(scene_assets.unwrap_or_else(prepare_scene_assets));
    }

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
    );
    app.add_plugins(ExtractResourcePlugin::<ZedDepthTexture>::default());
    app.get_sub_app_mut(RenderApp)
        .expect("render app should be present")
        .add_systems(
            Core3d,
            copy_zed_depth_texture
                .after(Core3dSystems::Prepass)
                .before(Core3dSystems::MainPass),
        );
    #[cfg(not(target_arch = "wasm32"))]
    app.add_systems(Update, windowing::set_copper_window_icon);
    app.add_plugins(PhysicsPlugins::default())
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
                spawn_zed_overlay,
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
                zed_camera_follow_quadcopter,
                update_zed_depth_preview,
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
    let scene_assets = prepare_scene_assets();
    let zed_store = sim_zed::SimZedFrameStore::default();
    let mut app = build_world_with_assets(false, false, Some(scene_assets), zed_store);
    #[cfg(not(target_arch = "wasm32"))]
    {
        app.add_systems(Startup, setup_copper);
        app.add_systems(
            FixedUpdate,
            (run_copper, apply_multicopter_dynamics)
                .chain()
                .after(sync_vehicle_state),
        );
        app.add_systems(PostUpdate, stop_copper_on_exit);
    }
    app.run();
}

#[cfg(feature = "bevymon")]
pub fn run_bevymon() {
    let scene_assets = prepare_scene_assets();
    let zed_store = sim_zed::SimZedFrameStore::default();
    let (monitor_model, copper) = build_bevymon_copper(zed_store.clone());

    let mut app = build_world_with_assets(false, true, Some(scene_assets), zed_store);
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
            (run_copper, apply_multicopter_dynamics)
                .chain()
                .after(sync_vehicle_state),
        )
        .add_systems(PostUpdate, stop_copper_on_exit);
    app.run();
}

#[cfg(feature = "sim")]
fn main() {
    run_sim();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Mutex, OnceLock};

    fn heading_from_mag_xy_deg(mag: [f32; 3]) -> f32 {
        let mut heading = libm::atan2f(mag[1], mag[0]).to_degrees();
        if heading < 0.0 {
            heading += 360.0;
        }
        heading
    }

    fn sim_gnss_test_lock() -> &'static Mutex<()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
    }

    fn copper_runtime_test_lock() -> &'static Mutex<()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn simulator_logs_are_anchored_to_the_example_directory() {
        let example_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        let (mcu, compute) = sim_log_paths();
        assert_eq!(mcu, example_dir.join("logs/flight_controller_sim.copper"));
        assert_eq!(compute, example_dir.join("logs/flight_compute_sim.copper"));
    }

    fn assert_heading_close(actual: f32, expected: f32) {
        let err = (actual - expected + 540.0).rem_euclid(360.0) - 180.0;
        assert!(
            err.abs() < 1.0e-3,
            "heading mismatch: actual={actual} expected={expected} err={err}"
        );
    }

    #[test]
    fn zed_reverse_z_depth_is_linearized_in_meters() {
        let near = sim_zed::ZED_SIM_NEAR_M;
        let far = sim_zed::ZED_SIM_MAX_DEPTH_M;
        assert!((zed_reverse_z_to_distance(1.0) - near).abs() < f32::EPSILON);
        assert!(zed_reverse_z_to_distance(0.0).is_nan());

        let expected_distance = 5.0;
        let reverse_z = near * (far - expected_distance) / (expected_distance * (far - near));
        assert!((zed_reverse_z_to_distance(reverse_z) - expected_distance).abs() < 1.0e-4);
    }

    #[test]
    fn vitfly_preview_arrow_uses_python_image_axes() {
        let width = 40;
        let height = 20;
        let mut pixels = vec![0; width * height * 4];
        draw_vitfly_arrow(&mut pixels, width, height, [2.0, 0.5, 0.5]);

        let is_red = |x: usize, y: usize| pixels[(y * width + x) * 4..][..4] == [255, 32, 32, 255];
        assert!(is_red(width / 2, height / 2));
        assert!(
            (0..width / 2).any(|x| (0..height / 2).any(|y| is_red(x, y))),
            "positive left/up velocity should point toward the image's upper-left"
        );
    }

    #[cfg(feature = "sim")]
    #[test]
    fn vitfly_identity_pose_uses_zero_aerospace_angles() {
        let pose = vitfly_pose(Quat::IDENTITY);
        assert_eq!(pose.roll.get::<radian>(), 0.0);
        assert_eq!(pose.pitch.get::<radian>(), 0.0);
        assert_eq!(pose.yaw.get::<radian>(), 0.0);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn elrs_arm_axis_takes_precedence_over_hid_buttons() {
        let mut rc_input = SimRcInput::default();
        let mut frame = RcFrame {
            arm: Some(1.0),
            switches: vec![rc_joystick::SwitchState {
                name: "ch9".to_string(),
                on: false,
            }],
            ..RcFrame::default()
        };

        apply_joystick_frame(&frame, &mut rc_input);
        assert!(rc_input.armed);

        frame.arm = Some(-1.0);
        frame.switches[0].on = true;
        apply_joystick_frame(&frame, &mut rc_input);
        assert!(!rc_input.armed);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn joystick_maps_sa_sb_and_sc_independently() {
        let mut rc_input = SimRcInput::default();
        let mut frame = RcFrame {
            arm: Some(1.0),
            knob_sb: 1.0,
            knob_sc: Some(0.0),
            ..RcFrame::default()
        };

        apply_joystick_frame(&frame, &mut rc_input);
        assert!(rc_input.armed);
        assert_eq!(rc_input.mode, messages::FlightMode::PositionHold);
        assert!(rc_input.auto);

        frame.knob_sb = -1.0;
        frame.knob_sc = Some(1.0);
        apply_joystick_frame(&frame, &mut rc_input);
        assert!(rc_input.armed);
        assert_eq!(rc_input.mode, messages::FlightMode::Acro);
        assert!(!rc_input.auto);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn joystick_reset_interlock_requires_safe_switch_positions() {
        let mut rc_input = SimRcInput {
            armed: true,
            auto: true,
            throttle: 0.8,
            ..SimRcInput::default()
        };
        let mut interlock = SimResetInterlock::default();
        interlock.begin(RcInputSource::Joystick, &mut rc_input);
        assert!(!rc_input.armed);
        assert!(!rc_input.auto);

        let mut frame = RcFrame {
            arm: Some(1.0),
            knob_sc: Some(0.0),
            throttle: 0.8,
            ..RcFrame::default()
        };
        apply_joystick_frame(&frame, &mut rc_input);
        interlock.apply_joystick_frame(&mut rc_input);
        assert!(!rc_input.armed, "held ARM switch must not bypass reset");
        assert!(!rc_input.auto, "held AUTO switch must not bypass reset");
        assert_eq!(rc_input.throttle, 0.0);

        frame.arm = Some(-1.0);
        apply_joystick_frame(&frame, &mut rc_input);
        interlock.apply_joystick_frame(&mut rc_input);
        assert!(
            interlock.waiting_for_joystick_safe,
            "AUTO must also be switched off"
        );

        frame.knob_sc = Some(1.0);
        apply_joystick_frame(&frame, &mut rc_input);
        interlock.apply_joystick_frame(&mut rc_input);
        assert!(!interlock.waiting_for_joystick_safe);

        frame.arm = Some(1.0);
        apply_joystick_frame(&frame, &mut rc_input);
        interlock.apply_joystick_frame(&mut rc_input);
        assert!(rc_input.armed, "arming should work after the safe cycle");
        assert!(!rc_input.auto);
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn build_test_copper_state() -> CopperState {
        let (clock, clock_mock) = RobotClock::mock();
        let mut mcu = mcu_copper::Runtime::without_logger(&clock);
        let mut compute =
            compute_copper::Runtime::new(&clock, sim_zed::SimZedFrameStore::default());

        mcu.start().expect("failed to start tasks");
        compute.start().expect("failed to start compute tasks");

        CopperState {
            clock,
            clock_mock,
            mcu,
            compute,
            autonomy_link: SimAutonomyLink::default(),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn build_scene_asset_test_app(register_reflect_types: bool) -> App {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            asset_plugin(),
            bevy::world_serialization::WorldSerializationPlugin,
            bevy::image::ImagePlugin::default(),
            bevy::mesh::MeshPlugin,
            bevy::pbr::MaterialPlugin::<bevy::prelude::StandardMaterial>::default(),
            bevy::gltf::GltfPlugin::default(),
        ));
        app.insert_resource(bevy::image::CompressedImageFormatSupport(
            bevy::image::CompressedImageFormats::NONE,
        ));
        if register_reflect_types {
            register_scene_reflect_types(&mut app);
        }
        app.finish();
        app
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn load_scene_asset_handles(
        app: &mut App,
        scene_assets: &SceneAssetPaths,
    ) -> (Handle<WorldAsset>, Handle<WorldAsset>) {
        let asset_server = app.world().resource::<AssetServer>();
        (
            asset_server.load(GltfAssetLabel::Scene(0).from_asset(scene_assets.quadcopter.clone())),
            asset_server.load(GltfAssetLabel::Scene(0).from_asset(scene_assets.world.clone())),
        )
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn wait_for_scene_asset_loads(
        app: &mut App,
        quadcopter_scene: &Handle<WorldAsset>,
        world_scene: &Handle<WorldAsset>,
    ) -> bool {
        for _ in 0..300 {
            app.update();
            let asset_server = app.world().resource::<AssetServer>();
            if asset_server.is_loaded_with_dependencies(quadcopter_scene.id())
                && asset_server.is_loaded_with_dependencies(world_scene.id())
            {
                return true;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        false
    }

    #[test]
    fn sim_world_starts() {
        let mut app = build_world(true, false);
        app.update();
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn sim_copper_runs_one_iteration() {
        let _guard = copper_runtime_test_lock().lock().unwrap();
        let mut copper = build_test_copper_state();
        let mut motors = SimMotorCommands::default();
        let mut osd_overlay = SimOsdOverlay::default();

        run_copper_iteration(
            &mut copper,
            0,
            SimVehicleState::default(),
            SimRcInput::default(),
            &mut motors,
            &mut osd_overlay,
        )
        .expect("copper sim iteration should keep running");

        copper
            .mcu
            .reset_sim_state()
            .expect("MCU task state should reset");
        copper
            .compute
            .reset_sim_state()
            .expect("compute task state should reset");

        copper.mcu.stop().expect("failed to stop tasks");
        copper
            .mcu
            .log_shutdown_completed()
            .expect("failed to log shutdown");
        copper.compute.stop().expect("failed to stop compute tasks");
        copper
            .compute
            .log_shutdown_completed()
            .expect("failed to log compute shutdown");
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn simulated_sc_auto_switch_reaches_vtx_osd() {
        let _guard = copper_runtime_test_lock().lock().unwrap();
        let mut copper = build_test_copper_state();
        let mut motors = SimMotorCommands::default();
        let mut osd_overlay = SimOsdOverlay::default();
        let rc = SimRcInput {
            armed: true,
            auto: true,
            ..SimRcInput::default()
        };

        run_copper_iteration(
            &mut copper,
            0,
            SimVehicleState::default(),
            rc,
            &mut motors,
            &mut osd_overlay,
        )
        .expect("copper sim iteration should keep running");

        assert!(
            osd_overlay.cells.windows(4).any(|window| window == b"AUTO"),
            "armed SC auto switch should render AUTO in the VTX OSD"
        );

        copper.mcu.stop().expect("failed to stop tasks");
        copper
            .mcu
            .log_shutdown_completed()
            .expect("failed to log shutdown");
        copper.compute.stop().expect("failed to stop compute tasks");
        copper
            .compute
            .log_shutdown_completed()
            .expect("failed to log compute shutdown");
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn vitfly_is_rate_limited_and_infers_between_context_packets() {
        let _guard = copper_runtime_test_lock().lock().unwrap();
        let (clock, clock_mock) = RobotClock::mock();
        let zed_store = sim_zed::SimZedFrameStore::default();
        let pixel_count = (sim_zed::ZED_SIM_WIDTH * sim_zed::ZED_SIM_HEIGHT) as usize;
        zed_store.set_left_image(vec![1; pixel_count * 4]);
        zed_store.set_right_image(vec![2; pixel_count * 4]);
        zed_store.set_depth(vec![3.5; pixel_count], vec![0.0; pixel_count]);

        let mut compute = compute_copper::Runtime::new(&clock, zed_store.clone());
        compute.start().expect("failed to start compute tasks");
        clock_mock.set_value(1_000_000);
        let mut autonomy_link = SimAutonomyLink::default();
        autonomy_link.context.tov = Tov::Time(clock.now());
        autonomy_link
            .context
            .set_payload(messages::AutonomyContext {
                sequence: 1,
                mission_generation: 1,
                active: false,
                pose: vitfly_pose(Quat::IDENTITY),
                desired_speed: cu29::units::si::f32::Velocity::new::<meter_per_second>(4.0),
                ..messages::AutonomyContext::default()
            });
        compute
            .run_iteration(&clock, &mut autonomy_link)
            .expect("failed to run compute iteration");
        assert!(
            autonomy_link.command.payload().is_none(),
            "inactive inference must not produce an MCU command"
        );

        let published = zed_store
            .published_depth()
            .expect("VitFly input should observe simulated ZED depth");
        assert_eq!(published.seq, 1);
        assert_eq!(published.format.width, sim_zed::ZED_SIM_WIDTH);
        assert_eq!(published.format.height, sim_zed::ZED_SIM_HEIGHT);
        published.buffer_handle.with_inner(|depth| {
            assert_eq!(depth.len(), pixel_count);
            assert_eq!(depth[0], 3.5);
        });
        #[cfg(feature = "sim")]
        {
            let (first_prediction_seq, prediction) = zed_store.vitfly_prediction();
            let prediction = prediction.expect("ViTFly listener should receive a prediction");
            assert!(prediction.iter().all(|component| component.is_finite()));

            clock_mock.increment(CuDuration::from_millis(1));
            compute
                .run_iteration(&clock, &mut autonomy_link)
                .expect("compute should run inside the ViTFly rate-limit interval");
            let (second_prediction_seq, second_prediction) = zed_store.vitfly_prediction();
            assert_eq!(second_prediction_seq, first_prediction_seq);
            assert_eq!(second_prediction, Some(prediction));

            clock_mock.increment(CuDuration::from_millis(33));
            compute
                .run_iteration(&clock, &mut autonomy_link)
                .expect("ViTFly should run at 30 Hz without a fresh context packet");
            let (third_prediction_seq, third_prediction) = zed_store.vitfly_prediction();
            assert!(third_prediction_seq > second_prediction_seq);
            assert!(third_prediction.is_some());

            clock_mock.increment(CuDuration::from_millis(34));
            autonomy_link.context.tov = Tov::Time(clock.now());
            autonomy_link
                .context
                .set_payload(messages::AutonomyContext {
                    sequence: 2,
                    mission_generation: 1,
                    active: true,
                    pose: vitfly_pose(Quat::IDENTITY),
                    desired_speed: cu29::units::si::f32::Velocity::new::<meter_per_second>(4.0),
                    ..messages::AutonomyContext::default()
                });
            compute
                .run_iteration(&clock, &mut autonomy_link)
                .expect("active ViTFly inference should produce a command");
            let command = autonomy_link
                .command
                .payload()
                .expect("active ViTFly command should cross the simulated bridge");
            assert!(command.north.get::<meter_per_second>() >= 1.0);
            assert_eq!(
                zed_store.vitfly_prediction().1,
                Some([
                    command.north.get::<meter_per_second>(),
                    command.west.get::<meter_per_second>(),
                    command.up.get::<meter_per_second>(),
                ]),
                "active preview should show the conditioned command sent to the controller"
            );
        }

        compute.stop().expect("failed to stop compute tasks");
        compute
            .log_shutdown_completed()
            .expect("failed to log compute shutdown");
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn sim_reset_clears_vitfly_recurrent_state() {
        let _guard = copper_runtime_test_lock().lock().unwrap();
        let (clock, clock_mock) = RobotClock::mock();
        let zed_store = sim_zed::SimZedFrameStore::default();
        let pixel_count = (sim_zed::ZED_SIM_WIDTH * sim_zed::ZED_SIM_HEIGHT) as usize;
        zed_store.set_depth(vec![6.0; pixel_count], vec![0.0; pixel_count]);

        let mut compute = compute_copper::Runtime::new(&clock, zed_store.clone());
        compute.start().expect("failed to start compute tasks");
        let context = messages::AutonomyContext {
            sequence: 1,
            mission_generation: 1,
            active: true,
            pose: vitfly_pose(Quat::IDENTITY),
            desired_speed: cu29::units::si::f32::Velocity::new::<meter_per_second>(4.0),
            ..messages::AutonomyContext::default()
        };
        let mut autonomy_link = SimAutonomyLink::default();

        clock_mock.set_value(1_000_000);
        autonomy_link.context.tov = Tov::Time(clock.now());
        autonomy_link.context.set_payload(context);
        compute
            .run_iteration(&clock, &mut autonomy_link)
            .expect("first ViTFly inference should run");
        let (_, first_prediction) = zed_store.vitfly_prediction();
        let first_prediction = first_prediction.expect("first prediction should be published");

        clock_mock.set_value(35_000_000);
        compute
            .run_iteration(&clock, &mut autonomy_link)
            .expect("second ViTFly inference should run");

        compute
            .reset_sim_state()
            .expect("ViTFly recurrent state should reset");
        clock_mock.set_value(3_000_000);
        autonomy_link.context.tov = Tov::Time(clock.now());
        autonomy_link.context.set_payload(context);
        compute
            .run_iteration(&clock, &mut autonomy_link)
            .expect("post-reset ViTFly inference should run");
        let (_, reset_prediction) = zed_store.vitfly_prediction();
        let reset_prediction = reset_prediction.expect("reset prediction should be published");

        for (first, reset) in first_prediction.into_iter().zip(reset_prediction) {
            assert!(
                (first - reset).abs() < 1.0e-5,
                "reset inference {reset} did not match initial inference {first}"
            );
        }

        compute.stop().expect("failed to stop compute tasks");
        compute
            .log_shutdown_completed()
            .expect("failed to log compute shutdown");
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn subsystem_logs_record_predictions_and_complete_distributed_replay() {
        let _guard = copper_runtime_test_lock().lock().unwrap();
        let temp_dir = tempfile::tempdir().expect("failed to create temporary log directory");
        let mcu_log_base = temp_dir.path().join("flight_controller_sim.copper");
        let compute_log_base = temp_dir.path().join("flight_compute_sim.copper");
        let zed_store = sim_zed::SimZedFrameStore::default();
        let pixel_count = (sim_zed::ZED_SIM_WIDTH * sim_zed::ZED_SIM_HEIGHT) as usize;
        zed_store.set_depth(vec![3.5; pixel_count], vec![0.0; pixel_count]);

        let mut copper = build_sim_copper_state(
            &mcu_log_base,
            &compute_log_base,
            Some(128 * 1024 * 1024),
            zed_store,
        );
        let mut motors = SimMotorCommands::default();
        let mut osd_overlay = SimOsdOverlay::default();
        let rc = SimRcInput {
            armed: true,
            auto: true,
            mode: messages::FlightMode::PositionHold,
            ..SimRcInput::default()
        };
        for elapsed_ns in [1_000_000, 2_000_000] {
            run_copper_iteration(
                &mut copper,
                elapsed_ns,
                SimVehicleState::default(),
                rc.clone(),
                &mut motors,
                &mut osd_overlay,
            )
            .expect("failed to run logged subsystem iteration");
        }
        copper.mcu.stop().expect("failed to stop MCU tasks");
        copper
            .mcu
            .log_shutdown_completed()
            .expect("failed to log MCU shutdown");
        copper.compute.stop().expect("failed to stop compute tasks");
        copper
            .compute
            .log_shutdown_completed()
            .expect("failed to log compute shutdown");
        drop(copper);

        let discovered =
            cu29::distributed_replay::DistributedReplayLog::discover(&compute_log_base)
                .expect("compute log should be discoverable for distributed replay");
        assert_eq!(discovered.instance_id(), 0);
        assert_eq!(discovered.subsystem_id(), Some("compute"));

        let UnifiedLogger::Read(logger) = UnifiedLoggerBuilder::new()
            .file_base_name(&compute_log_base)
            .build()
            .expect("failed to reopen compute log")
        else {
            panic!("expected a readable compute log");
        };
        let mut reader = UnifiedLoggerIOReader::new(logger, UnifiedLogType::CopperList);
        let prediction =
            cu29_export::copperlists_reader::<compute_copper::RecordedDataSet>(&mut reader)
                .find_map(|copperlist| copperlist.msgs.get_vitfly_output().payload().copied())
                .expect("compute log should contain a ViTFly prediction");

        assert!(prediction.iter().all(|component| {
            component
                .get::<cu29::units::si::velocity::meter_per_second>()
                .is_finite()
        }));

        let multi_config_path = Path::new(env!("CARGO_MANIFEST_DIR")).join("flight_controller.ron");
        let builder = cu29::distributed_replay::DistributedReplayPlan::builder(multi_config_path)
            .expect("failed to load distributed replay config")
            .discover_logs_under(temp_dir.path())
            .expect("failed to discover subsystem logs");
        let builder = mcu_copper::register_distributed_replay(builder)
            .expect("failed to register MCU replay app");
        let plan = compute_copper::register_distributed_replay(builder)
            .expect("failed to register compute replay app")
            .build()
            .expect("the two subsystem logs should form a valid replay plan");
        assert_eq!(plan.assignments.len(), 2);
        assert!(plan.assignment(0, "mcu").is_some());
        assert!(plan.assignment(0, "compute").is_some());

        let mut replay = plan.start().expect("failed to start distributed replay");
        replay
            .run_all()
            .expect("failed to replay both subsystem logs");
        assert_eq!(replay.executed_nodes(), replay.total_nodes());
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn sim_gltf_world_assets_load() {
        let scene_assets = prepare_scene_assets();
        let mut app = build_scene_asset_test_app(false);
        let (quadcopter_scene, world_scene) = load_scene_asset_handles(&mut app, &scene_assets);

        if wait_for_scene_asset_loads(&mut app, &quadcopter_scene, &world_scene) {
            return;
        }

        let asset_server = app.world().resource::<AssetServer>();
        panic!(
            "scene assets did not load: quadcopter_loaded={} world_loaded={}",
            asset_server.is_loaded_with_dependencies(quadcopter_scene.id()),
            asset_server.is_loaded_with_dependencies(world_scene.id())
        );
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn sim_gltf_world_assets_clone_with_registered_types() {
        let scene_assets = prepare_scene_assets();
        let mut app = build_scene_asset_test_app(true);
        let (quadcopter_scene, world_scene) = load_scene_asset_handles(&mut app, &scene_assets);
        assert!(
            wait_for_scene_asset_loads(&mut app, &quadcopter_scene, &world_scene),
            "scene assets did not load"
        );

        let registry = app
            .world()
            .resource::<bevy::ecs::reflect::AppTypeRegistry>()
            .clone();
        let world_assets = app.world().resource::<Assets<WorldAsset>>();
        for (name, handle) in [("quadcopter", &quadcopter_scene), ("world", &world_scene)] {
            let world_asset = world_assets
                .get(handle)
                .unwrap_or_else(|| panic!("{name} world asset did not load"));
            world_asset
                .clone_with(&registry)
                .unwrap_or_else(|err| panic!("{name} world asset cannot spawn: {err}"));
        }
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

        let north_body = Quat::IDENTITY.inverse() * world_mag;
        let north_fc = map_bevy_body_to_fc_magnetometer(north_body);
        assert_eq!(north_fc, [20.0, 0.0, 45.0]);
        assert_heading_close(heading_from_mag_xy_deg(north_fc), 0.0);

        // Positive Bevy yaw rotates counter-clockwise; compass heading decreases clockwise.
        let yaw_90_body = Quat::from_rotation_y(90.0_f32.to_radians()).inverse() * world_mag;
        let yaw_90_fc = map_bevy_body_to_fc_magnetometer(yaw_90_body);
        assert_heading_close(heading_from_mag_xy_deg(yaw_90_fc), 270.0);

        let yaw_180_body = Quat::from_rotation_y(180.0_f32.to_radians()).inverse() * world_mag;
        let yaw_180_fc = map_bevy_body_to_fc_magnetometer(yaw_180_body);
        assert_heading_close(heading_from_mag_xy_deg(yaw_180_fc), 180.0);
    }

    #[test]
    fn sim_sensor_frame_tracks_level_and_forward_tilt() {
        let settle_pose = |rotation: Quat| {
            let ctx = CuContext::new_with_clock();
            let mut ahrs = <cu_ahrs::CuAhrs as CuTask>::new(None, ()).expect("create AHRS");
            let accel_body = rotation.inverse() * Vec3::new(0.0, -9.81, 0.0);
            let imu = ImuPayload::from_raw(map_bevy_body_to_fc_polar(accel_body), [0.0; 3], 29.0);
            let body_mag = rotation.inverse() * Vec3::from_array(WORLD_MAG_FIELD_UT);
            let magnetometer =
                MagnetometerPayload::from_raw(map_bevy_body_to_fc_magnetometer(body_mag));
            let mut pose = None;

            for sample in 1..=512_u64 {
                let tov = Tov::Time(CuTime::from(sample * 15_625_000));
                let mut imu_msg = CuMsg::new(Some(imu));
                imu_msg.tov = tov;
                let mut mag_msg = CuMsg::new(Some(magnetometer));
                mag_msg.tov = tov;
                let mut output = CuMsg::new(None);
                ahrs.process(&ctx, &(&imu_msg, &mag_msg), &mut output)
                    .expect("process simulated sensor sample");
                pose = output.payload().copied();
            }

            pose.expect("AHRS should produce a pose")
        };

        let pose = settle_pose(Quat::IDENTITY);
        assert!(
            pose.roll.get::<radian>().abs() < 0.01,
            "level simulated roll drifted to {} degrees",
            pose.roll.get::<radian>().to_degrees()
        );
        assert!(
            pose.pitch.get::<radian>().abs() < 0.01,
            "level simulated pitch drifted to {} degrees",
            pose.pitch.get::<radian>().to_degrees()
        );

        let rotation = Quat::from_rotation_x(-10.0_f32.to_radians());
        let pose = settle_pose(rotation);
        let expected = vitfly_pose(rotation);
        assert!(
            (pose.pitch.get::<radian>() - expected.pitch.get::<radian>()).abs() < 0.02,
            "10-degree forward tilt estimated as {} degrees",
            pose.pitch.get::<radian>().to_degrees()
        );
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

    #[test]
    fn sim_gnss_horizontal_position_matches_bevy_world_axes() {
        let _guard = sim_gnss_test_lock().lock().unwrap();
        let state = sim_support::sim_gnss_state();

        sim_gnss_set_vehicle_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let lat0 = f64::from_bits(state.lat_deg_bits.load(Ordering::Relaxed));
        let lon0 = f64::from_bits(state.lon_deg_bits.load(Ordering::Relaxed));

        sim_gnss_set_vehicle_state([20.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let lon_east = f64::from_bits(state.lon_deg_bits.load(Ordering::Relaxed));
        assert!(lon_east > lon0, "east movement should increase longitude");

        sim_gnss_set_vehicle_state([-20.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let lon_west = f64::from_bits(state.lon_deg_bits.load(Ordering::Relaxed));
        assert!(lon_west < lon0, "west movement should decrease longitude");

        sim_gnss_set_vehicle_state([0.0, 0.0, -20.0], [0.0, 0.0, 0.0]);
        let lat_north = f64::from_bits(state.lat_deg_bits.load(Ordering::Relaxed));
        assert!(lat_north > lat0, "north movement should increase latitude");

        sim_gnss_set_vehicle_state([0.0, 0.0, 20.0], [0.0, 0.0, 0.0]);
        let lat_south = f64::from_bits(state.lat_deg_bits.load(Ordering::Relaxed));
        assert!(lat_south < lat0, "south movement should decrease latitude");
    }

    #[test]
    fn sim_gnss_populates_ne_down_velocity_and_heading() {
        let _guard = sim_gnss_test_lock().lock().unwrap();
        let state = sim_support::sim_gnss_state();

        // World velocity +X is east, -Z is north, and +Y is up.
        sim_gnss_set_vehicle_state([0.0, 0.0, 0.0], [5.0, 2.0, -3.0]);

        let vn = f32::from_bits(state.velocity_north_mps_bits.load(Ordering::Relaxed));
        let ve = f32::from_bits(state.velocity_east_mps_bits.load(Ordering::Relaxed));
        let vd = f32::from_bits(state.velocity_down_mps_bits.load(Ordering::Relaxed));
        let gs = f32::from_bits(state.ground_speed_mps_bits.load(Ordering::Relaxed));
        let hm = f32::from_bits(state.heading_motion_deg_bits.load(Ordering::Relaxed));

        assert!((vn - 3.0).abs() < 1.0e-6, "north velocity should be +3 m/s");
        assert!((ve - 5.0).abs() < 1.0e-6, "east velocity should be +5 m/s");
        assert!(
            (vd + 2.0).abs() < 1.0e-6,
            "down velocity should be -2 m/s for upward motion"
        );
        assert!(
            (gs - libm::sqrtf(34.0)).abs() < 1.0e-6,
            "ground speed should track horizontal speed"
        );
        assert!(
            (hm - libm::atan2f(5.0, 3.0).to_degrees()).abs() < 1.0e-6,
            "heading should use north/east velocity components"
        );
    }
}
