use crate::motor_model;
use crate::world::{AppliedForce, Cart, DragState, Rod};
use avian3d::math::Vector;
use avian3d::prelude::{ConstantForce, Physics};
use bevy::app::AppExit;
use bevy::prelude::*;
use cu_ads7883_new::ADSReadingPayload;
#[cfg(feature = "bevymon")]
use cu_bevymon::MonitorModel;
use cu_rp_encoder::EncoderPayload;
#[cfg(not(target_arch = "wasm32"))]
use cu29::prelude::debug;
use cu29::prelude::{error, *};
use cu29::units::si::ratio::ratio;
#[cfg(not(target_arch = "wasm32"))]
use cu29_helpers::basic_copper_setup;
#[cfg(not(target_arch = "wasm32"))]
use std::fs;
#[cfg(not(target_arch = "wasm32"))]
use std::path::{Path, PathBuf};
#[cfg(feature = "bevymon")]
use std::sync::{Arc, Mutex};

#[derive(Resource)]
pub struct CopperSim<T: Send + Sync + 'static> {
    _runtime_state: T,
    clock: RobotClock,
    copper_app: crate::BalanceBotSim,
}

#[derive(Resource, Default)]
pub struct LastCopperTick(pub Option<u64>);

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

pub fn default_callback(step: crate::default::SimStep) -> SimOverride {
    match step {
        crate::default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
        crate::default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
        crate::default::SimStep::Motor(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

#[cfg(not(target_arch = "wasm32"))]
#[cfg_attr(feature = "bevymon", allow(dead_code))]
pub fn setup_native_copper(mut commands: Commands) {
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);
    let logger_path = "logs/balance.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }

    eprintln!(
        "[WARNING] BalanceBot sim using real RobotClock; revert when sim clock driving is fixed."
    );
    let robot_clock = RobotClock::new();
    let copper_ctx = basic_copper_setup(
        &PathBuf::from(logger_path),
        LOG_SLAB_SIZE,
        true,
        Some(robot_clock.clone()),
    )
    .expect("Failed to setup logger.");
    debug!(
        "Logger created at {path}. This is a simulation.",
        path = logger_path
    );

    let mut copper_app = crate::BalanceBotSimBuilder::new()
        .with_context(&copper_ctx)
        .with_sim_callback(&mut default_callback)
        .build()
        .expect("Failed to create runtime.");

    copper_app
        .start_all_tasks(&mut default_callback)
        .expect("Failed to start all tasks.");

    commands.insert_resource(CopperSim {
        _runtime_state: copper_ctx,
        clock: robot_clock,
        copper_app,
    });
    commands.insert_resource(LastCopperTick::default());
}

#[cfg(feature = "bevymon")]
pub fn build_bevymon_copper() -> (MonitorModel, CopperSim<LoggerRuntime>) {
    #[allow(clippy::identity_op)]
    const LOG_SLAB_SIZE: Option<usize> = Some(1 * 1024 * 1024 * 1024);
    const STRUCTURED_LOG_SECTION_SIZE: usize = 4096 * 10;

    #[cfg(not(target_arch = "wasm32"))]
    eprintln!(
        "[WARNING] BalanceBot sim using real RobotClock; revert when sim clock driving is fixed."
    );

    let clock = RobotClock::new();
    let unified_logger = build_unified_logger(LOG_SLAB_SIZE).expect("Failed to create logger.");
    let logger_runtime =
        init_logger_runtime(&clock, unified_logger.clone(), STRUCTURED_LOG_SECTION_SIZE)
            .expect("Failed to initialize Copper structured logging.");

    let mut sim_callback = default_callback;
    let mut copper_app = <crate::BalanceBotSim as CuSimApplication<
        BevyMonSectionStorage,
        BevyMonUnifiedLogger,
    >>::new(clock.clone(), unified_logger, None, &mut sim_callback)
    .expect("Failed to create runtime.");

    copper_app
        .start_all_tasks(&mut sim_callback)
        .expect("Failed to start all tasks.");

    let monitor_model = copper_app.copper_runtime_mut().monitor.model();
    (
        monitor_model,
        CopperSim {
            _runtime_state: logger_runtime,
            clock,
            copper_app,
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
    let logger_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("logs/balance.copper");
    if let Some(parent) = logger_path.parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }

    let logger = UnifiedLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_base_name(&logger_path)
        .preallocated_size(log_slab_size.unwrap_or(10 * 1024 * 1024))
        .build()
        .map_err(|err| CuError::new_with_cause("Failed to create balancebot logger", err))?;
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

#[allow(clippy::type_complexity)]
pub fn run_copper_callback<T: Send + Sync + 'static>(
    mut query_set: ParamSet<(
        Query<(&Transform, &mut ConstantForce, &mut AppliedForce), With<Cart>>,
        Query<&Transform, With<Rod>>,
    )>,
    physics_time: Res<Time<Physics>>,
    mut copper_ctx: ResMut<CopperSim<T>>,
    mut last_tick: ResMut<LastCopperTick>,
    drag_state: Res<DragState>,
    mut exit_writer: MessageWriter<AppExit>,
) {
    if query_set.p0().is_empty() {
        return;
    }

    let current_time = physics_time.elapsed().as_nanos() as u64;
    if last_tick.0 == Some(current_time) {
        return;
    }
    last_tick.0 = Some(current_time);

    let clock = copper_ctx.clock.clone();
    let mut sim_callback = move |step: crate::default::SimStep| -> SimOverride {
        match step {
            crate::default::SimStep::Balpos(CuTaskCallbackState::Process(_, output)) => {
                let bindings = query_set.p1();
                let rod_transform = bindings.single().expect("Failed to get rod transform");

                let (_roll, _pitch, yaw) = rod_transform.rotation.to_euler(EulerRot::YXZ);

                let mut angle_radians = yaw - std::f32::consts::FRAC_PI_2;
                if angle_radians < 0.0 {
                    angle_radians += 2.0 * std::f32::consts::PI;
                }

                let analog_value = (angle_radians / (2.0 * std::f32::consts::PI) * 4096.0) as u16;
                let now = clock.now();
                output.set_payload(ADSReadingPayload { analog_value });
                output.tov = now.into();
                output.metadata.process_time.start = now.into();
                output.metadata.process_time.end = now.into();
                SimOverride::ExecutedBySim
            }
            crate::default::SimStep::Balpos(_) => SimOverride::ExecutedBySim,
            crate::default::SimStep::Railpos(CuTaskCallbackState::Process(_, output)) => {
                let mut bindings = query_set.p0();
                let (cart_transform, _, _) =
                    bindings.single_mut().expect("Failed to get cart transform");
                let ticks = (cart_transform.translation.x * 2000.0) as i32;
                let now = clock.now();
                output.set_payload(EncoderPayload { ticks });
                output.tov = now.into();
                output.metadata.process_time.start = now.into();
                output.metadata.process_time.end = now.into();
                SimOverride::ExecutedBySim
            }
            crate::default::SimStep::Railpos(_) => SimOverride::ExecutedBySim,
            crate::default::SimStep::Motor(CuTaskCallbackState::Process(input, output)) => {
                let mut bindings = query_set.p0();
                let (_, mut cart_force, mut applied_force) =
                    bindings.single_mut().expect("Failed to get cart force");
                let maybe_motor_actuation = input.payload();
                let override_motor = drag_state.override_motor;
                let now = clock.now();
                if override_motor {
                    if let Some(motor_actuation) = maybe_motor_actuation
                        && !motor_actuation.power.get::<ratio>().is_nan()
                    {
                        let total_mass = motor_model::total_mass_kg();
                        let force_magnitude = motor_model::force_from_power(
                            motor_actuation.power.get::<ratio>(),
                            total_mass,
                        );
                        output
                            .metadata
                            .set_status(format!("Applied force: {force_magnitude}"));
                    }
                    output.metadata.process_time.start = now.into();
                    output.metadata.process_time.end = now.into();
                    return SimOverride::ExecutedBySim;
                }
                if let Some(motor_actuation) = maybe_motor_actuation {
                    if motor_actuation.power.get::<ratio>().is_nan() {
                        cart_force.0 = Vector::ZERO;
                        output.metadata.process_time.start = now.into();
                        output.metadata.process_time.end = now.into();
                        return SimOverride::ExecutedBySim;
                    }
                    let total_mass = motor_model::total_mass_kg();
                    let force_magnitude = motor_model::force_from_power(
                        motor_actuation.power.get::<ratio>(),
                        total_mass,
                    );
                    let new_force = Vector::new(force_magnitude, 0.0, 0.0);
                    cart_force.0 = new_force;
                    applied_force.0 = new_force;
                    output
                        .metadata
                        .set_status(format!("Applied force: {force_magnitude}"));
                    output.metadata.process_time.start = now.into();
                    output.metadata.process_time.end = now.into();
                    SimOverride::ExecutedBySim
                } else {
                    cart_force.0 = Vector::ZERO;
                    output.metadata.process_time.start = now.into();
                    output.metadata.process_time.end = now.into();
                    SimOverride::Errored("Safety Mode.".into())
                }
            }
            crate::default::SimStep::Motor(_) => SimOverride::ExecutedBySim,
            _ => SimOverride::ExecuteByRuntime,
        }
    };
    if let Err(err) = copper_ctx.copper_app.run_one_iteration(&mut sim_callback) {
        error!("Simulation stopped: {err}");
        eprintln!("Simulation stopped: {err}");
        exit_writer.write(AppExit::Success);
    }
}

pub fn stop_copper_on_exit<T: Send + Sync + 'static>(
    mut exit_events: MessageReader<AppExit>,
    mut copper_ctx: ResMut<CopperSim<T>>,
) {
    if exit_events.read().next().is_some() {
        copper_ctx
            .copper_app
            .stop_all_tasks(&mut default_callback)
            .expect("Failed to stop all tasks.");
        let _ = copper_ctx.copper_app.log_shutdown_completed();
    }
}
