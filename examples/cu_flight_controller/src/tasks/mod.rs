#![allow(dead_code)]

use crate::messages::{
    BatteryVoltage, BodyCommand, BodyRateSetpoint, ControlInputs, FlightMode, GeographicHeading,
};
use cu_ahrs::AhrsPose;
use cu_bdshot::EscCommand;
use cu_crsf::messages::RcChannelsPayload;
use cu_pid::{PIDControlOutputPayload, PIDController};
use cu_sensor_payloads::ImuPayload;
use cu29::prelude::*;
use cu29::units::si::angle::{degree, radian};
use cu29::units::si::angular_velocity::{degree_per_second, radian_per_second};
use cu29::units::si::f32::{Angle, AngularVelocity, Ratio};
use cu29::units::si::ratio::ratio;
use cu29::units::si::thermodynamic_temperature::degree_celsius;

const LOG_PERIOD_MS: u64 = 500;
const HEAP_LOG_PERIOD_MS: u64 = 500;
const VTX_HEARTBEAT_PERIOD_MS: u64 = 1000;
const VTX_DRAW_PERIOD_MS: u64 = 250;
// Matches Betaflight ARMING_DISABLE_FLAGS_COUNT (log2(ARM_SWITCH) + 1).
const MSP_ARMING_DISABLE_FLAGS_COUNT: u8 = 29;
const VTX_WATERMARK_LINES: [&str; 3] = [" /\\_/\\ ", "( O O )", " > ^ <  [CU29]"];
const VTX_CELL_DIVISOR: u16 = 4;
const BATTERY_VREF_MV_DEFAULT: u32 = 3300;
const BATTERY_VBAT_SCALE_DEFAULT: u32 = 210;
const BATTERY_VBAT_RES_DIV_VAL_DEFAULT: u32 = 10;
const BATTERY_VBAT_RES_DIV_MULT_DEFAULT: u32 = 1;
const BATTERY_MIN_CELL_CENTIVOLTS_DEFAULT: u16 = 330;
const BATTERY_MAX_CELL_CENTIVOLTS_DEFAULT: u16 = 420;
const BATTERY_WARN_CELL_CENTIVOLTS_DEFAULT: u16 = 350;
const BATTERY_VOLTAGE_METER_SOURCE_ADC: u8 = 1;
const BATTERY_CURRENT_METER_SOURCE_NONE: u8 = 0;
const MSP_VOLTAGE_METER_ID_BATTERY_1: u8 = 10;
const MSP_VOLTAGE_METER_SENSOR_TYPE_ADC_RES_DIV: u8 = 0;
const MSP_VOLTAGE_METER_ADC_SUBFRAME_LEN: u8 = 5;
const MSP_API_PROTOCOL_VERSION: u8 = 1;
const MSP_API_VERSION_MAJOR: u8 = 1;
const MSP_API_VERSION_MINOR: u8 = 47;
const MSP_FC_VERSION_MAJOR: u8 = 4;
const MSP_FC_VERSION_MINOR: u8 = 4;
const MSP_FC_VERSION_PATCH: u8 = 0;

pub mod activity_led;
pub mod battery;
pub mod flow_msp;
pub mod gnss;
pub mod monitor;
pub mod vtx;

struct LogRateLimiter {
    last: OptionCuTime,
    period_ms: u64,
}

impl LogRateLimiter {
    const fn new(period_ms: u64) -> Self {
        Self {
            last: OptionCuTime::none(),
            period_ms,
        }
    }

    fn should_log(&mut self, now: CuTime) -> bool {
        if self.last.is_none() {
            self.last = now.into();
            return true;
        }
        if now - self.last.unwrap() >= CuDuration::from_millis(self.period_ms) {
            self.last = now.into();
            return true;
        }
        false
    }
}

#[macro_export]
macro_rules! debug_rl {
    ($state:expr, $now:expr, { $($body:tt)* }) => {{
        let mut state = $state.lock();
        if state.should_log($now) {
            $($body)*
        }
    }};
    ($state:expr, $now:expr, $($arg:tt)+) => {{
        debug_rl!($state, $now, { debug!($($arg)+); });
    }};
}

macro_rules! status_if_not_firmware {
    ($metadata:expr, $status:expr) => {{
        #[cfg(not(feature = "firmware"))]
        {
            $metadata.set_status($status);
        }
    }};
}

static LOG_CTRL: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new(LOG_PERIOD_MS));
static LOG_TELEMETRY: spin::Mutex<LogRateLimiter> =
    spin::Mutex::new(LogRateLimiter::new(LOG_PERIOD_MS));
static LOG_IMU: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new(LOG_PERIOD_MS));
static LOG_RC: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new(LOG_PERIOD_MS));
static LOG_RATE: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new(LOG_PERIOD_MS));
static LOG_MOTORS: spin::Mutex<LogRateLimiter> =
    spin::Mutex::new(LogRateLimiter::new(LOG_PERIOD_MS));

#[derive(Debug, Clone, Copy, PartialEq, Eq, Reflect)]
enum StatusLabel {
    Disarmed,
    Calibrating,
    Angle,
    Air,
    Position,
}

impl StatusLabel {
    const fn as_str(self) -> &'static str {
        match self {
            StatusLabel::Disarmed => " XXX ",
            StatusLabel::Calibrating => " CAL ",
            StatusLabel::Angle => "ANGLE",
            StatusLabel::Air => " AIR ",
            StatusLabel::Position => " POS ",
        }
    }
}

fn clamp_u8(value: u16) -> u8 {
    value.min(u16::from(u8::MAX)) as u8
}

fn estimate_cell_count(voltage_centi: u16) -> Option<u8> {
    if voltage_centi == 0 {
        return None;
    }
    let cells = u32::from(voltage_centi).div_ceil(420);
    Some(clamp_u8(cells.min(u32::from(u8::MAX)) as u16))
}

#[derive(Reflect)]
pub struct ImuLogger {
    last_tov: Option<CuTime>,
}

impl Freezable for ImuLogger {}

impl CuSinkTask for ImuLogger {
    type Input<'m> = CuMsg<ImuPayload>;
    type Resources<'r> = ();

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { last_tov: None })
    }

    fn process<'i>(&mut self, _ctx: &CuContext, input: &Self::Input<'i>) -> CuResult<()> {
        if let Some(payload) = input.payload() {
            let tov_time = expect_tov_time(input.tov)?;
            debug_rl!(&LOG_IMU, tov_time, {
                let tov_kind = 1_u8;
                let tov_start_ns = tov_time.as_nanos();
                let tov_end_ns = tov_time.as_nanos();
                let dt_us = match self.last_tov {
                    Some(prev) => (tov_time - prev).as_micros(),
                    None => 0,
                };
                self.last_tov = Some(tov_time);
                let gx_dps = payload.gyro_x.get::<degree_per_second>();
                let gy_dps = payload.gyro_y.get::<degree_per_second>();
                let gz_dps = payload.gyro_z.get::<degree_per_second>();
                let temp_c = payload.temperature.get::<degree_celsius>();
                debug!(
                    "imu ax={} m.s⁻² ay={} m.s⁻² az={} m.s⁻² gx={} deg.s⁻¹ gy={} deg.s⁻¹ gz={} deg.s⁻¹ t={} °C tov_kind={} tov_start_ns={} tov_end_ns={} tov_dt_us={}",
                    payload.accel_x.value,
                    payload.accel_y.value,
                    payload.accel_z.value,
                    gx_dps,
                    gy_dps,
                    gz_dps,
                    temp_c,
                    tov_kind,
                    tov_start_ns,
                    tov_end_ns,
                    dt_us
                );
            });
        }
        Ok(())
    }
}

#[cfg(feature = "firmware")]
pub type Bmi088Source = cu_bmi088::Bmi088Source<
    cu_micoairh743::Bmi088Spi,
    cu_micoairh743::Bmi088AccCs,
    cu_micoairh743::Bmi088GyrCs,
    cu_micoairh743::Bmi088Delay,
>;
#[cfg(feature = "firmware")]
pub type Dps310Source = cu_dps310::Dps310Source<cu_micoairh743::Dps310I2c>;
#[cfg(feature = "firmware")]
pub type Ist8310Source = cu_ist8310::Ist8310Source<cu_micoairh743::Ist8310I2c>;

#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
pub type Bmi088Source = crate::sim_support::SimBmi088Source;
#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
pub type Dps310Source = crate::sim_support::SimDps310Source;
#[cfg(all(any(feature = "sim", feature = "bevymon"), not(feature = "firmware")))]
pub type Ist8310Source = crate::sim_support::SimIst8310Source;

#[derive(Reflect)]
pub struct MagneticTrueHeading {
    declination_deg: f32,
}

impl Freezable for MagneticTrueHeading {}

impl CuTask for MagneticTrueHeading {
    type Input<'m> = CuMsg<AhrsPose>;
    type Output<'m> = CuMsg<GeographicHeading>;
    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            declination_deg: cfg_f32(config, "declination_deg", 0.0)?,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let tov_time = expect_tov_time(input.tov)?;
        output.tov = Tov::Time(tov_time);

        let Some(pose) = input.payload() else {
            status_if_not_firmware!(output.metadata, "hdg none");
            output.clear_payload();
            return Ok(());
        };

        let yaw_rad = pose.yaw.get::<radian>();
        if !yaw_rad.is_finite() || !self.declination_deg.is_finite() {
            status_if_not_firmware!(output.metadata, "hdg nan");
            output.clear_payload();
            return Ok(());
        }

        let Some(heading_deg) = heading_from_yaw_deg(yaw_rad.to_degrees(), self.declination_deg)
        else {
            status_if_not_firmware!(output.metadata, "hdg bad");
            output.clear_payload();
            return Ok(());
        };

        output.set_payload(GeographicHeading {
            heading: Angle::new::<degree>(heading_deg),
        });
        status_if_not_firmware!(
            output.metadata,
            format!("hdg {}", heading_deg.round() as i16)
        );
        Ok(())
    }
}

impl Freezable for FlightMode {}
impl Freezable for ControlInputs {}
impl Freezable for BodyRateSetpoint {}
impl Freezable for BodyCommand {}
impl Freezable for BatteryVoltage {}
impl Freezable for GeographicHeading {}

#[derive(Reflect)]
pub struct RcMapper {
    rc_min: u16,
    rc_mid: u16,
    rc_max: u16,
    deadband: u16,
    arm_channel: usize,
    arm_min: u16,
    arm_max: u16,
    mode_channel: Option<usize>,
    mode_low_max: Option<u16>,
    mode_mid_max: Option<u16>,
}

impl Freezable for RcMapper {}

impl CuTask for RcMapper {
    type Input<'m> = CuMsg<RcChannelsPayload>;
    type Output<'m> = CuMsg<ControlInputs>;
    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let arm_cfg = match config {
            Some(cfg) => cfg.get::<u32>("arm_channel")?,
            None => None,
        };
        let mut arm_channel = arm_cfg.map(|v| v as usize).unwrap_or(3);
        if arm_channel > 15 {
            warning!(
                "rc mapper arm_channel {} out of range, clamping to 15",
                arm_channel
            );
            arm_channel = 15;
        }
        let arm_min = cfg_u16(config, "arm_min", 1700)?;
        let arm_max = cfg_u16(config, "arm_max", 1811)?;
        let mode_cfg = match config {
            Some(cfg) => cfg.get::<u32>("mode_channel")?.map(|v| v as usize),
            None => None,
        };
        let mode_low_max = match config {
            Some(cfg) => cfg
                .get::<u32>("mode_low_max")?
                .map(|v| v.min(u16::MAX as u32) as u16),
            None => None,
        };
        let mode_mid_max = match config {
            Some(cfg) => cfg
                .get::<u32>("mode_mid_max")?
                .map(|v| v.min(u16::MAX as u32) as u16),
            None => None,
        };

        info!(
            "rc mapper cfg arm_channel={:?} arm_min={} arm_max={} mode_channel={:?} mode_low_max={} mode_mid_max={}",
            arm_cfg,
            arm_min,
            arm_max,
            mode_cfg,
            mode_low_max.unwrap_or(0),
            mode_mid_max.unwrap_or(0)
        );

        Ok(Self {
            rc_min: cfg_u16(config, "rc_min", 172)?,
            rc_mid: cfg_u16(config, "rc_mid", 992)?,
            rc_max: cfg_u16(config, "rc_max", 1811)?,
            deadband: cfg_u16(config, "rc_deadband", 0)?,
            arm_channel,
            arm_min,
            arm_max,
            mode_channel: mode_cfg,
            mode_low_max,
            mode_mid_max,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        inputs: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let tov_time = expect_tov_time(inputs.tov)?;
        let Some(rc) = inputs.payload() else {
            output.tov = Tov::Time(tov_time);
            status_if_not_firmware!(output.metadata, "rc none");
            output.clear_payload();
            return Ok(());
        };

        let channels = &rc.inner().0;
        let roll = normalize_axis(channels.first().copied().unwrap_or(self.rc_mid), self);
        let pitch = -normalize_axis(channels.get(1).copied().unwrap_or(self.rc_mid), self);
        let throttle = normalize_throttle(
            channels.get(2).copied().unwrap_or(self.rc_min),
            self.rc_min,
            self.rc_max,
        );
        let yaw = normalize_axis(channels.get(3).copied().unwrap_or(self.rc_mid), self);

        let arm_value = channels.get(self.arm_channel).copied().unwrap_or(0);
        let armed = arm_value >= self.arm_min && arm_value <= self.arm_max;

        let mode = match self.mode_channel.and_then(|idx| channels.get(idx).copied()) {
            Some(value) => {
                let (low_max, mid_max) = match (self.mode_low_max, self.mode_mid_max) {
                    (Some(low), Some(mid)) => (low, mid),
                    _ => {
                        let low = ((self.rc_min as u32 + self.rc_mid as u32) / 2) as u16;
                        let mid = ((self.rc_mid as u32 + self.rc_max as u32) / 2) as u16;
                        (low, mid)
                    }
                };
                if value <= low_max {
                    FlightMode::Acro
                } else if value <= mid_max {
                    FlightMode::Angle
                } else {
                    FlightMode::PositionHold
                }
            }
            None => FlightMode::Angle,
        };

        debug_rl!(&LOG_RC, tov_time, {
            let mode_channel = self.mode_channel.unwrap_or(0);
            let mode_value = self
                .mode_channel
                .and_then(|idx| channels.get(idx).copied())
                .unwrap_or(0);
            debug!(
                "rc ch0={} ch1={} ch2={} ch3={} ch4={} ch5={} ch6={} ch7={} ch8={} ch9={} ch10={} ch11={} ch12={} ch13={} ch14={} ch15={} arm_ch0={} arm_raw={} armed={} mode_ch0={} mode_raw={} mode={} mode_low_max={} mode_mid_max={}",
                channels[0],
                channels[1],
                channels[2],
                channels[3],
                channels[4],
                channels[5],
                channels[6],
                channels[7],
                channels[8],
                channels[9],
                channels[10],
                channels[11],
                channels[12],
                channels[13],
                channels[14],
                channels[15],
                self.arm_channel,
                arm_value,
                armed,
                mode_channel,
                mode_value,
                mode_label(mode),
                self.mode_low_max.unwrap_or(0),
                self.mode_mid_max.unwrap_or(0)
            );
        });

        output.tov = Tov::Time(tov_time);
        output.set_payload(ControlInputs {
            roll: Ratio::new::<ratio>(roll),
            pitch: Ratio::new::<ratio>(pitch),
            yaw: Ratio::new::<ratio>(yaw),
            throttle: Ratio::new::<ratio>(throttle),
            armed,
            mode,
        });
        status_if_not_firmware!(
            output.metadata,
            format!(
                "{} {} T{}%",
                if armed { "ARM" } else { "DIS" },
                mode_tag(mode),
                throttle_percent(throttle)
            )
        );
        Ok(())
    }
}

#[derive(Reflect)]
pub struct ImuCalibrator {
    bias: [f32; 3],
    sum: [f32; 3],
    samples: u32,
    required_samples: u32,
    last_armed: bool,
    calibrating: bool,
}

impl Freezable for ImuCalibrator {}

impl CuTask for ImuCalibrator {
    type Input<'m> = input_msg!('m, ImuPayload, ControlInputs);
    type Output<'m> = CuMsg<ImuPayload>;
    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cal_ms = cfg_u32(config, "cal_ms", 3000)?;
        let sample_period_ms = cfg_u32(config, "sample_period_ms", 10)?;
        let default_samples = (cal_ms / sample_period_ms.max(1)).max(1);
        let required_samples = cfg_u32(config, "cal_samples", default_samples)?;

        Ok(Self {
            bias: [0.0; 3],
            sum: [0.0; 3],
            samples: 0,
            required_samples,
            last_armed: false,
            calibrating: false,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (imu_msg, ctrl_msg) = *input;
        let imu_tov = expect_tov_time(imu_msg.tov)?;
        let _ = expect_tov_time(ctrl_msg.tov)?;
        let Some(imu) = imu_msg.payload() else {
            output.tov = Tov::Time(imu_tov);
            status_if_not_firmware!(output.metadata, "imu none");
            output.clear_payload();
            return Ok(());
        };
        let armed = ctrl_msg.payload().map(|c| c.armed).unwrap_or(false);

        if !armed {
            self.calibrating = false;
            self.sum = [0.0; 3];
            self.samples = 0;
            self.last_armed = false;
            output.tov = Tov::Time(imu_tov);
            status_if_not_firmware!(output.metadata, "disarm");
            output.set_payload(*imu);
            return Ok(());
        }

        if armed && !self.last_armed {
            self.calibrating = true;
            self.sum = [0.0; 3];
            self.samples = 0;
        }
        self.last_armed = armed;

        if self.calibrating {
            self.sum[0] += imu.gyro_x.value;
            self.sum[1] += imu.gyro_y.value;
            self.sum[2] += imu.gyro_z.value;
            self.samples = self.samples.saturating_add(1);

            if self.samples >= self.required_samples {
                let inv = 1.0 / (self.samples as f32);
                self.bias = [self.sum[0] * inv, self.sum[1] * inv, self.sum[2] * inv];
                self.calibrating = false;
                let bias_deg = [
                    self.bias[0].to_degrees(),
                    self.bias[1].to_degrees(),
                    self.bias[2].to_degrees(),
                ];
                info!(
                    "imu gyro bias x={} deg.s⁻¹ y={} deg.s⁻¹ z={} deg.s⁻¹",
                    bias_deg[0], bias_deg[1], bias_deg[2]
                );
            }

            output.tov = Tov::Time(imu_tov);
            output.clear_payload();
            #[cfg(not(feature = "firmware"))]
            {
                let progress =
                    ((self.samples.saturating_mul(100)) / self.required_samples.max(1)).min(100);
                status_if_not_firmware!(output.metadata, format!("cal {}%", progress));
            }
            return Ok(());
        }

        let accel_mps2 = [imu.accel_x.value, imu.accel_y.value, imu.accel_z.value];
        let gyro_rad = [
            imu.gyro_x.value - self.bias[0],
            imu.gyro_y.value - self.bias[1],
            imu.gyro_z.value - self.bias[2],
        ];
        let temp_c = imu.temperature.get::<degree_celsius>();

        output.tov = Tov::Time(imu_tov);
        status_if_not_firmware!(output.metadata, "ok");
        output.set_payload(ImuPayload::from_raw(accel_mps2, gyro_rad, temp_c));
        Ok(())
    }
}

struct AxisPid {
    pid: PIDController,
    initialized: bool,
}

impl AxisPid {
    fn new(pid: PIDController) -> Self {
        Self {
            pid,
            initialized: false,
        }
    }

    fn reset(&mut self) {
        self.pid.reset();
        self.initialized = false;
    }

    fn reset_integral(&mut self) {
        self.pid.reset_integral();
    }

    fn update(&mut self, measurement: f32, dt: CuDuration) -> Option<PIDControlOutputPayload> {
        // cu_pid expects dt in microseconds (it divides by 1e6), so scale nanoseconds down.
        let dt_pid = CuDuration::from_nanos((dt.as_nanos() / 1_000).max(1));
        if !self.initialized {
            self.pid.init_measurement(measurement);
            self.initialized = true;
            return None;
        }
        Some(self.pid.next_control_output(measurement, dt_pid))
    }
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct AttitudeController {
    #[reflect(ignore)]
    roll_pid: AxisPid,
    #[reflect(ignore)]
    pitch_pid: AxisPid,
    angle_limit_rad: f32,
    rate_limit_rad: f32,
    acro_rate_rad: f32,
    acro_expo: f32,
    dt_fallback: CuDuration,
    last_time: Option<CuTime>,
    #[reflect(ignore)]
    last_mode: FlightMode,
}

impl Freezable for AttitudeController {}

impl CuTask for AttitudeController {
    type Input<'m> = input_msg!('m, AhrsPose, ControlInputs);
    type Output<'m> = CuMsg<BodyRateSetpoint>;
    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let angle_limit_rad = cfg_f32(config, "angle_limit_deg", 25.0)?.to_radians();
        let rate_limit_rad = cfg_f32(config, "rate_limit_dps", 180.0)?.to_radians();
        let acro_rate_rad = cfg_f32(config, "acro_rate_dps", 360.0)?.to_radians();
        let acro_expo = normalize_expo(cfg_f32(config, "acro_expo", 0.0)?);
        let kp = cfg_f32(config, "kp", 4.0)?;
        let ki = cfg_f32(config, "ki", 0.0)?;
        let kd = cfg_f32(config, "kd", 0.0)?;
        let dt_fallback = CuDuration::from_millis(cfg_u32(config, "dt_ms", 10)? as u64);

        let roll_pid = PIDController::new(
            kp,
            ki,
            kd,
            0.0,
            rate_limit_rad,
            rate_limit_rad,
            rate_limit_rad,
            rate_limit_rad,
            CuDuration::default(),
        );
        let pitch_pid = PIDController::new(
            kp,
            ki,
            kd,
            0.0,
            rate_limit_rad,
            rate_limit_rad,
            rate_limit_rad,
            rate_limit_rad,
            CuDuration::default(),
        );

        Ok(Self {
            roll_pid: AxisPid::new(roll_pid),
            pitch_pid: AxisPid::new(pitch_pid),
            angle_limit_rad,
            rate_limit_rad,
            acro_rate_rad,
            acro_expo,
            dt_fallback,
            last_time: None,
            last_mode: FlightMode::Angle,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (pose_msg, ctrl_msg) = *input;
        let pose_tov = expect_tov_time(pose_msg.tov)?;
        let _ = expect_tov_time(ctrl_msg.tov)?;
        let output_tov = Tov::Time(pose_tov);
        let Some(pose) = pose_msg.payload() else {
            output.tov = output_tov;
            status_if_not_firmware!(output.metadata, "att no-pose");
            output.clear_payload();
            return Ok(());
        };
        let Some(ctrl) = ctrl_msg.payload() else {
            output.tov = output_tov;
            status_if_not_firmware!(output.metadata, "att no-ctrl");
            output.clear_payload();
            return Ok(());
        };

        if !ctrl.armed {
            self.roll_pid.reset();
            self.pitch_pid.reset();
            self.last_mode = ctrl.mode;
            output.tov = output_tov;
            status_if_not_firmware!(output.metadata, "disarm");
            output.set_payload(BodyRateSetpoint::default());
            return Ok(());
        }

        if ctrl.mode != self.last_mode {
            self.roll_pid.reset();
            self.pitch_pid.reset();
            self.last_mode = ctrl.mode;
        }

        let now = ctx.now();
        let dt = dt_or_fallback(&mut self.last_time, now, self.dt_fallback);

        let (roll_rate, pitch_rate) =
            if matches!(ctrl.mode, FlightMode::Angle | FlightMode::PositionHold) {
                let target_roll = (ctrl.roll.get::<ratio>() * self.angle_limit_rad)
                    .clamp(-self.angle_limit_rad, self.angle_limit_rad);
                let target_pitch = (ctrl.pitch.get::<ratio>() * self.angle_limit_rad)
                    .clamp(-self.angle_limit_rad, self.angle_limit_rad);
                let roll_measure = pose.roll.get::<radian>() - target_roll;
                let pitch_measure = pose.pitch.get::<radian>() - target_pitch;
                let roll_out = self.roll_pid.update(roll_measure, dt).unwrap_or_default();
                let pitch_out = self.pitch_pid.update(pitch_measure, dt).unwrap_or_default();
                let roll_rate = roll_out.output;
                let pitch_rate = pitch_out.output;
                (
                    roll_rate.clamp(-self.rate_limit_rad, self.rate_limit_rad),
                    pitch_rate.clamp(-self.rate_limit_rad, self.rate_limit_rad),
                )
            } else {
                self.roll_pid.reset();
                self.pitch_pid.reset();
                let roll_cmd = apply_expo(ctrl.roll.get::<ratio>(), self.acro_expo);
                let pitch_cmd = apply_expo(ctrl.pitch.get::<ratio>(), self.acro_expo);
                (
                    (roll_cmd * self.acro_rate_rad).clamp(-self.acro_rate_rad, self.acro_rate_rad),
                    (pitch_cmd * self.acro_rate_rad).clamp(-self.acro_rate_rad, self.acro_rate_rad),
                )
            };

        let yaw_cmd = apply_expo(ctrl.yaw.get::<ratio>(), self.acro_expo);
        let yaw_rate =
            (yaw_cmd * self.acro_rate_rad).clamp(-self.acro_rate_rad, self.acro_rate_rad);

        output.tov = output_tov;
        status_if_not_firmware!(
            output.metadata,
            format!(
                "{} r{} p{} y{}",
                mode_tag(ctrl.mode),
                roll_rate.to_degrees().round() as i16,
                pitch_rate.to_degrees().round() as i16,
                yaw_rate.to_degrees().round() as i16
            )
        );
        output.set_payload(BodyRateSetpoint {
            roll: AngularVelocity::new::<radian_per_second>(roll_rate),
            pitch: AngularVelocity::new::<radian_per_second>(pitch_rate),
            yaw: AngularVelocity::new::<radian_per_second>(yaw_rate),
        });
        Ok(())
    }
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct RateController {
    #[reflect(ignore)]
    roll_pid: AxisPid,
    #[reflect(ignore)]
    pitch_pid: AxisPid,
    #[reflect(ignore)]
    yaw_pid: AxisPid,
    output_limit: f32,
    dt_fallback: CuDuration,
    i_throttle_min: f32,
    airmode_enabled: bool,
    airmode_start_throttle: f32,
    airmode_active: bool,
    last_time: Option<CuTime>,
}

impl Freezable for RateController {}

impl CuTask for RateController {
    type Input<'m> = input_msg!('m, BodyRateSetpoint, ImuPayload, ControlInputs);
    type Output<'m> = CuMsg<BodyCommand>;
    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let kp = cfg_f32(config, "kp", 0.15)?;
        let ki = cfg_f32(config, "ki", 0.0)?;
        let kd = cfg_f32(config, "kd", 0.0)?;
        let kp_yaw = cfg_f32(config, "kp_yaw", kp)?;
        let ki_yaw = cfg_f32(config, "ki_yaw", ki)?;
        let kd_yaw = cfg_f32(config, "kd_yaw", kd)?;
        let output_limit = cfg_f32(config, "output_limit", 1.0)?;
        let dt_fallback = CuDuration::from_millis(cfg_u32(config, "dt_ms", 10)? as u64);
        let i_throttle_min = cfg_f32(config, "i_throttle_min", 0.05)?;
        let airmode_enabled = cfg_bool(config, "airmode", false)?;
        let airmode_start_throttle =
            normalize_percent(cfg_f32(config, "airmode_start_throttle_percent", 25.0)?);

        let pid = |p: f32, i: f32, d: f32| {
            PIDController::new(
                p,
                i,
                d,
                0.0,
                output_limit,
                output_limit,
                output_limit,
                output_limit,
                CuDuration::default(),
            )
        };

        Ok(Self {
            roll_pid: AxisPid::new(pid(kp, ki, kd)),
            pitch_pid: AxisPid::new(pid(kp, ki, kd)),
            yaw_pid: AxisPid::new(pid(kp_yaw, ki_yaw, kd_yaw)),
            output_limit,
            dt_fallback,
            i_throttle_min,
            airmode_enabled,
            airmode_start_throttle,
            airmode_active: false,
            last_time: None,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (setpoint_msg, imu_msg, ctrl_msg) = *input;
        let _ = expect_tov_time(setpoint_msg.tov)?;
        let imu_tov = expect_tov_time(imu_msg.tov)?;
        let ctrl_tov = expect_tov_time(ctrl_msg.tov)?;
        let output_tov = Tov::Time(imu_tov);
        let Some(setpoint) = setpoint_msg.payload() else {
            output.tov = output_tov;
            status_if_not_firmware!(output.metadata, "rate no-sp");
            output.clear_payload();
            return Ok(());
        };
        let Some(imu) = imu_msg.payload() else {
            output.tov = output_tov;
            status_if_not_firmware!(output.metadata, "rate no-imu");
            output.clear_payload();
            return Ok(());
        };
        let ctrl = ctrl_msg.payload();
        let armed = ctrl.map(|c| c.armed).unwrap_or(false);
        let throttle = ctrl.map(|c| c.throttle.get::<ratio>()).unwrap_or(0.0);

        if !armed {
            self.roll_pid.reset();
            self.pitch_pid.reset();
            self.yaw_pid.reset();
            self.airmode_active = false;
            output.tov = output_tov;
            status_if_not_firmware!(output.metadata, "disarm");
            output.set_payload(BodyCommand::default());
            return Ok(());
        }

        // Latch airmode active once throttle is raised (betaflight behavior)
        if self.airmode_enabled && throttle >= self.airmode_start_throttle {
            self.airmode_active = true;
        } else if !self.airmode_enabled {
            self.airmode_active = false;
        }
        // Note: airmode_active stays true once activated until disarm

        let now = ctx.now();
        let dt = dt_or_fallback(&mut self.last_time, now, self.dt_fallback);

        if self.airmode_enabled && !self.airmode_active {
            self.roll_pid.reset();
            self.pitch_pid.reset();
            self.yaw_pid.reset();
            output.tov = output_tov;
            status_if_not_firmware!(
                output.metadata,
                format!("air wait t{}%", throttle_percent(throttle))
            );
            output.set_payload(BodyCommand::default());
            return Ok(());
        }

        let roll_measure = imu.gyro_x.value - setpoint.roll.get::<radian_per_second>();
        let pitch_measure = imu.gyro_y.value - setpoint.pitch.get::<radian_per_second>();
        let yaw_measure = imu.gyro_z.value - setpoint.yaw.get::<radian_per_second>();

        let roll_out = self.roll_pid.update(roll_measure, dt).unwrap_or_default();
        let pitch_out = self.pitch_pid.update(pitch_measure, dt).unwrap_or_default();
        let yaw_out = self.yaw_pid.update(yaw_measure, dt).unwrap_or_default();
        let roll_cmd = roll_out.output;
        let pitch_cmd = pitch_out.output;
        let yaw_cmd = yaw_out.output;

        debug_rl!(&LOG_RATE, ctrl_tov, {
            let sp_roll = setpoint.roll.get::<radian_per_second>().to_degrees();
            let sp_pitch = setpoint.pitch.get::<radian_per_second>().to_degrees();
            let sp_yaw = setpoint.yaw.get::<radian_per_second>().to_degrees();
            let gyro_roll = imu.gyro_x.value.to_degrees();
            let gyro_pitch = imu.gyro_y.value.to_degrees();
            let gyro_yaw = imu.gyro_z.value.to_degrees();
            info!(
                "rate_pid dt_us={} sp_r={} gyro_r={} out_r={} p_r={} i_r={} d_r={}",
                dt.as_micros(),
                sp_roll,
                gyro_roll,
                roll_out.output,
                roll_out.p,
                roll_out.i,
                roll_out.d
            );
            info!(
                "rate_pid sp_p={} gyro_p={} out_p={} p_p={} i_p={} d_p={}",
                sp_pitch, gyro_pitch, pitch_out.output, pitch_out.p, pitch_out.i, pitch_out.d
            );
            info!(
                "rate_pid sp_y={} gyro_y={} out_y={} p_y={} i_y={} d_y={}",
                sp_yaw, gyro_yaw, yaw_out.output, yaw_out.p, yaw_out.i, yaw_out.d
            );
        });

        if throttle < self.i_throttle_min && !self.airmode_active {
            self.roll_pid.reset_integral();
            self.pitch_pid.reset_integral();
            self.yaw_pid.reset_integral();
        }

        output.tov = output_tov;
        status_if_not_firmware!(
            output.metadata,
            format!(
                "t{} r{} p{} y{}",
                throttle_percent(throttle),
                (roll_cmd * 100.0).round() as i16,
                (pitch_cmd * 100.0).round() as i16,
                (yaw_cmd * 100.0).round() as i16
            )
        );
        output.set_payload(BodyCommand {
            roll: Ratio::new::<ratio>(roll_cmd.clamp(-self.output_limit, self.output_limit)),
            pitch: Ratio::new::<ratio>(pitch_cmd.clamp(-self.output_limit, self.output_limit)),
            yaw: Ratio::new::<ratio>(yaw_cmd.clamp(-self.output_limit, self.output_limit)),
        });
        Ok(())
    }
}

const QUADX_MIX: [(f32, f32, f32); 4] = [
    (-1.0, -1.0, -1.0), // rear right
    (-1.0, 1.0, 1.0),   // front right
    (1.0, -1.0, 1.0),   // rear left
    (1.0, 1.0, -1.0),   // front left
];
const DSHOT_MIN_ARM_CMD: u16 = 100;

struct MotorLogState {
    values: [u16; 4],
}

impl MotorLogState {
    const fn new() -> Self {
        Self { values: [0; 4] }
    }
}

static MOTOR_LOG: spin::Mutex<MotorLogState> = spin::Mutex::new(MotorLogState::new());

#[derive(Reflect)]
pub struct QuadXMixer {
    motor_index: usize,
    props_out: bool,
    airmode_idle: f32, // Motor idle value when in airmode at low throttle (0.0-1.0)
}

impl Freezable for QuadXMixer {}

impl CuTask for QuadXMixer {
    type Input<'m> = input_msg!('m, ControlInputs, BodyCommand);
    type Output<'m> = CuMsg<EscCommand>;
    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let motor_index = cfg_usize(config, "motor_index", 0)?;
        if motor_index >= QUADX_MIX.len() {
            return Err(CuError::from("motor_index out of range for QuadX"));
        }
        let props_out = match config {
            Some(cfg) => cfg.get::<bool>("props_out")?.unwrap_or(true),
            None => true,
        };
        let airmode_idle = cfg_f32(config, "airmode_idle_percent", 8.0)? / 100.0;

        Ok(Self {
            motor_index,
            props_out,
            airmode_idle: airmode_idle.clamp(0.0, 0.3), // Max 30% idle
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (ctrl_msg, cmd_msg) = *input;
        let ctrl_tov = expect_tov_time(ctrl_msg.tov)?;
        let cmd_tov = expect_tov_time(cmd_msg.tov)?;
        let output_tov = Tov::Time(cmd_tov);
        let Some(ctrl) = ctrl_msg.payload() else {
            output.tov = output_tov;
            status_if_not_firmware!(output.metadata, format!("m{} disarm", self.motor_index));
            output.set_payload(EscCommand::disarm());
            return Ok(());
        };

        let command = match cmd_msg.payload() {
            Some(cmd) if ctrl.armed => {
                let (roll_coeff, pitch_coeff, mut yaw_coeff) = QUADX_MIX[self.motor_index];
                if self.props_out {
                    yaw_coeff = -yaw_coeff;
                }
                let mix = cmd.roll.get::<ratio>() * roll_coeff
                    + cmd.pitch.get::<ratio>() * pitch_coeff
                    + cmd.yaw.get::<ratio>() * yaw_coeff;
                let throttle = ctrl.throttle.get::<ratio>().clamp(0.0, 1.0);

                // Apply airmode idle at low throttle (for control authority during power loops)
                // When throttle < 50%, scale from idle to normal. Above 50%, normal mixing.
                let motor = if throttle < 0.5 {
                    // Low throttle: blend from idle (at 0%) to normal (at 50%)
                    let blend = throttle * 2.0; // 0.0 at 0%, 1.0 at 50%
                    let base = self.airmode_idle * (1.0 - blend) + throttle * blend;
                    (base + mix).clamp(0.0, 1.0)
                } else {
                    // High throttle: normal mixing
                    (throttle + mix).clamp(0.0, 1.0)
                };

                let throttle_raw = (motor * 2047.0) as u16;
                EscCommand {
                    throttle: throttle_raw.max(DSHOT_MIN_ARM_CMD),
                    request_telemetry: true,
                }
            }
            _ => EscCommand::disarm(),
        };

        {
            let mut state = MOTOR_LOG.lock();
            if self.motor_index < state.values.len() {
                state.values[self.motor_index] = command.throttle;
            }
        }

        if self.motor_index == 0 {
            debug_rl!(&LOG_MOTORS, ctrl_tov, {
                let state = MOTOR_LOG.lock();
                debug!(
                    "motors cmd0={} cmd1={} cmd2={} cmd3={}",
                    state.values[0], state.values[1], state.values[2], state.values[3]
                );
            });
        }

        output.tov = output_tov;
        status_if_not_firmware!(
            output.metadata,
            format!(
                "m{} {}% {}",
                self.motor_index,
                dshot_percent(command.throttle),
                command.throttle
            )
        );
        output.set_payload(command);
        Ok(())
    }
}

fn normalize_axis(raw: u16, cfg: &RcMapper) -> f32 {
    let raw_i = raw as i32;
    let mid_i = cfg.rc_mid as i32;
    let deadband = cfg.deadband as i32;
    if (raw_i - mid_i).abs() <= deadband {
        return 0.0;
    }
    if raw >= cfg.rc_mid {
        let denom = cfg.rc_max.saturating_sub(cfg.rc_mid) as f32;
        if denom <= 0.0 {
            return 0.0;
        }
        ((raw - cfg.rc_mid) as f32 / denom).clamp(0.0, 1.0)
    } else {
        let denom = cfg.rc_mid.saturating_sub(cfg.rc_min) as f32;
        if denom <= 0.0 {
            return 0.0;
        }
        -((cfg.rc_mid - raw) as f32 / denom).clamp(0.0, 1.0)
    }
}

fn normalize_throttle(raw: u16, min: u16, max: u16) -> f32 {
    if max <= min {
        return 0.0;
    }
    let span = (max - min) as f32;
    (raw.saturating_sub(min) as f32 / span).clamp(0.0, 1.0)
}

fn expect_tov_time(tov: Tov) -> CuResult<CuTime> {
    match tov {
        Tov::Time(time) => Ok(time),
        Tov::Range(range) => {
            info!(
                "tov mismatch: expected time, got range start={} end={}",
                range.start.as_nanos(),
                range.end.as_nanos()
            );
            Err(CuError::from("Expected TOV::Time"))
        }
        Tov::None => {
            info!("tov mismatch: expected time, got none");
            Err(CuError::from("Expected TOV::Time"))
        }
    }
}

fn mode_label(mode: FlightMode) -> &'static str {
    match mode {
        FlightMode::Angle => "angle",
        FlightMode::Acro => "air",
        FlightMode::PositionHold => "position",
    }
}

#[cfg(not(feature = "firmware"))]
fn mode_tag(mode: FlightMode) -> &'static str {
    match mode {
        FlightMode::Angle => "ANG",
        FlightMode::Acro => "ACR",
        FlightMode::PositionHold => "POS",
    }
}

#[cfg(not(feature = "firmware"))]
fn throttle_percent(throttle: f32) -> u16 {
    (throttle.clamp(0.0, 1.0) * 100.0).round() as u16
}

#[cfg(not(feature = "firmware"))]
fn dshot_percent(raw: u16) -> u16 {
    ((u32::from(raw) * 100 + 1023) / 2047) as u16
}

fn dt_or_fallback(last_time: &mut Option<CuTime>, now: CuTime, fallback: CuDuration) -> CuDuration {
    let dt = last_time.map(|prev| now - prev);
    *last_time = Some(now);
    match dt {
        Some(duration) if duration > CuDuration::MIN => duration,
        _ => fallback,
    }
}

fn normalize_percent(raw: f32) -> f32 {
    let value = if raw > 1.0 { raw / 100.0 } else { raw };
    value.clamp(0.0, 1.0)
}

fn normalize_expo(raw: f32) -> f32 {
    normalize_percent(raw)
}

fn apply_expo(value: f32, expo: f32) -> f32 {
    let abs = value.abs();
    let weight = expo * abs * abs * abs + (1.0 - expo);
    value * weight
}

fn wrap_heading_deg(value: f32) -> f32 {
    let mut wrapped = libm::fmodf(value, 360.0);
    if wrapped < 0.0 {
        wrapped += 360.0;
    }
    wrapped
}

fn heading_from_yaw_deg(yaw_deg: f32, declination_deg: f32) -> Option<f32> {
    if !yaw_deg.is_finite() || !declination_deg.is_finite() {
        return None;
    }
    let heading_deg = wrap_heading_deg(-yaw_deg + declination_deg);
    heading_deg.is_finite().then_some(heading_deg)
}

fn heading_from_mag_xy_deg(mag_x: f32, mag_y: f32, declination_deg: f32) -> Option<f32> {
    if !mag_x.is_finite() || !mag_y.is_finite() || !declination_deg.is_finite() {
        return None;
    }
    let heading_deg = wrap_heading_deg(libm::atan2f(mag_y, mag_x).to_degrees() + declination_deg);
    heading_deg.is_finite().then_some(heading_deg)
}

fn heading_from_mag_level_deg(
    mag_x: f32,
    mag_y: f32,
    mag_z: f32,
    roll_rad: f32,
    pitch_rad: f32,
    declination_deg: f32,
) -> Option<f32> {
    if !mag_x.is_finite()
        || !mag_y.is_finite()
        || !mag_z.is_finite()
        || !roll_rad.is_finite()
        || !pitch_rad.is_finite()
        || !declination_deg.is_finite()
    {
        return None;
    }

    // Project body-frame magnetic vector into a level frame using roll/pitch.
    // Body axes are aerospace/NED: X forward, Y right, Z down.
    let sin_r = libm::sinf(roll_rad);
    let cos_r = libm::cosf(roll_rad);
    let sin_p = libm::sinf(pitch_rad);
    let cos_p = libm::cosf(pitch_rad);

    let horizontal_x = mag_x * cos_p + mag_z * sin_p;
    let horizontal_y = mag_x * sin_r * sin_p + mag_y * cos_r - mag_z * sin_r * cos_p;
    let horizontal_norm2 = horizontal_x * horizontal_x + horizontal_y * horizontal_y;
    if !horizontal_norm2.is_finite() || horizontal_norm2 <= 1.0e-12 {
        return None;
    }

    let heading_deg =
        wrap_heading_deg(libm::atan2f(horizontal_y, horizontal_x).to_degrees() + declination_deg);
    heading_deg.is_finite().then_some(heading_deg)
}

fn cfg_f32(config: Option<&ComponentConfig>, key: &str, default: f32) -> CuResult<f32> {
    let value = match config {
        Some(cfg) => cfg.get::<f64>(key)?,
        None => None,
    };
    Ok(value.map(|v| v as f32).unwrap_or(default))
}

fn cfg_u32(config: Option<&ComponentConfig>, key: &str, default: u32) -> CuResult<u32> {
    let value = match config {
        Some(cfg) => cfg.get::<u32>(key)?,
        None => None,
    };
    Ok(value.unwrap_or(default))
}

fn cfg_u16(config: Option<&ComponentConfig>, key: &str, default: u16) -> CuResult<u16> {
    let value = match config {
        Some(cfg) => cfg.get::<u32>(key)?,
        None => None,
    };
    Ok(value
        .map(|v| v.min(u16::MAX as u32) as u16)
        .unwrap_or(default))
}

fn cfg_bool(config: Option<&ComponentConfig>, key: &str, default: bool) -> CuResult<bool> {
    let value = match config {
        Some(cfg) => cfg.get::<bool>(key)?,
        None => None,
    };
    Ok(value.unwrap_or(default))
}

fn cfg_usize(config: Option<&ComponentConfig>, key: &str, default: usize) -> CuResult<usize> {
    let value = match config {
        Some(cfg) => cfg.get::<u32>(key)?,
        None => None,
    };
    Ok(value.map(|v| v as usize).unwrap_or(default))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_heading_close(actual: f32, expected: f32) {
        let err = (actual - expected + 540.0).rem_euclid(360.0) - 180.0;
        assert!(
            err.abs() < 1.0e-3,
            "heading mismatch: actual={actual} expected={expected} err={err}"
        );
    }

    #[test]
    fn heading_from_mag_xy_applies_declination_and_wraps() {
        assert_heading_close(
            heading_from_mag_xy_deg(1.0, 0.0, 12.5).expect("finite heading"),
            12.5,
        );
        assert_heading_close(
            heading_from_mag_xy_deg(1.0, 0.0, -10.0).expect("finite heading"),
            350.0,
        );
    }

    #[test]
    fn heading_from_mag_xy_uses_east_as_positive() {
        assert_heading_close(
            heading_from_mag_xy_deg(0.0, 1.0, 0.0).expect("finite heading"),
            90.0,
        );
        assert_heading_close(
            heading_from_mag_xy_deg(0.0, -1.0, 0.0).expect("finite heading"),
            270.0,
        );
    }

    #[test]
    fn heading_from_mag_xy_rejects_non_finite_inputs() {
        assert!(heading_from_mag_xy_deg(f32::NAN, 0.0, 0.0).is_none());
        assert!(heading_from_mag_xy_deg(1.0, 0.0, f32::NAN).is_none());
    }

    #[test]
    fn heading_from_yaw_converts_ahrs_sign_to_compass_convention() {
        assert_heading_close(
            heading_from_yaw_deg(90.0, 0.0).expect("finite heading"),
            270.0,
        );
        assert_heading_close(
            heading_from_yaw_deg(-90.0, 0.0).expect("finite heading"),
            90.0,
        );
        assert_heading_close(
            heading_from_yaw_deg(350.0, 20.0).expect("finite heading"),
            30.0,
        );
    }

    #[test]
    fn heading_from_mag_level_removes_pitch_induced_flip() {
        // World field in NED: north=20uT, east=0uT, down=45uT.
        // At +30 deg pitch and no yaw, raw atan2(mag_y,mag_x) would flip to 180 deg.
        let pitch = 30.0_f32.to_radians();
        let mag_x = 20.0 * libm::cosf(pitch) - 45.0 * libm::sinf(pitch);
        let mag_y = 0.0;
        let mag_z = 20.0 * libm::sinf(pitch) + 45.0 * libm::cosf(pitch);

        assert_heading_close(
            heading_from_mag_xy_deg(mag_x, mag_y, 0.0).expect("raw heading"),
            180.0,
        );
        assert_heading_close(
            heading_from_mag_level_deg(mag_x, mag_y, mag_z, 0.0, pitch, 0.0)
                .expect("level heading"),
            0.0,
        );
    }

    #[test]
    fn heading_from_mag_level_rejects_degenerate_horizontal_projection() {
        assert!(
            heading_from_mag_level_deg(0.0, 0.0, 42.0, 0.0, 0.0, 0.0).is_none(),
            "no horizontal magnetic component should not produce heading"
        );
    }
}
