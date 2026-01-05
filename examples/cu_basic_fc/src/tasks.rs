#![allow(dead_code)]

use crate::GreenLed;
use cu_ahrs::AhrsPose;
use cu_bdshot::{EscCommand, EscTelemetry};
use cu_crsf::messages::RcChannelsPayload;
use cu_msp_bridge::{MspRequestBatch, MspResponseBatch};
use cu_msp_lib::structs::{MspDisplayPort, MspRequest};
use cu_pid::{PIDControlOutputPayload, PIDController};
use cu_sensor_payloads::ImuPayload;
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::Serialize;
use uom::si::angular_velocity::degree_per_second;
use uom::si::thermodynamic_temperature::degree_celsius;

mod bmi088;

#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, PartialEq, Eq)]
#[repr(u8)]
pub enum FlightMode {
    #[default]
    Angle = 0,
    Acro = 1,
    PositionHold = 2,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize)]
pub struct ControlInputs {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub armed: bool,
    pub mode: FlightMode,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize)]
pub struct BodyRateSetpoint {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize)]
pub struct BodyCommand {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

const LOG_PERIOD_MS: u64 = 500;
const HEAP_LOG_PERIOD_MS: u64 = 500;
const VTX_HEARTBEAT_PERIOD_MS: u64 = 1000;
const VTX_DRAW_PERIOD_MS: u64 = 250;

resources!({
    led => Owned<spin::Mutex<GreenLed>>,
});

struct LogRateLimiter {
    last: Option<CuTime>,
}

impl LogRateLimiter {
    const fn new() -> Self {
        Self { last: None }
    }

    fn should_log(&mut self, clock: &RobotClock, tov: Tov) -> bool {
        let now = match tov {
            Tov::Time(time) => time,
            Tov::Range(range) => range.end,
            Tov::None => clock.now(),
        };
        match self.last {
            Some(prev) if now - prev < CuDuration::from_millis(LOG_PERIOD_MS) => false,
            _ => {
                self.last = Some(now);
                true
            }
        }
    }
}

macro_rules! info_rl {
    ($state:expr, $clock:expr, $tov:expr, { $($body:tt)* }) => {{
        let mut state = $state.lock();
        if state.should_log($clock, $tov) {
            $($body)*
        }
    }};
    ($state:expr, $clock:expr, $tov:expr, $($arg:tt)+) => {{
        info_rl!($state, $clock, $tov, { defmt::info!($($arg)+); });
    }};
}

static LOG_CTRL: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());
static LOG_TELEMETRY: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());
static LOG_IMU: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());
static LOG_RC: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());
static LOG_RATE: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());
static LOG_MOTORS: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());

pub struct LedBeat {
    on: bool,
    led: spin::Mutex<GreenLed>,
}

pub struct ControlSink {
    last_heap_log: Option<CuTime>,
}
impl Freezable for ControlSink {}
impl CuSinkTask for ControlSink {
    type Resources<'r> = ();
    type Input<'m> = CuMsg<ControlInputs>;

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            last_heap_log: None,
        })
    }

    fn process<'i>(&mut self, clock: &RobotClock, inputs: &Self::Input<'i>) -> CuResult<()> {
        let now = match inputs.tov {
            Tov::Time(time) => time,
            Tov::Range(range) => range.end,
            Tov::None => clock.now(),
        };

        if self
            .last_heap_log
            .map(|prev| now - prev >= CuDuration::from_millis(HEAP_LOG_PERIOD_MS))
            .unwrap_or(true)
        {
            let (user, allocated, total, free) = crate::heap_stats();
            defmt::info!(
                "Heap stats(rt): user={} alloc={} total={} free={}",
                user,
                allocated,
                total,
                free
            );
            self.last_heap_log = Some(now);
        }

        if let Some(ctrl) = inputs.payload() {
            info_rl!(
                &LOG_CTRL,
                clock,
                inputs.tov,
                "ctrl roll={} pitch={} yaw={} thr={} armed={} mode={}",
                ctrl.roll,
                ctrl.pitch,
                ctrl.yaw,
                ctrl.throttle,
                ctrl.armed,
                mode_label(ctrl.mode)
            );
        }
        Ok(())
    }
}

pub struct ThrottleToEsc;

impl Freezable for ThrottleToEsc {}

impl CuTask for ThrottleToEsc {
    type Resources<'r> = ();
    type Input<'m> = CuMsg<ControlInputs>;
    type Output<'m> = CuMsg<EscCommand>;

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process<'i, 'o>(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = ensure_tov(clock, input.tov);
        if let Some(ctrl) = input.payload() {
            let raw = if ctrl.armed { ctrl.throttle } else { 0.0 };
            let throttle = (raw.clamp(0.0, 1.0) * 2047.0) as u16;
            output.set_payload(EscCommand {
                throttle,
                request_telemetry: true,
            });
        } else {
            output.clear_payload();
        }
        Ok(())
    }
}

pub struct TelemetryLogger<const ESC: usize> {}

impl<const ESC: usize> Freezable for TelemetryLogger<ESC> {}

impl<const ESC: usize> CuSinkTask for TelemetryLogger<ESC> {
    type Resources<'r> = ();
    type Input<'m> = CuMsg<EscTelemetry>;

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process<'i>(&mut self, clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()> {
        if let Some(payload) = input.payload() {
            info_rl!(&LOG_TELEMETRY, clock, input.tov, {
                if let Some(sample) = payload.sample {
                    defmt::info!("ESC{} telemetry {}", ESC, sample);
                } else {
                    defmt::info!("ESC{} telemetry missing", ESC);
                }
            });
        }
        Ok(())
    }
}

pub type TelemetryLogger0 = TelemetryLogger<0>;
pub type TelemetryLogger1 = TelemetryLogger<1>;
pub type TelemetryLogger2 = TelemetryLogger<2>;
pub type TelemetryLogger3 = TelemetryLogger<3>;

pub struct MspNoopSource;

impl Freezable for MspNoopSource {}

impl CuSrcTask for MspNoopSource {
    type Resources<'r> = ();
    type Output<'m> = CuMsg<MspRequestBatch>;

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process<'o>(&mut self, clock: &RobotClock, output: &mut Self::Output<'o>) -> CuResult<()> {
        output.tov = ensure_tov(clock, Tov::None);
        output.clear_payload();
        Ok(())
    }
}

pub struct MspNoopSink;

impl Freezable for MspNoopSink {}

impl CuSinkTask for MspNoopSink {
    type Resources<'r> = ();
    type Input<'m> = CuMsg<MspResponseBatch>;

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process<'i>(&mut self, _clock: &RobotClock, _input: &Self::Input<'i>) -> CuResult<()> {
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StatusLabel {
    Disarmed,
    Angle,
    Air,
}

impl StatusLabel {
    const fn as_str(self) -> &'static str {
        match self {
            StatusLabel::Disarmed => "XXX  ",
            StatusLabel::Angle => "ANGLE",
            StatusLabel::Air => "AIR  ",
        }
    }
}

pub struct VtxOsd {
    row: u8,
    cols: u8,
    col_center: u8,
    last_label: Option<StatusLabel>,
    last_heartbeat: Option<CuTime>,
    last_draw: Option<CuTime>,
}

impl Freezable for VtxOsd {}

impl CuTask for VtxOsd {
    type Resources<'r> = ();
    type Input<'m> = CuMsg<ControlInputs>;
    type Output<'m> = CuMsg<MspRequestBatch>;

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cols = cfg_u16(config, "cols", 53).max(1).min(u8::MAX as u16) as u8;
        let default_center = (cols / 2) as u16;
        let col_center =
            cfg_u16(config, "col_center", default_center).min(cols.saturating_sub(1) as u16) as u8;
        let row = cfg_u16(config, "row", 13).min(u8::MAX as u16) as u8;
        Ok(Self {
            row,
            cols,
            col_center,
            last_label: None,
            last_heartbeat: None,
            last_draw: None,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.tov = ensure_tov(clock, input.tov);
        let now = match output.tov {
            Tov::Time(time) => time,
            Tov::Range(range) => range.end,
            Tov::None => clock.now(),
        };
        let mut batch = MspRequestBatch::new();

        let heartbeat_due = self
            .last_heartbeat
            .map(|prev| now - prev >= CuDuration::from_millis(VTX_HEARTBEAT_PERIOD_MS))
            .unwrap_or(true);
        if heartbeat_due {
            batch.push(MspRequest::MspDisplayPort(MspDisplayPort::heartbeat()));
            self.last_heartbeat = Some(now);
        }

        let Some(ctrl) = input.payload() else {
            if batch.0.is_empty() {
                output.clear_payload();
            } else {
                output.set_payload(batch);
            }
            return Ok(());
        };

        let label = if !ctrl.armed {
            StatusLabel::Disarmed
        } else if ctrl.mode == FlightMode::Acro {
            StatusLabel::Air
        } else {
            StatusLabel::Angle
        };

        let label_changed = self.last_label != Some(label);
        if label_changed {
            self.last_label = Some(label);
        }

        let text = label.as_str();
        let width = text.len() as u8;
        let col = if self.cols <= width {
            0
        } else {
            let half = width / 2;
            let mut col = self.col_center.saturating_sub(half);
            if col.saturating_add(width) > self.cols {
                col = self.cols.saturating_sub(width);
            }
            col
        };

        if label_changed {
            batch.push(MspRequest::MspDisplayPort(MspDisplayPort::clear_screen()));
            batch.push(MspRequest::MspDisplayPort(MspDisplayPort::write_string(
                self.row, col, 0, text,
            )));
        }

        let draw_due = self
            .last_draw
            .map(|prev| now - prev >= CuDuration::from_millis(VTX_DRAW_PERIOD_MS))
            .unwrap_or(true);
        if label_changed || draw_due {
            batch.push(MspRequest::MspDisplayPort(MspDisplayPort::draw_screen()));
            self.last_draw = Some(now);
        }

        if batch.0.is_empty() {
            output.clear_payload();
        } else {
            output.set_payload(batch);
        }
        Ok(())
    }
}

pub struct ImuLogger {
    last_tov: Option<CuTime>,
}

impl Freezable for ImuLogger {}

impl CuSinkTask for ImuLogger {
    type Resources<'r> = ();
    type Input<'m> = CuMsg<ImuPayload>;

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { last_tov: None })
    }

    fn process<'i>(&mut self, clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()> {
        if let Some(payload) = input.payload() {
            info_rl!(&LOG_IMU, clock, input.tov, {
                let (tov_kind, tov_start_ns, tov_end_ns, tov_time) = match input.tov {
                    Tov::Time(t) => (1_u8, t.as_nanos(), t.as_nanos(), Some(t)),
                    Tov::Range(r) => (2_u8, r.start.as_nanos(), r.end.as_nanos(), Some(r.end)),
                    Tov::None => (0_u8, 0_u64, 0_u64, None),
                };
                let dt_us = match (self.last_tov, tov_time) {
                    (Some(prev), Some(curr)) => (curr - prev).as_micros(),
                    _ => 0,
                };
                self.last_tov = tov_time;
                let gx_dps = payload.gyro_x.get::<degree_per_second>();
                let gy_dps = payload.gyro_y.get::<degree_per_second>();
                let gz_dps = payload.gyro_z.get::<degree_per_second>();
                let temp_c = payload.temperature.get::<degree_celsius>();
                defmt::info!(
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

pub type Bmi088Source = bmi088::Bmi088Source<
    crate::resources::Bmi088Spi,
    crate::resources::Bmi088AccCs,
    crate::resources::Bmi088GyrCs,
    crate::resources::Bmi088Delay,
>;

impl Freezable for FlightMode {}
impl Freezable for ControlInputs {}
impl Freezable for BodyRateSetpoint {}
impl Freezable for BodyCommand {}
impl Freezable for LedBeat {}

impl CuSinkTask for LedBeat {
    type Resources<'r> = Resources;
    type Input<'m> = CuMsg<ControlInputs>;

    fn new_with(_config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            on: false,
            led: resources.led.0,
        })
    }

    fn process<'i>(&mut self, _clock: &RobotClock, _inputs: &Self::Input<'i>) -> CuResult<()> {
        // Toggle the green LED so we can see if the Copper loop is alive.
        let mut led = self.led.lock();
        if self.on {
            led.set_low();
        } else {
            led.set_high();
        }
        self.on = !self.on;
        Ok(())
    }
}

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
    type Resources<'r> = ();
    type Input<'m> = CuMsg<RcChannelsPayload>;
    type Output<'m> = CuMsg<ControlInputs>;

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let arm_cfg = config.and_then(|cfg| cfg.get::<u32>("arm_channel"));
        let mut arm_channel = arm_cfg.map(|v| v as usize).unwrap_or(3);
        if arm_channel > 15 {
            defmt::info!(
                "rc mapper arm_channel {} out of range, clamping to 15",
                arm_channel
            );
            arm_channel = 15;
        }
        let arm_min = cfg_u16(config, "arm_min", 1700);
        let arm_max = cfg_u16(config, "arm_max", 1811);
        let mode_cfg = config
            .and_then(|cfg| cfg.get::<u32>("mode_channel"))
            .map(|v| v as usize);
        let mode_low_max = config
            .and_then(|cfg| cfg.get::<u32>("mode_low_max"))
            .map(|v| v.min(u16::MAX as u32) as u16);
        let mode_mid_max = config
            .and_then(|cfg| cfg.get::<u32>("mode_mid_max"))
            .map(|v| v.min(u16::MAX as u32) as u16);

        defmt::info!(
            "rc mapper cfg arm_channel={:?} arm_min={} arm_max={} mode_channel={:?} mode_low_max={} mode_mid_max={}",
            arm_cfg,
            arm_min,
            arm_max,
            mode_cfg,
            mode_low_max.unwrap_or(0),
            mode_mid_max.unwrap_or(0)
        );

        Ok(Self {
            rc_min: cfg_u16(config, "rc_min", 172),
            rc_mid: cfg_u16(config, "rc_mid", 992),
            rc_max: cfg_u16(config, "rc_max", 1811),
            deadband: cfg_u16(config, "rc_deadband", 0),
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
        clock: &RobotClock,
        inputs: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let Some(rc) = inputs.payload() else {
            output.tov = ensure_tov(clock, inputs.tov);
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

        info_rl!(&LOG_RC, clock, inputs.tov, {
            let mode_channel = self.mode_channel.unwrap_or(0);
            let mode_value = self
                .mode_channel
                .and_then(|idx| channels.get(idx).copied())
                .unwrap_or(0);
            defmt::info!(
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

        output.tov = ensure_tov(clock, inputs.tov);
        output.set_payload(ControlInputs {
            roll,
            pitch,
            yaw,
            throttle,
            armed,
            mode,
        });
        Ok(())
    }
}

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
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, ImuPayload, ControlInputs);
    type Output<'m> = CuMsg<ImuPayload>;

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cal_ms = cfg_u32(config, "cal_ms", 3000);
        let sample_period_ms = cfg_u32(config, "sample_period_ms", 10);
        let default_samples = (cal_ms / sample_period_ms.max(1)).max(1);
        let required_samples = cfg_u32(config, "cal_samples", default_samples);

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
        clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (imu_msg, ctrl_msg) = *input;
        let Some(imu) = imu_msg.payload() else {
            output.tov = ensure_tov(clock, imu_msg.tov);
            output.clear_payload();
            return Ok(());
        };
        let armed = ctrl_msg.payload().map(|c| c.armed).unwrap_or(false);

        if !armed {
            self.calibrating = false;
            self.sum = [0.0; 3];
            self.samples = 0;
            self.last_armed = false;
            output.tov = ensure_tov(clock, imu_msg.tov);
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
                defmt::info!(
                    "imu gyro bias x={} deg.s⁻¹ y={} deg.s⁻¹ z={} deg.s⁻¹",
                    bias_deg[0],
                    bias_deg[1],
                    bias_deg[2]
                );
            }

            output.tov = ensure_tov(clock, imu_msg.tov);
            output.clear_payload();
            output.metadata.set_status("cal");
            return Ok(());
        }

        let accel_mps2 = [imu.accel_x.value, imu.accel_y.value, imu.accel_z.value];
        let gyro_rad = [
            imu.gyro_x.value - self.bias[0],
            imu.gyro_y.value - self.bias[1],
            imu.gyro_z.value - self.bias[2],
        ];
        let temp_c = imu.temperature.get::<degree_celsius>();

        output.tov = ensure_tov(clock, imu_msg.tov);
        output.metadata.set_status("ok");
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

pub struct AttitudeController {
    roll_pid: AxisPid,
    pitch_pid: AxisPid,
    angle_limit_rad: f32,
    rate_limit_rad: f32,
    acro_rate_rad: f32,
    dt_fallback: CuDuration,
    last_time: Option<CuTime>,
    last_mode: FlightMode,
}

impl Freezable for AttitudeController {}

impl CuTask for AttitudeController {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, AhrsPose, ControlInputs);
    type Output<'m> = CuMsg<BodyRateSetpoint>;

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let angle_limit_rad = cfg_f32(config, "angle_limit_deg", 25.0).to_radians();
        let rate_limit_rad = cfg_f32(config, "rate_limit_dps", 180.0).to_radians();
        let acro_rate_rad = cfg_f32(config, "acro_rate_dps", 360.0).to_radians();
        let kp = cfg_f32(config, "kp", 4.0);
        let ki = cfg_f32(config, "ki", 0.0);
        let kd = cfg_f32(config, "kd", 0.0);
        let dt_fallback = CuDuration::from_millis(cfg_u32(config, "dt_ms", 10) as u64);

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
            dt_fallback,
            last_time: None,
            last_mode: FlightMode::Angle,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (pose_msg, ctrl_msg) = *input;
        let output_tov = ensure_tov(clock, prefer_tov(pose_msg.tov, ctrl_msg.tov));
        let Some(pose) = pose_msg.payload() else {
            output.tov = output_tov;
            output.clear_payload();
            return Ok(());
        };
        let Some(ctrl) = ctrl_msg.payload() else {
            output.tov = output_tov;
            output.clear_payload();
            return Ok(());
        };

        if !ctrl.armed {
            self.roll_pid.reset();
            self.pitch_pid.reset();
            self.last_mode = ctrl.mode;
            output.tov = output_tov;
            output.set_payload(BodyRateSetpoint::default());
            return Ok(());
        }

        if ctrl.mode != self.last_mode {
            self.roll_pid.reset();
            self.pitch_pid.reset();
            self.last_mode = ctrl.mode;
        }

        let now = clock.now();
        let dt = dt_or_fallback(&mut self.last_time, now, self.dt_fallback);

        let (roll_rate, pitch_rate) =
            if matches!(ctrl.mode, FlightMode::Angle | FlightMode::PositionHold) {
                let target_roll = (ctrl.roll * self.angle_limit_rad)
                    .clamp(-self.angle_limit_rad, self.angle_limit_rad);
                let target_pitch = (ctrl.pitch * self.angle_limit_rad)
                    .clamp(-self.angle_limit_rad, self.angle_limit_rad);
                let roll_measure = pose.roll - target_roll;
                let pitch_measure = pose.pitch - target_pitch;
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
                (
                    (ctrl.roll * self.acro_rate_rad).clamp(-self.acro_rate_rad, self.acro_rate_rad),
                    (ctrl.pitch * self.acro_rate_rad)
                        .clamp(-self.acro_rate_rad, self.acro_rate_rad),
                )
            };

        let yaw_rate =
            (ctrl.yaw * self.acro_rate_rad).clamp(-self.acro_rate_rad, self.acro_rate_rad);

        output.tov = output_tov;
        output.set_payload(BodyRateSetpoint {
            roll: roll_rate,
            pitch: pitch_rate,
            yaw: yaw_rate,
        });
        Ok(())
    }
}

pub struct RateController {
    roll_pid: AxisPid,
    pitch_pid: AxisPid,
    yaw_pid: AxisPid,
    output_limit: f32,
    dt_fallback: CuDuration,
    i_throttle_min: f32,
    last_time: Option<CuTime>,
}

impl Freezable for RateController {}

impl CuTask for RateController {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, BodyRateSetpoint, ImuPayload, ControlInputs);
    type Output<'m> = CuMsg<BodyCommand>;

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let kp = cfg_f32(config, "kp", 0.15);
        let ki = cfg_f32(config, "ki", 0.0);
        let kd = cfg_f32(config, "kd", 0.0);
        let kp_yaw = cfg_f32(config, "kp_yaw", kp);
        let ki_yaw = cfg_f32(config, "ki_yaw", ki);
        let kd_yaw = cfg_f32(config, "kd_yaw", kd);
        let output_limit = cfg_f32(config, "output_limit", 1.0);
        let dt_fallback = CuDuration::from_millis(cfg_u32(config, "dt_ms", 10) as u64);
        let i_throttle_min = cfg_f32(config, "i_throttle_min", 0.05);

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
            last_time: None,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (setpoint_msg, imu_msg, ctrl_msg) = *input;
        let output_tov = ensure_tov(
            clock,
            prefer_tov(imu_msg.tov, prefer_tov(setpoint_msg.tov, ctrl_msg.tov)),
        );
        let Some(setpoint) = setpoint_msg.payload() else {
            output.tov = output_tov;
            output.clear_payload();
            return Ok(());
        };
        let Some(imu) = imu_msg.payload() else {
            output.tov = output_tov;
            output.clear_payload();
            return Ok(());
        };
        let ctrl = ctrl_msg.payload();
        let armed = ctrl.map(|c| c.armed).unwrap_or(false);
        let throttle = ctrl.map(|c| c.throttle).unwrap_or(0.0);

        if !armed {
            self.roll_pid.reset();
            self.pitch_pid.reset();
            self.yaw_pid.reset();
            output.tov = output_tov;
            output.set_payload(BodyCommand::default());
            return Ok(());
        }

        let now = clock.now();
        let dt = dt_or_fallback(&mut self.last_time, now, self.dt_fallback);

        let roll_measure = imu.gyro_x.value - setpoint.roll;
        let pitch_measure = imu.gyro_y.value - setpoint.pitch;
        let yaw_measure = imu.gyro_z.value - setpoint.yaw;

        let roll_out = self.roll_pid.update(roll_measure, dt).unwrap_or_default();
        let pitch_out = self.pitch_pid.update(pitch_measure, dt).unwrap_or_default();
        let yaw_out = self.yaw_pid.update(yaw_measure, dt).unwrap_or_default();
        let roll_cmd = roll_out.output;
        let pitch_cmd = pitch_out.output;
        let yaw_cmd = yaw_out.output;

        let log_tov = ensure_tov(
            clock,
            prefer_tov(ctrl_msg.tov, prefer_tov(setpoint_msg.tov, imu_msg.tov)),
        );
        info_rl!(&LOG_RATE, clock, log_tov, {
            let sp_roll = setpoint.roll.to_degrees();
            let sp_pitch = setpoint.pitch.to_degrees();
            let sp_yaw = setpoint.yaw.to_degrees();
            let gyro_roll = imu.gyro_x.value.to_degrees();
            let gyro_pitch = imu.gyro_y.value.to_degrees();
            let gyro_yaw = imu.gyro_z.value.to_degrees();
            defmt::info!(
                "rate_pid dt_us={} sp_r={} gyro_r={} out_r={} p_r={} i_r={} d_r={}",
                dt.as_micros(),
                sp_roll,
                gyro_roll,
                roll_out.output,
                roll_out.p,
                roll_out.i,
                roll_out.d
            );
            defmt::info!(
                "rate_pid sp_p={} gyro_p={} out_p={} p_p={} i_p={} d_p={}",
                sp_pitch,
                gyro_pitch,
                pitch_out.output,
                pitch_out.p,
                pitch_out.i,
                pitch_out.d
            );
            defmt::info!(
                "rate_pid sp_y={} gyro_y={} out_y={} p_y={} i_y={} d_y={}",
                sp_yaw,
                gyro_yaw,
                yaw_out.output,
                yaw_out.p,
                yaw_out.i,
                yaw_out.d
            );
        });

        if throttle < self.i_throttle_min {
            self.roll_pid.reset_integral();
            self.pitch_pid.reset_integral();
            self.yaw_pid.reset_integral();
        }

        output.tov = output_tov;
        output.set_payload(BodyCommand {
            roll: roll_cmd.clamp(-self.output_limit, self.output_limit),
            pitch: pitch_cmd.clamp(-self.output_limit, self.output_limit),
            yaw: yaw_cmd.clamp(-self.output_limit, self.output_limit),
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

pub struct QuadXMixer {
    motor_index: usize,
    props_out: bool,
}

impl Freezable for QuadXMixer {}

impl CuTask for QuadXMixer {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, ControlInputs, BodyCommand);
    type Output<'m> = CuMsg<EscCommand>;

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let motor_index = cfg_usize(config, "motor_index", 0);
        if motor_index >= QUADX_MIX.len() {
            return Err(CuError::from("motor_index out of range for QuadX"));
        }
        let props_out = config
            .and_then(|cfg| cfg.get::<bool>("props_out"))
            .unwrap_or(true);

        Ok(Self {
            motor_index,
            props_out,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (ctrl_msg, cmd_msg) = *input;
        let output_tov = ensure_tov(clock, prefer_tov(cmd_msg.tov, ctrl_msg.tov));
        let Some(ctrl) = ctrl_msg.payload() else {
            output.tov = output_tov;
            output.set_payload(EscCommand::disarm());
            return Ok(());
        };

        let command = match cmd_msg.payload() {
            Some(cmd) if ctrl.armed => {
                let (roll_coeff, pitch_coeff, mut yaw_coeff) = QUADX_MIX[self.motor_index];
                if self.props_out {
                    yaw_coeff = -yaw_coeff;
                }
                let mix = cmd.roll * roll_coeff + cmd.pitch * pitch_coeff + cmd.yaw * yaw_coeff;
                let throttle = ctrl.throttle.clamp(0.0, 1.0);
                let motor = (throttle + mix).clamp(0.0, 1.0);
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
            info_rl!(&LOG_MOTORS, clock, ctrl_msg.tov, {
                let state = MOTOR_LOG.lock();
                defmt::info!(
                    "motors cmd0={} cmd1={} cmd2={} cmd3={}",
                    state.values[0],
                    state.values[1],
                    state.values[2],
                    state.values[3]
                );
            });
        }

        output.tov = output_tov;
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
        let denom = (cfg.rc_max.saturating_sub(cfg.rc_mid)) as f32;
        if denom <= 0.0 {
            return 0.0;
        }
        ((raw - cfg.rc_mid) as f32 / denom).clamp(0.0, 1.0)
    } else {
        let denom = (cfg.rc_mid.saturating_sub(cfg.rc_min)) as f32;
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
    ((raw.saturating_sub(min)) as f32 / span).clamp(0.0, 1.0)
}

fn prefer_tov(primary: Tov, fallback: Tov) -> Tov {
    match primary {
        Tov::None => fallback,
        _ => primary,
    }
}

fn ensure_tov(clock: &RobotClock, tov: Tov) -> Tov {
    match tov {
        Tov::None => Tov::Time(clock.now()),
        _ => tov,
    }
}

fn mode_label(mode: FlightMode) -> &'static str {
    match mode {
        FlightMode::Angle => "angle",
        FlightMode::Acro => "air",
        FlightMode::PositionHold => "position",
    }
}

fn dt_or_fallback(last_time: &mut Option<CuTime>, now: CuTime, fallback: CuDuration) -> CuDuration {
    let dt = last_time.map(|prev| now - prev);
    *last_time = Some(now);
    match dt {
        Some(duration) if duration > CuDuration::MIN => duration,
        _ => fallback,
    }
}

fn cfg_f32(config: Option<&ComponentConfig>, key: &str, default: f32) -> f32 {
    config
        .and_then(|cfg| cfg.get::<f64>(key))
        .map(|v| v as f32)
        .unwrap_or(default)
}

fn cfg_u32(config: Option<&ComponentConfig>, key: &str, default: u32) -> u32 {
    config
        .and_then(|cfg| cfg.get::<u32>(key))
        .unwrap_or(default)
}

fn cfg_u16(config: Option<&ComponentConfig>, key: &str, default: u16) -> u16 {
    config
        .and_then(|cfg| cfg.get::<u32>(key))
        .map(|v| v.min(u16::MAX as u32) as u16)
        .unwrap_or(default)
}

fn cfg_usize(config: Option<&ComponentConfig>, key: &str, default: usize) -> usize {
    config
        .and_then(|cfg| cfg.get::<u32>(key))
        .map(|v| v as usize)
        .unwrap_or(default)
}
