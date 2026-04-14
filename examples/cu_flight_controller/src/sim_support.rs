#![cfg(any(feature = "sim", feature = "bevymon"))]

use cu_gnss_payloads::{
    GeodeticPosition, GnssAccuracy, GnssCommandAck, GnssEpochTime, GnssFixSolution, GnssFixType,
    GnssInfoText, GnssRawUbxFrame, GnssRfStatus, GnssSatelliteState, GnssSatsInView,
    GnssSignalState,
};
use cu_sensor_payloads::{BarometerPayload, ImuPayload, MagnetometerPayload};
use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::f32::{Angle as Angle32, Length, Velocity};
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, OnceLock};

static SIM_ACTIVITY_LED_STATE: OnceLock<Arc<AtomicBool>> = OnceLock::new();
static SIM_BATTERY_THROTTLE_BITS: OnceLock<Arc<AtomicU32>> = OnceLock::new();
static SIM_BATTERY_ARMED_STATE: OnceLock<Arc<AtomicBool>> = OnceLock::new();
static SIM_GNSS_STATE: OnceLock<Arc<SimGnssState>> = OnceLock::new();
pub(crate) const GNSS_FIXED_LAT_DEG: f64 = 30.389861114639405;
pub(crate) const GNSS_FIXED_LON_DEG: f64 = -97.69316827380047;
pub(crate) const GNSS_FIXED_ELLIPSOID_ALT_M: f32 = 225.0;
pub(crate) const GNSS_FIXED_MSL_ALT_M: f32 = 212.0;
const GNSS_FIXED_SAT_COUNT: u8 = 14;

#[derive(Default)]
pub(crate) struct SimGnssState {
    pub(crate) lat_deg_bits: AtomicU64,
    pub(crate) lon_deg_bits: AtomicU64,
    pub(crate) ellipsoid_alt_m_bits: AtomicU32,
    pub(crate) msl_alt_m_bits: AtomicU32,
    pub(crate) velocity_north_mps_bits: AtomicU32,
    pub(crate) velocity_east_mps_bits: AtomicU32,
    pub(crate) velocity_down_mps_bits: AtomicU32,
    pub(crate) ground_speed_mps_bits: AtomicU32,
    pub(crate) heading_motion_deg_bits: AtomicU32,
}

pub(crate) fn sim_gnss_state() -> Arc<SimGnssState> {
    SIM_GNSS_STATE
        .get_or_init(|| {
            Arc::new(SimGnssState {
                lat_deg_bits: AtomicU64::new(GNSS_FIXED_LAT_DEG.to_bits()),
                lon_deg_bits: AtomicU64::new(GNSS_FIXED_LON_DEG.to_bits()),
                ellipsoid_alt_m_bits: AtomicU32::new(GNSS_FIXED_ELLIPSOID_ALT_M.to_bits()),
                msl_alt_m_bits: AtomicU32::new(GNSS_FIXED_MSL_ALT_M.to_bits()),
                velocity_north_mps_bits: AtomicU32::new(0.0_f32.to_bits()),
                velocity_east_mps_bits: AtomicU32::new(0.0_f32.to_bits()),
                velocity_down_mps_bits: AtomicU32::new(0.0_f32.to_bits()),
                ground_speed_mps_bits: AtomicU32::new(0.0_f32.to_bits()),
                heading_motion_deg_bits: AtomicU32::new(0.0_f32.to_bits()),
            })
        })
        .clone()
}

pub(crate) fn sim_activity_led_state() -> Arc<AtomicBool> {
    SIM_ACTIVITY_LED_STATE
        .get_or_init(|| Arc::new(AtomicBool::new(false)))
        .clone()
}

pub(crate) fn sim_battery_throttle_state() -> Arc<AtomicU32> {
    SIM_BATTERY_THROTTLE_BITS
        .get_or_init(|| Arc::new(AtomicU32::new(0.0_f32.to_bits())))
        .clone()
}

pub(crate) fn sim_battery_armed_state() -> Arc<AtomicBool> {
    SIM_BATTERY_ARMED_STATE
        .get_or_init(|| Arc::new(AtomicBool::new(false)))
        .clone()
}

#[derive(Clone, Default)]
pub struct SimActivityLed {
    state: Arc<AtomicBool>,
}

impl SimActivityLed {
    pub fn set(&self, on: bool) {
        self.state.store(on, Ordering::Relaxed);
    }
}

pub fn sim_activity_led() -> SimActivityLed {
    SimActivityLed {
        state: sim_activity_led_state(),
    }
}

fn sim_battery_throttle() -> f32 {
    let bits = sim_battery_throttle_state().load(Ordering::Relaxed);
    f32::from_bits(bits).clamp(0.0, 1.0)
}

fn sim_battery_is_armed() -> bool {
    sim_battery_armed_state().load(Ordering::Relaxed)
}

#[derive(Clone, Reflect)]
pub struct SimBatteryAdc {
    base_voltage: f32,
    phase: f32,
    sag_max_ratio: f32,
}

impl Default for SimBatteryAdc {
    fn default() -> Self {
        Self {
            base_voltage: 16.0,
            phase: 0.0,
            sag_max_ratio: 0.08,
        }
    }
}

impl SimBatteryAdc {
    pub fn read_voltage_v(&mut self) -> f32 {
        if !sim_battery_is_armed() {
            self.phase = 0.0;
            return self.base_voltage.max(0.0);
        }
        // Keep a small deterministic ripple so downstream battery logic sees live updates.
        self.phase += 0.05;
        let ripple = self.phase.sin() * 0.2;
        let sag_ratio = (self.sag_max_ratio * sim_battery_throttle()).clamp(0.0, 0.5);
        let sagged = self.base_voltage * (1.0 - sag_ratio);
        (sagged + ripple).max(0.0)
    }
}

pub fn sim_battery_adc(base_voltage: f32, sag_max_ratio: f32) -> SimBatteryAdc {
    SimBatteryAdc {
        base_voltage,
        phase: 0.0,
        sag_max_ratio: sag_max_ratio.clamp(0.0, 0.5),
    }
}

#[derive(Default, Reflect)]
pub struct SimBmi088Source {
    step: u64,
}

impl Freezable for SimBmi088Source {}

impl CuSrcTask for SimBmi088Source {
    type Resources<'r> = ();
    type Output<'m> = CuMsg<ImuPayload>;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { step: 0 })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let t = (self.step as f32) * 0.01;
        self.step = self.step.saturating_add(1);
        output.tov = Tov::Time(ctx.now());
        output.set_payload(ImuPayload::from_raw(
            [0.0, 0.0, 9.81],
            [0.05 * t.sin(), 0.03 * t.cos(), 0.01 * t.sin()],
            32.0,
        ));
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct SimDps310Source {
    step: u64,
}

impl Freezable for SimDps310Source {}

impl CuSrcTask for SimDps310Source {
    type Resources<'r> = ();
    type Output<'m> = CuMsg<BarometerPayload>;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { step: 0 })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let t = (self.step as f32) * 0.02;
        self.step = self.step.saturating_add(1);
        output.tov = Tov::Time(ctx.now());
        output.set_payload(BarometerPayload::from_raw(
            101_325.0 + (t.sin() * 20.0),
            28.0,
        ));
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct SimIst8310Source {
    step: u64,
}

impl Freezable for SimIst8310Source {}

impl CuSrcTask for SimIst8310Source {
    type Resources<'r> = ();
    type Output<'m> = CuMsg<MagnetometerPayload>;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { step: 0 })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let t = (self.step as f32) * 0.015;
        self.step = self.step.saturating_add(1);
        output.tov = Tov::Time(ctx.now());
        output.set_payload(MagnetometerPayload::from_raw([
            35.0 + 2.0 * t.cos(),
            0.5 * t.sin(),
            -42.0,
        ]));
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct SimGnssSource;

impl Freezable for SimGnssSource {}

impl CuSrcTask for SimGnssSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(
        GnssEpochTime,
        GnssFixSolution,
        GnssAccuracy,
        GnssSatsInView,
        GnssSatelliteState,
        GnssSignalState,
        GnssRfStatus,
        GnssInfoText,
        GnssCommandAck,
        GnssRawUbxFrame
    );

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let state = sim_gnss_state();
        let lat_deg = f64::from_bits(state.lat_deg_bits.load(Ordering::Relaxed));
        let lon_deg = f64::from_bits(state.lon_deg_bits.load(Ordering::Relaxed));
        let ellipsoid_alt_m = f32::from_bits(state.ellipsoid_alt_m_bits.load(Ordering::Relaxed));
        let msl_alt_m = f32::from_bits(state.msl_alt_m_bits.load(Ordering::Relaxed));
        let velocity_north_mps =
            f32::from_bits(state.velocity_north_mps_bits.load(Ordering::Relaxed));
        let velocity_east_mps =
            f32::from_bits(state.velocity_east_mps_bits.load(Ordering::Relaxed));
        let velocity_down_mps =
            f32::from_bits(state.velocity_down_mps_bits.load(Ordering::Relaxed));
        let ground_speed_mps = f32::from_bits(state.ground_speed_mps_bits.load(Ordering::Relaxed));
        let heading_motion_deg =
            f32::from_bits(state.heading_motion_deg_bits.load(Ordering::Relaxed));
        let now = ctx.now();

        // Runtime message buffers are reused; clear every optional GNSS output each tick
        // to avoid stale payload bytes being reinterpreted by the logger/exporter.
        output.0.tov = Tov::Time(now);
        output.0.clear_payload();
        output.1.tov = Tov::Time(now);
        output.1.clear_payload();
        output.2.tov = Tov::Time(now);
        output.2.clear_payload();
        output.3.tov = Tov::Time(now);
        output.3.clear_payload();
        output.4.tov = Tov::Time(now);
        output.4.clear_payload();
        output.5.tov = Tov::Time(now);
        output.5.clear_payload();
        output.6.tov = Tov::Time(now);
        output.6.clear_payload();
        output.7.tov = Tov::Time(now);
        output.7.clear_payload();
        output.8.tov = Tov::Time(now);
        output.8.clear_payload();
        output.9.tov = Tov::Time(now);
        output.9.clear_payload();

        let mut fix = GnssFixSolution {
            fix_type: GnssFixType::Fix3D,
            gnss_fix_ok: true,
            invalid_llh: false,
            num_satellites_used: GNSS_FIXED_SAT_COUNT,
            ..GnssFixSolution::default()
        };
        fix.position = GeodeticPosition::from_degrees(lat_deg, lon_deg);
        fix.height_ellipsoid = Length::new::<meter>(ellipsoid_alt_m);
        fix.height_msl = Length::new::<meter>(msl_alt_m);
        fix.velocity_north = Velocity::new::<meter_per_second>(velocity_north_mps);
        fix.velocity_east = Velocity::new::<meter_per_second>(velocity_east_mps);
        fix.velocity_down = Velocity::new::<meter_per_second>(velocity_down_mps);
        fix.ground_speed = Velocity::new::<meter_per_second>(ground_speed_mps);
        fix.heading_motion = Angle32::new::<degree>(heading_motion_deg);

        output.1.set_payload(fix);
        Ok(())
    }
}
