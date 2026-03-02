#![cfg(feature = "sim")]

use cu_gnss_payloads::{GnssFixSolution, GnssFixType};
use cu_sensor_payloads::{BarometerPayload, ImuPayload, MagnetometerPayload};
use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::f32::{Angle, Length};
use cu29::units::si::length::meter;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, OnceLock};

static SIM_ACTIVITY_LED_STATE: OnceLock<Arc<AtomicBool>> = OnceLock::new();
static SIM_BATTERY_THROTTLE_BITS: OnceLock<Arc<AtomicU32>> = OnceLock::new();
static SIM_BATTERY_ARMED_STATE: OnceLock<Arc<AtomicBool>> = OnceLock::new();
const GNSS_FIXED_LAT_DEG: f32 = 30.389861114639405_f64 as f32;
const GNSS_FIXED_LON_DEG: f32 = -97.69316827380047_f64 as f32;
const GNSS_FIXED_ELLIPSOID_ALT_M: f32 = 225.0;
const GNSS_FIXED_MSL_ALT_M: f32 = 212.0;
const GNSS_FIXED_SAT_COUNT: u8 = 14;

fn sim_activity_led_state() -> Arc<AtomicBool> {
    SIM_ACTIVITY_LED_STATE
        .get_or_init(|| Arc::new(AtomicBool::new(false)))
        .clone()
}

fn sim_battery_throttle_state() -> Arc<AtomicU32> {
    SIM_BATTERY_THROTTLE_BITS
        .get_or_init(|| Arc::new(AtomicU32::new(0.0_f32.to_bits())))
        .clone()
}

fn sim_battery_armed_state() -> Arc<AtomicBool> {
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

pub fn sim_activity_led_is_on() -> bool {
    sim_activity_led_state().load(Ordering::Relaxed)
}

pub fn sim_battery_set_throttle(throttle: f32) {
    let clamped = throttle.clamp(0.0, 1.0);
    sim_battery_throttle_state().store(clamped.to_bits(), Ordering::Relaxed);
}

pub fn sim_battery_set_armed(armed: bool) {
    sim_battery_armed_state().store(armed, Ordering::Relaxed);
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
    type Output<'m> = output_msg!(GnssFixSolution);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let mut fix = GnssFixSolution {
            fix_type: GnssFixType::Fix3D,
            gnss_fix_ok: true,
            invalid_llh: false,
            num_satellites_used: GNSS_FIXED_SAT_COUNT,
            ..GnssFixSolution::default()
        };
        fix.latitude = Angle::new::<degree>(GNSS_FIXED_LAT_DEG);
        fix.longitude = Angle::new::<degree>(GNSS_FIXED_LON_DEG);
        fix.height_ellipsoid = Length::new::<meter>(GNSS_FIXED_ELLIPSOID_ALT_M);
        fix.height_msl = Length::new::<meter>(GNSS_FIXED_MSL_ALT_M);

        output.tov = Tov::Time(ctx.now());
        output.set_payload(fix);
        Ok(())
    }
}
