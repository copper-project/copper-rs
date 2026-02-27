#![cfg(feature = "sim")]

use cu_sensor_payloads::{BarometerPayload, ImuPayload, MagnetometerPayload};
use cu29::prelude::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, OnceLock};

static SIM_ACTIVITY_LED_STATE: OnceLock<Arc<AtomicBool>> = OnceLock::new();

fn sim_activity_led_state() -> Arc<AtomicBool> {
    SIM_ACTIVITY_LED_STATE
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

#[derive(Clone, Reflect)]
pub struct SimBatteryAdc {
    base_voltage: f32,
    phase: f32,
}

impl Default for SimBatteryAdc {
    fn default() -> Self {
        Self {
            base_voltage: 16.0,
            phase: 0.0,
        }
    }
}

impl SimBatteryAdc {
    pub fn read_voltage_v(&mut self) -> f32 {
        // Keep a small deterministic ripple so downstream battery logic sees live updates.
        self.phase += 0.05;
        let ripple = self.phase.sin() * 0.2;
        (self.base_voltage + ripple).max(0.0)
    }
}

pub fn sim_battery_adc(base_voltage: f32) -> SimBatteryAdc {
    SimBatteryAdc {
        base_voltage,
        phase: 0.0,
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
