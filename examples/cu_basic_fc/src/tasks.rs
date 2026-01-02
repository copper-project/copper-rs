#![allow(dead_code)]

use crate::GreenLed;
pub use crate::messages::{AhrsPose, Axis, AxisCommand, ControlInputs, EscStatus, RateSetpoint};
use cu_bdshot::{EscCommand, EscTelemetry};
use cu_crsf::messages::RcChannelsPayload;
use cu_sensor_payloads::ImuPayload;
use cu29::prelude::*;
use defmt::info;

const TELEMETRY_LOG_EVERY: u32 = 1000;

resources!({
    led => Owned<spin::Mutex<GreenLed>>,
});

pub struct LedBeat {
    on: bool,
    led: spin::Mutex<GreenLed>,
}

pub struct ControlSink;
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
        Ok(Self)
    }

    fn process<'i>(&mut self, _clock: &RobotClock, inputs: &Self::Input<'i>) -> CuResult<()> {
        if let Some(ctrl) = inputs.payload() {
            info!(
                "ctrl roll={} pitch={} yaw={} thr={} armed={}",
                ctrl.roll, ctrl.pitch, ctrl.yaw, ctrl.throttle, ctrl.armed
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
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        if let Some(ctrl) = input.payload() {
            let raw = if ctrl.armed { ctrl.throttle } else { 0.0 };
            let throttle = raw.clamp(0.0, 2047.0) as u16;
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

pub struct TelemetryLogger<const ESC: usize> {
    count: u32,
}

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
        Ok(Self { count: 0 })
    }

    fn process<'i>(&mut self, _clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()> {
        if let Some(payload) = input.payload() {
            self.count = self.count.wrapping_add(1);
            if self.count.is_multiple_of(TELEMETRY_LOG_EVERY) {
                if let Some(sample) = payload.sample {
                    info!("ESC{} telemetry {}", ESC, sample);
                } else {
                    info!("ESC{} telemetry missing", ESC);
                }
            }
        }
        Ok(())
    }
}

pub type TelemetryLogger0 = TelemetryLogger<0>;
pub type TelemetryLogger1 = TelemetryLogger<1>;
pub type TelemetryLogger2 = TelemetryLogger<2>;
pub type TelemetryLogger3 = TelemetryLogger<3>;

impl Freezable for ControlInputs {}
impl Freezable for RateSetpoint {}
impl Freezable for AxisCommand {}
impl Freezable for EscStatus {}
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

pub struct RcMapper;

impl Freezable for RcMapper {}

impl CuTask for RcMapper {
    type Resources<'r> = ();
    type Input<'m> = CuMsg<RcChannelsPayload>;
    type Output<'m> = CuMsg<ControlInputs>;

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
        _clock: &RobotClock,
        inputs: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let armed = true;
        // Forward the control inputs downstream when present.
        if let Some(rc) = inputs.payload() {
            output.set_payload(ControlInputs {
                roll: rc.inner().first().copied().unwrap_or(0) as f32,
                pitch: rc.inner().get(1).copied().unwrap_or(0) as f32,
                yaw: rc.inner().get(3).copied().unwrap_or(0) as f32,
                throttle: rc.inner().get(2).copied().unwrap_or(0) as f32,
                armed,
            });
        } else {
            output.clear_payload();
        }
        Ok(())
    }
}

macro_rules! attitude_pid {
    ($name:ident, $axis:expr) => {
        pub struct $name;
        impl Freezable for $name {}
        impl CuTask for $name {
            type Resources<'r> = ();
            type Input<'m> = (&'m CuMsg<AhrsPose>, &'m CuMsg<ControlInputs>);
            type Output<'m> = CuMsg<RateSetpoint>;

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
                _clock: &RobotClock,
                _input: &Self::Input<'i>,
                output: &mut Self::Output<'o>,
            ) -> CuResult<()> {
                output.set_payload(RateSetpoint {
                    axis: $axis,
                    rate: 0.0,
                });
                Ok(())
            }
        }
    };
}

attitude_pid!(AttitudePidRoll, Axis::Roll);
attitude_pid!(AttitudePidPitch, Axis::Pitch);
attitude_pid!(AttitudePidYaw, Axis::Yaw);

macro_rules! rate_pid {
    ($name:ident, $axis:expr) => {
        pub struct $name;
        impl Freezable for $name {}
        impl CuTask for $name {
            type Resources<'r> = ();
            type Input<'m> = (&'m CuMsg<RateSetpoint>, &'m CuMsg<ImuPayload>);
            type Output<'m> = CuMsg<AxisCommand>;

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
                _clock: &RobotClock,
                _input: &Self::Input<'i>,
                output: &mut Self::Output<'o>,
            ) -> CuResult<()> {
                output.set_payload(AxisCommand {
                    axis: $axis,
                    value: 0.0,
                });
                Ok(())
            }
        }
    };
}

rate_pid!(RatePidRoll, Axis::Roll);
rate_pid!(RatePidPitch, Axis::Pitch);
rate_pid!(RatePidYaw, Axis::Yaw);

// Motor mixer and health are RP2350/BDShot-specific; commented until ported to H743.
// pub struct QuadXMixer;
// impl Freezable for QuadXMixer {}
// impl CuTask for QuadXMixer { ... }

// EscHealth stub removed; mapper no longer depends on esc status.
