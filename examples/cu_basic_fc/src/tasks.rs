#![allow(dead_code)]

use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use cu_crsf::messages::RcChannelsPayload;
use cu_sensor_payloads::ImuPayload;
use defmt::info;
use serde::Serialize;

// Placeholder AHRS pose until the sensor/estimator is brought up on STM32.
#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, PartialEq, Eq)]
pub struct AhrsPose;

#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, PartialEq, Eq)]
#[repr(u8)]
pub enum Axis {
    #[default]
    Roll = 0,
    Pitch = 1,
    Yaw = 2,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize)]
pub struct ControlInputs {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub armed: bool,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize)]
pub struct RateSetpoint {
    pub axis: Axis,
    pub rate: f32,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize)]
pub struct AxisCommand {
    pub axis: Axis,
    pub value: f32,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize)]
pub struct EscStatus {
    pub fault: bool,
}

pub struct ControlSink;
impl Freezable for ControlSink {}
impl CuSinkTask for ControlSink {
    type Input<'m> = CuMsg<ControlInputs>;

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process<'i>(&mut self, _clock: &RobotClock, inputs: &Self::Input<'i>) -> CuResult<()> {
        if let Some(ctrl) = inputs.payload() {
            info!(
                "ctrl roll={} pitch={} yaw={} thr={} armed={}",
                ctrl.roll,
                ctrl.pitch,
                ctrl.yaw,
                ctrl.throttle,
                ctrl.armed
            );
        }
        Ok(())
    }
}

impl Freezable for ControlInputs {}
impl Freezable for RateSetpoint {}
impl Freezable for AxisCommand {}
impl Freezable for EscStatus {}

pub struct RcMapper;

impl Freezable for RcMapper {}

impl CuTask for RcMapper {
    type Input<'m> = CuMsg<RcChannelsPayload>;
    type Output<'m> = CuMsg<ControlInputs>;

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
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
            type Input<'m> = (&'m CuMsg<AhrsPose>, &'m CuMsg<ControlInputs>);
            type Output<'m> = CuMsg<RateSetpoint>;

            fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
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
            type Input<'m> = (&'m CuMsg<RateSetpoint>, &'m CuMsg<ImuPayload>);
            type Output<'m> = CuMsg<AxisCommand>;

            fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
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
