#![allow(dead_code)]

use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use cu_ahrs::AhrsPose;
use cu_bdshot::{EscCommand, EscTelemetry};
use cu_crsf::messages::RcChannelsPayload;
use cu_sensor_payloads::ImuPayload;
use serde::Serialize;

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

impl Freezable for ControlInputs {}
impl Freezable for RateSetpoint {}
impl Freezable for AxisCommand {}
impl Freezable for EscStatus {}

pub struct RcMapper;

impl Freezable for RcMapper {}

impl CuTask for RcMapper {
    type Input<'m> = (&'m CuMsg<RcChannelsPayload>, &'m CuMsg<EscStatus>);
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
        let armed = inputs.1.payload().map(|s| !s.fault).unwrap_or(true);
        let control = if let Some(rc) = inputs.0.payload() {
            ControlInputs {
                roll: rc.inner().get(0).copied().unwrap_or_default() as f32,
                pitch: rc.inner().get(1).copied().unwrap_or_default() as f32,
                yaw: rc.inner().get(3).copied().unwrap_or_default() as f32,
                throttle: rc.inner().get(2).copied().unwrap_or_default() as f32,
                armed,
            }
        } else {
            ControlInputs {
                armed,
                ..ControlInputs::default()
            }
        };

        output.set_payload(control);
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

pub struct QuadXMixer;

impl Freezable for QuadXMixer {}

impl CuTask for QuadXMixer {
    type Input<'m> = (
        &'m CuMsg<AxisCommand>,
        &'m CuMsg<AxisCommand>,
        &'m CuMsg<AxisCommand>,
        &'m CuMsg<ControlInputs>,
    );
    type Output<'m> = CuMsg<EscCommand>;

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let throttle = input.3.payload().map(|c| c.throttle).unwrap_or_default();
        output.set_payload(EscCommand {
            throttle: throttle as u16,
            request_telemetry: true,
        });
        Ok(())
    }
}

pub struct EscHealth;

impl Freezable for EscHealth {}

impl CuTask for EscHealth {
    type Input<'m> = (
        &'m CuMsg<EscTelemetry>,
        &'m CuMsg<EscTelemetry>,
        &'m CuMsg<EscTelemetry>,
        &'m CuMsg<EscTelemetry>,
    );
    type Output<'m> = CuMsg<EscStatus>;

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        _input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        output.set_payload(EscStatus { fault: false });
        Ok(())
    }
}
