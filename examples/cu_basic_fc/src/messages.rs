use cu29::bincode::{Decode, Encode};
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
