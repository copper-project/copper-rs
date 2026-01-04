use cu29::bincode::{Decode, Encode};
use serde::Serialize;

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
