use cu29::bincode::{Decode, Encode};
use cu29::reflect::Reflect;
use serde::{Deserialize, Serialize};

#[derive(
    Debug, Default, Clone, Copy, Encode, Decode, Serialize, Deserialize, PartialEq, Eq, Reflect,
)]
#[repr(u8)]
pub enum FlightMode {
    #[default]
    Angle = 0,
    Acro = 1,
    PositionHold = 2,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct ControlInputs {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub armed: bool,
    pub mode: FlightMode,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct BodyRateSetpoint {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct BodyCommand {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct BatteryVoltage {
    /// Battery voltage in 0.01V steps.
    pub centivolts: u16,
}
