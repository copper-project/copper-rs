use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::f32::{AngularVelocity, ElectricPotential, Ratio};
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
    pub roll: Ratio,
    pub pitch: Ratio,
    pub yaw: Ratio,
    pub throttle: Ratio,
    pub armed: bool,
    pub mode: FlightMode,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct BodyRateSetpoint {
    pub roll: AngularVelocity,
    pub pitch: AngularVelocity,
    pub yaw: AngularVelocity,
}

#[derive(Debug, Default, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct BodyCommand {
    pub roll: Ratio,
    pub pitch: Ratio,
    pub yaw: Ratio,
}

#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct BatteryVoltage {
    pub voltage: ElectricPotential,
}
