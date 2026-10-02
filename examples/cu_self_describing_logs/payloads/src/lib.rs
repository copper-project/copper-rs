//! Payload definitions shared by the host packager and target application.

use cu29::bincode::Decode;
use cu29::bincode::Encode;
use cu29::prelude::*;
use cu29::units::si::f32::Length;
use cu29::units::si::f32::Velocity;

#[derive(Clone, Debug, Default, Serialize, Deserialize, Encode, Decode, Reflect)]
#[bincode(crate = "cu29::bincode")]
pub enum SensorState {
    #[default]
    Ready,
    Calibrating {
        remaining: u16,
    },
    Fault(String),
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, Encode, Decode, Reflect)]
#[bincode(crate = "cu29::bincode")]
pub struct WheelSample {
    pub ticks: u32,
    pub distance: Length,
    pub speed: Velocity,
    pub timestamp: CuTime,
    pub acceleration: [f32; 3],
    pub temperatures: Vec<f32>,
    pub state: SensorState,
}
