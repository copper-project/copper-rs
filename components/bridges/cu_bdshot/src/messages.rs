use bincode::{Decode, Encode};
use cu29::bevy_reflect;
use cu29::reflect::Reflect;
use serde::{Deserialize, Serialize};

/// Telemetry payload decoded from a DSHOT ESC.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub enum DShotTelemetry {
    EncodingError,
    Erpm(u16),
    Temp(u8),
    Voltage(u8),
    Amps(u8),
    Debug1(u8),
    Debug2(u8),
    Debug3(u8),
    Event(u8),
}

/// Command sent from Copper into the ESC bridge.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct EscCommand {
    /// Raw throttle value (0 - 2047). Values >= 48 arm the ESC per DSHOT spec.
    pub throttle: u16,
    /// Whether the bridge should request telemetry for this frame.
    pub request_telemetry: bool,
}

impl Default for EscCommand {
    fn default() -> Self {
        Self {
            throttle: 0,
            request_telemetry: true,
        }
    }
}

impl EscCommand {
    pub fn disarm() -> Self {
        Self::default()
    }
}

/// Telemetry sample received from the ESC bridge.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct EscTelemetry {
    pub sample: Option<DShotTelemetry>,
}
