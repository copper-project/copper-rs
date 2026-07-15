//! Typed Copper bridge contract between the flight MCU and onboard compute.

use crate::messages::{AutonomyContext, AutonomyVelocityCommand};
use cu_zenoh_bridge::ZenohBridge;
use cu29::prelude::*;

tx_channels! {
    pub struct AutonomyTxChannels : AutonomyTxId {
        context => AutonomyContext = "flight/autonomy/context",
        command => AutonomyVelocityCommand = "flight/autonomy/command",
    }
}

rx_channels! {
    pub struct AutonomyRxChannels : AutonomyRxId {
        context => AutonomyContext = "flight/autonomy/context",
        command => AutonomyVelocityCommand = "flight/autonomy/command",
    }
}

/// Host implementation. The future MCU implementation keeps this channel contract
/// and replaces only the backend with the no-std UDP transport.
#[allow(dead_code)]
pub type AutonomyBridge = ZenohBridge<AutonomyTxChannels, AutonomyRxChannels>;
