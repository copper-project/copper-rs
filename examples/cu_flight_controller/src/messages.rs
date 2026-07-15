#![allow(dead_code)]

use cu_ahrs::AhrsPose;
use cu_gnss_payloads::GeodeticPosition;
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::f32::{Angle, AngularVelocity, ElectricPotential, Length, Ratio, Velocity};
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

#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct ControlInputs {
    pub roll: Ratio,
    pub pitch: Ratio,
    pub yaw: Ratio,
    pub throttle: Ratio,
    pub armed: bool,
    pub mode: FlightMode,
    pub auto: bool,
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

#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct GeographicHeading {
    pub heading: Angle,
}

#[derive(
    Debug, Default, Clone, Copy, Encode, Decode, Serialize, Deserialize, PartialEq, Eq, Reflect,
)]
#[repr(u8)]
pub enum AutoMissionLeg {
    #[default]
    Outbound = 0,
    Return = 1,
}

/// Fused MCU-side navigation state used by the AUTO mission and compute context.
#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct NavigationState {
    pub valid: bool,
    pub position: GeodeticPosition,
    pub altitude_msl: Length,
    pub velocity_north: Velocity,
    pub velocity_east: Velocity,
    pub velocity_down: Velocity,
    pub heading: Angle,
    pub pose: AhrsPose,
}

/// Current target in the infinite home/away AUTO mission.
#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct AutoMissionTarget {
    pub active: bool,
    pub generation: u32,
    pub leg: AutoMissionLeg,
    pub position: GeodeticPosition,
    pub altitude_msl: Length,
}

/// Fixed-size MCU -> compute payload consumed by ViTFly's context adapter.
#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct AutonomyContext {
    pub sequence: u64,
    pub mission_generation: u32,
    pub active: bool,
    pub pose: AhrsPose,
    pub desired_speed: Velocity,
}

/// Fixed-size compute -> MCU body-frame velocity command in `[forward, left, up]` axes.
#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct AutonomyVelocityCommand {
    pub context_sequence: u64,
    pub mission_generation: u32,
    pub forward: Velocity,
    pub left: Velocity,
    pub up: Velocity,
}

/// AUTO controller output kept distinct from raw RC input in the Copper graph.
#[derive(Debug, Default, Clone, Copy, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct AutonomyControl {
    pub valid: bool,
    pub controls: ControlInputs,
}
