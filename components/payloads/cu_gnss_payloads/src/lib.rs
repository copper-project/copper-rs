#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;
use bincode::{Decode, Encode};
pub use cu_spatial_payloads::GeodeticPosition;
use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::f32::{Angle as Angle32, Length, Ratio, Time, Velocity};
use cu29::units::si::length::meter;
use cu29::units::si::ratio::ratio;
use cu29::units::si::time::second;
use cu29::units::si::velocity::meter_per_second;
use serde::{Deserialize, Serialize};

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub enum GnssFixType {
    #[default]
    NoFix,
    DeadReckoningOnly,
    Fix2D,
    Fix3D,
    GnssDeadReckoningCombined,
    TimeOnly,
    Reserved(u8),
}

impl From<u8> for GnssFixType {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::NoFix,
            1 => Self::DeadReckoningOnly,
            2 => Self::Fix2D,
            3 => Self::Fix3D,
            4 => Self::GnssDeadReckoningCombined,
            5 => Self::TimeOnly,
            other => Self::Reserved(other),
        }
    }
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub enum GnssInfoSeverity {
    Debug,
    #[default]
    Notice,
    Warning,
    Error,
    Test,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub enum GnssAckKind {
    #[default]
    Ack,
    Nak,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct GnssEpochTime {
    pub itow_ms: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub valid_date: bool,
    pub valid_time: bool,
    pub fully_resolved: bool,
    pub valid_magnetic_declination: bool,
}

/// Precision notes for GNSS payloads.
///
/// | Field(s) | UBX/source resolution | `f32` precision at relevant scale | `f64` precision | Recommendation |
/// | --- | --- | --- | --- | --- |
/// | `position.{latitude,longitude}` | `1e-7 deg` | at `32 deg`: `3.81e-6 deg` (`0.42 m` lat, `0.36 m` lon); at `180 deg`: `1.53e-5 deg` (`1.70 m` lat) | `~1e-14 deg` | `f64` |
/// | `height_*` | `1 mm` | at `1000 m`: `6.1e-5 m`; at `10000 m`: `9.8e-4 m` | `~1e-13..1e-12 m` | `f32` |
/// | `velocity_*`, `ground_speed` | `1 mm/s` | at `100 m/s`: `7.6e-6 m/s`; at `1000 m/s`: `6.1e-5 m/s` | `~1e-14..1e-13 m/s` | `f32` |
/// | `heading_*` | `1e-5 deg` | at `360 deg`: `3.05e-5 deg` | `~5.7e-14 deg` | `f32` |
/// | `magnetic_declination` | `1e-2 deg` | at `20 deg`: `1.91e-6 deg` | `~3.6e-15 deg` | `f32` |
/// | `accuracy.*` | `1 mm`, `1 mm/s`, `1e-5 deg`, `1 ns`, `0.01` | already below or close to source quantization | much smaller than source quantization | `f32` |
/// | satellite/signal angles and residuals | `1 deg`, `0.1 m` | already below source quantization | much smaller than source quantization | `f32` |
///
/// The only fields that materially benefit from `f64` in practice are the
/// absolute geodetic coordinates, because path planning and local projection
/// often subtract nearby fixes and `f32` loses sub-meter detail there.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct GnssFixSolution {
    pub fix_type: GnssFixType,
    pub gnss_fix_ok: bool,
    pub differential_solution: bool,
    pub carrier_solution: u8,
    pub invalid_llh: bool,
    pub num_satellites_used: u8,
    pub position: GeodeticPosition,
    pub height_ellipsoid: Length,
    pub height_msl: Length,
    pub velocity_north: Velocity,
    pub velocity_east: Velocity,
    pub velocity_down: Velocity,
    pub ground_speed: Velocity,
    pub heading_motion: Angle32,
    pub heading_vehicle: Angle32,
    pub magnetic_declination: Angle32,
    pub psm_state: u8,
    pub correction_age_bucket: u8,
}

impl Default for GnssFixSolution {
    fn default() -> Self {
        Self {
            fix_type: GnssFixType::NoFix,
            gnss_fix_ok: false,
            differential_solution: false,
            carrier_solution: 0,
            invalid_llh: true,
            num_satellites_used: 0,
            position: GeodeticPosition::default(),
            height_ellipsoid: Length::new::<meter>(0.0),
            height_msl: Length::new::<meter>(0.0),
            velocity_north: Velocity::new::<meter_per_second>(0.0),
            velocity_east: Velocity::new::<meter_per_second>(0.0),
            velocity_down: Velocity::new::<meter_per_second>(0.0),
            ground_speed: Velocity::new::<meter_per_second>(0.0),
            heading_motion: Angle32::new::<degree>(0.0),
            heading_vehicle: Angle32::new::<degree>(0.0),
            magnetic_declination: Angle32::new::<degree>(0.0),
            psm_state: 0,
            correction_age_bucket: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct GnssAccuracy {
    pub horizontal: Length,
    pub vertical: Length,
    pub speed: Velocity,
    pub heading: Angle32,
    pub time: Time,
    pub position_dop: Ratio,
}

impl Default for GnssAccuracy {
    fn default() -> Self {
        Self {
            horizontal: Length::new::<meter>(0.0),
            vertical: Length::new::<meter>(0.0),
            speed: Velocity::new::<meter_per_second>(0.0),
            heading: Angle32::new::<degree>(0.0),
            time: Time::new::<second>(0.0),
            position_dop: Ratio::new::<ratio>(0.0),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct GnssNavEpoch {
    pub time: GnssEpochTime,
    pub fix: GnssFixSolution,
    pub accuracy: GnssAccuracy,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct GnssSatelliteInfo {
    pub gnss_id: u8,
    pub sv_id: u8,
    pub cno_dbhz: u8,
    pub elevation: Angle32,
    pub azimuth: Angle32,
    pub pseudorange_residual: Length,
    pub quality_ind: u8,
    pub used_for_navigation: bool,
    pub health: u8,
    pub differential_correction_available: bool,
    pub pseudorange_smoothed: bool,
    pub orbit_source: u8,
    pub sbas_corr_used: bool,
    pub rtcm_corr_used: bool,
    pub slas_corr_used: bool,
    pub spartn_corr_used: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct GnssSatelliteState {
    pub itow_ms: u32,
    pub satellites: Vec<GnssSatelliteInfo>,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct GnssSatsInView {
    pub itow_ms: u32,
    pub count: u16,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct GnssSignalInfo {
    pub gnss_id: u8,
    pub sv_id: u8,
    pub signal_id: u8,
    pub frequency_id: u8,
    pub pseudorange_residual: Length,
    pub cno_dbhz: u8,
    pub quality_ind: u8,
    pub correction_source: u8,
    pub iono_model: u8,
    pub health: u8,
    pub pseudorange_smoothed: bool,
    pub pseudorange_used: bool,
    pub carrier_used: bool,
    pub doppler_used: bool,
    pub pseudorange_correction_used: bool,
    pub carrier_correction_used: bool,
    pub doppler_correction_used: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct GnssSignalState {
    pub itow_ms: u32,
    pub signals: Vec<GnssSignalInfo>,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct GnssRfBlockStatus {
    pub block_id: u8,
    pub jamming_state: u8,
    pub antenna_status: u8,
    pub antenna_power: u8,
    pub post_status: u32,
    pub noise_per_ms: u16,
    pub agc_count: u16,
    pub cw_jam_indicator: u8,
    pub i_imbalance: i8,
    pub i_magnitude: u8,
    pub q_imbalance: i8,
    pub q_magnitude: u8,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct GnssRfStatus {
    pub blocks: Vec<GnssRfBlockStatus>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct GnssInfoText {
    pub severity: GnssInfoSeverity,
    pub text: String,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
pub struct GnssCommandAck {
    pub kind: GnssAckKind,
    pub class_id: u8,
    pub msg_id: u8,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct GnssRawUbxFrame {
    pub class_id: u8,
    pub msg_id: u8,
    pub payload: Vec<u8>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub enum GnssEvent {
    #[default]
    None,
    NavEpoch(GnssNavEpoch),
    SatelliteState(GnssSatelliteState),
    SignalState(GnssSignalState),
    RfStatus(GnssRfStatus),
    InfoText(GnssInfoText),
    CommandAck(GnssCommandAck),
    RawUbx(GnssRawUbxFrame),
}
