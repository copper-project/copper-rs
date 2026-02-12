#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;
use bincode::de::{BorrowDecode, BorrowDecoder, Decoder};
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use uom::si::angle::{degree, radian};
use uom::si::f32::{Angle, Length, Time, Velocity};
use uom::si::length::meter;
use uom::si::time::second;
use uom::si::velocity::meter_per_second;

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[reflect(opaque, from_reflect = false)]
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
#[reflect(opaque, from_reflect = false)]
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
#[reflect(opaque, from_reflect = false)]
pub enum GnssAckKind {
    #[default]
    Ack,
    Nak,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[reflect(opaque, from_reflect = false)]
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

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct GnssFixSolution {
    pub fix_type: GnssFixType,
    pub gnss_fix_ok: bool,
    pub differential_solution: bool,
    pub carrier_solution: u8,
    pub invalid_llh: bool,
    pub num_satellites_used: u8,
    pub longitude: Angle,
    pub latitude: Angle,
    pub height_ellipsoid: Length,
    pub height_msl: Length,
    pub velocity_north: Velocity,
    pub velocity_east: Velocity,
    pub velocity_down: Velocity,
    pub ground_speed: Velocity,
    pub heading_motion: Angle,
    pub heading_vehicle: Angle,
    pub magnetic_declination: Angle,
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
            longitude: Angle::new::<degree>(0.0),
            latitude: Angle::new::<degree>(0.0),
            height_ellipsoid: Length::new::<meter>(0.0),
            height_msl: Length::new::<meter>(0.0),
            velocity_north: Velocity::new::<meter_per_second>(0.0),
            velocity_east: Velocity::new::<meter_per_second>(0.0),
            velocity_down: Velocity::new::<meter_per_second>(0.0),
            ground_speed: Velocity::new::<meter_per_second>(0.0),
            heading_motion: Angle::new::<degree>(0.0),
            heading_vehicle: Angle::new::<degree>(0.0),
            magnetic_declination: Angle::new::<degree>(0.0),
            psm_state: 0,
            correction_age_bucket: 0,
        }
    }
}

impl Encode for GnssFixSolution {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.fix_type, encoder)?;
        Encode::encode(&self.gnss_fix_ok, encoder)?;
        Encode::encode(&self.differential_solution, encoder)?;
        Encode::encode(&self.carrier_solution, encoder)?;
        Encode::encode(&self.invalid_llh, encoder)?;
        Encode::encode(&self.num_satellites_used, encoder)?;
        Encode::encode(&self.longitude.get::<radian>(), encoder)?;
        Encode::encode(&self.latitude.get::<radian>(), encoder)?;
        Encode::encode(&self.height_ellipsoid.get::<meter>(), encoder)?;
        Encode::encode(&self.height_msl.get::<meter>(), encoder)?;
        Encode::encode(&self.velocity_north.get::<meter_per_second>(), encoder)?;
        Encode::encode(&self.velocity_east.get::<meter_per_second>(), encoder)?;
        Encode::encode(&self.velocity_down.get::<meter_per_second>(), encoder)?;
        Encode::encode(&self.ground_speed.get::<meter_per_second>(), encoder)?;
        Encode::encode(&self.heading_motion.get::<radian>(), encoder)?;
        Encode::encode(&self.heading_vehicle.get::<radian>(), encoder)?;
        Encode::encode(&self.magnetic_declination.get::<radian>(), encoder)?;
        Encode::encode(&self.psm_state, encoder)?;
        Encode::encode(&self.correction_age_bucket, encoder)?;
        Ok(())
    }
}

impl<Context> Decode<Context> for GnssFixSolution {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            fix_type: Decode::decode(decoder)?,
            gnss_fix_ok: Decode::decode(decoder)?,
            differential_solution: Decode::decode(decoder)?,
            carrier_solution: Decode::decode(decoder)?,
            invalid_llh: Decode::decode(decoder)?,
            num_satellites_used: Decode::decode(decoder)?,
            longitude: Angle::new::<radian>(Decode::decode(decoder)?),
            latitude: Angle::new::<radian>(Decode::decode(decoder)?),
            height_ellipsoid: Length::new::<meter>(Decode::decode(decoder)?),
            height_msl: Length::new::<meter>(Decode::decode(decoder)?),
            velocity_north: Velocity::new::<meter_per_second>(Decode::decode(decoder)?),
            velocity_east: Velocity::new::<meter_per_second>(Decode::decode(decoder)?),
            velocity_down: Velocity::new::<meter_per_second>(Decode::decode(decoder)?),
            ground_speed: Velocity::new::<meter_per_second>(Decode::decode(decoder)?),
            heading_motion: Angle::new::<radian>(Decode::decode(decoder)?),
            heading_vehicle: Angle::new::<radian>(Decode::decode(decoder)?),
            magnetic_declination: Angle::new::<radian>(Decode::decode(decoder)?),
            psm_state: Decode::decode(decoder)?,
            correction_age_bucket: Decode::decode(decoder)?,
        })
    }
}

impl<'de, Context> BorrowDecode<'de, Context> for GnssFixSolution {
    fn borrow_decode<D: BorrowDecoder<'de, Context = Context>>(
        decoder: &mut D,
    ) -> Result<Self, DecodeError> {
        <Self as Decode<Context>>::decode(decoder)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct GnssAccuracy {
    pub horizontal: Length,
    pub vertical: Length,
    pub speed: Velocity,
    pub heading: Angle,
    pub time: Time,
    pub position_dop: f32,
}

impl Default for GnssAccuracy {
    fn default() -> Self {
        Self {
            horizontal: Length::new::<meter>(0.0),
            vertical: Length::new::<meter>(0.0),
            speed: Velocity::new::<meter_per_second>(0.0),
            heading: Angle::new::<degree>(0.0),
            time: Time::new::<second>(0.0),
            position_dop: 0.0,
        }
    }
}

impl Encode for GnssAccuracy {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.horizontal.get::<meter>(), encoder)?;
        Encode::encode(&self.vertical.get::<meter>(), encoder)?;
        Encode::encode(&self.speed.get::<meter_per_second>(), encoder)?;
        Encode::encode(&self.heading.get::<radian>(), encoder)?;
        Encode::encode(&self.time.get::<second>(), encoder)?;
        Encode::encode(&self.position_dop, encoder)?;
        Ok(())
    }
}

impl<Context> Decode<Context> for GnssAccuracy {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            horizontal: Length::new::<meter>(Decode::decode(decoder)?),
            vertical: Length::new::<meter>(Decode::decode(decoder)?),
            speed: Velocity::new::<meter_per_second>(Decode::decode(decoder)?),
            heading: Angle::new::<radian>(Decode::decode(decoder)?),
            time: Time::new::<second>(Decode::decode(decoder)?),
            position_dop: Decode::decode(decoder)?,
        })
    }
}

impl<'de, Context> BorrowDecode<'de, Context> for GnssAccuracy {
    fn borrow_decode<D: BorrowDecoder<'de, Context = Context>>(
        decoder: &mut D,
    ) -> Result<Self, DecodeError> {
        <Self as Decode<Context>>::decode(decoder)
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct GnssNavEpoch {
    pub time: GnssEpochTime,
    pub fix: GnssFixSolution,
    pub accuracy: GnssAccuracy,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[reflect(opaque, from_reflect = false)]
pub struct GnssSatelliteInfo {
    pub gnss_id: u8,
    pub sv_id: u8,
    pub cno_dbhz: u8,
    pub elevation_deg: i8,
    pub azimuth_deg: i16,
    pub pseudorange_residual_dm: i16,
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
#[reflect(opaque, from_reflect = false)]
pub struct GnssSatelliteState {
    pub itow_ms: u32,
    pub satellites: Vec<GnssSatelliteInfo>,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[reflect(opaque, from_reflect = false)]
pub struct GnssSatsInView {
    pub itow_ms: u32,
    pub count: u16,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[reflect(opaque, from_reflect = false)]
pub struct GnssSignalInfo {
    pub gnss_id: u8,
    pub sv_id: u8,
    pub signal_id: u8,
    pub frequency_id: u8,
    pub pseudorange_residual_dm: i16,
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
#[reflect(opaque, from_reflect = false)]
pub struct GnssSignalState {
    pub itow_ms: u32,
    pub signals: Vec<GnssSignalInfo>,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[reflect(opaque, from_reflect = false)]
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
#[reflect(opaque, from_reflect = false)]
pub struct GnssRfStatus {
    pub blocks: Vec<GnssRfBlockStatus>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct GnssInfoText {
    pub severity: GnssInfoSeverity,
    pub text: String,
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect,
)]
#[reflect(opaque, from_reflect = false)]
pub struct GnssCommandAck {
    pub kind: GnssAckKind,
    pub class_id: u8,
    pub msg_id: u8,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct GnssRawUbxFrame {
    pub class_id: u8,
    pub msg_id: u8,
    pub payload: Vec<u8>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
#[reflect(opaque, from_reflect = false)]
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
