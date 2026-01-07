//! MSP structures

use packed_struct::derive::{PackedStruct, PrimitiveEnum};
use serde::{Deserialize, Serialize};

use crate::MspPacketDirection::{FromFlightController, ToFlightController};
use crate::commands::MspCommandCode;
use crate::{MspPacket, MspPacketData};

use alloc::{borrow::ToOwned, string::String, vec::Vec};

#[cfg(feature = "bincode")]
use bincode::{Decode, Encode};
use packed_struct::types::bits::ByteArray;
use packed_struct::{PackedStruct, PackingError, PrimitiveEnum};
use smallvec::SmallVec;

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
pub struct MspApiVersion {
    pub protocol_version: u8,
    pub api_version_major: u8,
    pub api_version_minor: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
pub struct MspFlightControllerVariant {
    pub identifier: [u8; 4],
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
pub struct MspFlightControllerVersion {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspBoardInfo {
    pub board_id: [u8; 4],
    pub hardware_revision: u16,
    pub fc_type: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
pub struct MspBuildInfo {
    pub date_str: [u8; 11],
    pub time_str: [u8; 8],
    pub git_str: [u8; 7],
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
pub struct MspUniqueId {
    pub uid: [u8; 12],
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspAvailableSensors {
    #[packed_field(bits = "2")]
    pub sonar: bool,
    #[packed_field(bits = "4")]
    pub gps: bool,
    #[packed_field(bits = "5")]
    pub mag: bool,
    #[packed_field(bits = "6")]
    pub baro: bool,
    #[packed_field(bits = "7")]
    pub acc: bool,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(Serialize, Deserialize, Debug, Copy, Clone, Default, PartialEq, Eq)]
pub struct MspStatusSensors {
    pub acc: bool,
    pub baro: bool,
    pub mag: bool,
    pub gps: bool,
    pub rangefinder: bool,
    pub gyro: bool,
    pub optical_flow: bool,
}

impl MspStatusSensors {
    pub fn from_bits(bits: u16) -> Self {
        Self {
            acc: bits & (1 << 0) != 0,
            baro: bits & (1 << 1) != 0,
            mag: bits & (1 << 2) != 0,
            gps: bits & (1 << 3) != 0,
            rangefinder: bits & (1 << 4) != 0,
            gyro: bits & (1 << 5) != 0,
            optical_flow: bits & (1 << 6) != 0,
        }
    }

    pub fn to_bits(self) -> u16 {
        (self.acc as u16)
            | (self.baro as u16) << 1
            | (self.mag as u16) << 2
            | (self.gps as u16) << 3
            | (self.rangefinder as u16) << 4
            | (self.gyro as u16) << 5
            | (self.optical_flow as u16) << 6
    }
}

impl From<u16> for MspStatusSensors {
    fn from(bits: u16) -> Self {
        Self::from_bits(bits)
    }
}

impl From<MspStatusSensors> for u16 {
    fn from(value: MspStatusSensors) -> Self {
        value.to_bits()
    }
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(Serialize, Deserialize, Debug, Clone, Default, PartialEq)]
pub struct MspStatus {
    pub cycle_time: u16,
    pub i2c_errors: u16,
    pub sensors: MspStatusSensors,
    pub flight_mode_flags: u32,
    pub current_pid_profile_index: u8,
    pub average_system_load_percent: u16,
    pub gyro_cycle_time: u16,
    pub extra_flight_mode_flags: Vec<u8>,
    pub arming_disable_flags_count: u8,
    pub arming_disable_flags: u32,
    pub config_state_flags: u8,
    pub core_temp_celsius: u16,
    pub control_rate_profile_count: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(Serialize, Deserialize, Debug, Clone, Default, PartialEq)]
pub struct MspStatusEx {
    pub cycle_time: u16,
    pub i2c_errors: u16,
    pub sensors: MspStatusSensors,
    pub flight_mode_flags: u32,
    pub current_pid_profile_index: u8,
    pub average_system_load_percent: u16,
    pub max_profile_count: u8,
    pub current_control_rate_profile_index: u8,
    pub extra_flight_mode_flags: Vec<u8>,
    pub arming_disable_flags_count: u8,
    pub arming_disable_flags: u32,
    pub config_state_flags: u8,
    pub core_temp_celsius: u16,
    pub control_rate_profile_count: u8,
}

impl MspStatus {
    pub fn from_bytes(data: &[u8]) -> Result<Self, PackingError> {
        let mut offset = 0;
        let cycle_time = read_u16(data, &mut offset)?;
        let i2c_errors = read_u16(data, &mut offset)?;
        let sensors = MspStatusSensors::from(read_u16(data, &mut offset)?);
        let flight_mode_flags = read_u32(data, &mut offset)?;
        let current_pid_profile_index = read_u8(data, &mut offset)?;
        let average_system_load_percent = read_u16(data, &mut offset)?;
        let gyro_cycle_time = read_u16(data, &mut offset)?;
        let extra_flight_mode_flags_len = read_u8(data, &mut offset)? as usize;
        let extra_flight_mode_flags = read_bytes(data, &mut offset, extra_flight_mode_flags_len)?;
        let arming_disable_flags_count = read_u8(data, &mut offset)?;
        let arming_disable_flags = read_u32(data, &mut offset)?;
        let config_state_flags = read_u8(data, &mut offset)?;
        let core_temp_celsius = read_u16(data, &mut offset)?;
        let control_rate_profile_count = read_u8(data, &mut offset)?;

        Ok(Self {
            cycle_time,
            i2c_errors,
            sensors,
            flight_mode_flags,
            current_pid_profile_index,
            average_system_load_percent,
            gyro_cycle_time,
            extra_flight_mode_flags,
            arming_disable_flags_count,
            arming_disable_flags,
            config_state_flags,
            core_temp_celsius,
            control_rate_profile_count,
        })
    }

    pub fn to_packet_data(&self) -> Result<MspPacketData, PackingError> {
        if self.extra_flight_mode_flags.len() > 15 {
            return Err(PackingError::InvalidValue);
        }
        let mut data = SmallVec::<[u8; 256]>::new();
        data.extend_from_slice(&self.cycle_time.to_le_bytes());
        data.extend_from_slice(&self.i2c_errors.to_le_bytes());
        data.extend_from_slice(&u16::from(self.sensors).to_le_bytes());
        data.extend_from_slice(&self.flight_mode_flags.to_le_bytes());
        data.push(self.current_pid_profile_index);
        data.extend_from_slice(&self.average_system_load_percent.to_le_bytes());
        data.extend_from_slice(&self.gyro_cycle_time.to_le_bytes());
        data.push(self.extra_flight_mode_flags.len() as u8);
        data.extend_from_slice(&self.extra_flight_mode_flags);
        data.push(self.arming_disable_flags_count);
        data.extend_from_slice(&self.arming_disable_flags.to_le_bytes());
        data.push(self.config_state_flags);
        data.extend_from_slice(&self.core_temp_celsius.to_le_bytes());
        data.push(self.control_rate_profile_count);

        Ok(MspPacketData(data))
    }
}

impl MspStatusEx {
    pub fn from_bytes(data: &[u8]) -> Result<Self, PackingError> {
        let mut offset = 0;
        let cycle_time = read_u16(data, &mut offset)?;
        let i2c_errors = read_u16(data, &mut offset)?;
        let sensors = MspStatusSensors::from(read_u16(data, &mut offset)?);
        let flight_mode_flags = read_u32(data, &mut offset)?;
        let current_pid_profile_index = read_u8(data, &mut offset)?;
        let average_system_load_percent = read_u16(data, &mut offset)?;
        let max_profile_count = read_u8(data, &mut offset)?;
        let current_control_rate_profile_index = read_u8(data, &mut offset)?;
        let extra_flight_mode_flags_len = read_u8(data, &mut offset)? as usize;
        let extra_flight_mode_flags = read_bytes(data, &mut offset, extra_flight_mode_flags_len)?;
        let arming_disable_flags_count = read_u8(data, &mut offset)?;
        let arming_disable_flags = read_u32(data, &mut offset)?;
        let config_state_flags = read_u8(data, &mut offset)?;
        let core_temp_celsius = read_u16(data, &mut offset)?;
        let control_rate_profile_count = read_u8(data, &mut offset)?;

        Ok(Self {
            cycle_time,
            i2c_errors,
            sensors,
            flight_mode_flags,
            current_pid_profile_index,
            average_system_load_percent,
            max_profile_count,
            current_control_rate_profile_index,
            extra_flight_mode_flags,
            arming_disable_flags_count,
            arming_disable_flags,
            config_state_flags,
            core_temp_celsius,
            control_rate_profile_count,
        })
    }

    pub fn to_packet_data(&self) -> Result<MspPacketData, PackingError> {
        if self.extra_flight_mode_flags.len() > 15 {
            return Err(PackingError::InvalidValue);
        }
        let mut data = SmallVec::<[u8; 256]>::new();
        data.extend_from_slice(&self.cycle_time.to_le_bytes());
        data.extend_from_slice(&self.i2c_errors.to_le_bytes());
        data.extend_from_slice(&u16::from(self.sensors).to_le_bytes());
        data.extend_from_slice(&self.flight_mode_flags.to_le_bytes());
        data.push(self.current_pid_profile_index);
        data.extend_from_slice(&self.average_system_load_percent.to_le_bytes());
        data.push(self.max_profile_count);
        data.push(self.current_control_rate_profile_index);
        data.push(self.extra_flight_mode_flags.len() as u8);
        data.extend_from_slice(&self.extra_flight_mode_flags);
        data.push(self.arming_disable_flags_count);
        data.extend_from_slice(&self.arming_disable_flags.to_le_bytes());
        data.push(self.config_state_flags);
        data.extend_from_slice(&self.core_temp_celsius.to_le_bytes());
        data.push(self.control_rate_profile_count);

        Ok(MspPacketData(data))
    }
}

fn read_u8(data: &[u8], offset: &mut usize) -> Result<u8, PackingError> {
    if *offset + 1 > data.len() {
        return Err(PackingError::BufferSizeMismatch {
            expected: *offset + 1,
            actual: data.len(),
        });
    }
    let value = data[*offset];
    *offset += 1;
    Ok(value)
}

fn read_u16(data: &[u8], offset: &mut usize) -> Result<u16, PackingError> {
    if *offset + 2 > data.len() {
        return Err(PackingError::BufferSizeMismatch {
            expected: *offset + 2,
            actual: data.len(),
        });
    }
    let value = u16::from_le_bytes([data[*offset], data[*offset + 1]]);
    *offset += 2;
    Ok(value)
}

fn read_u32(data: &[u8], offset: &mut usize) -> Result<u32, PackingError> {
    if *offset + 4 > data.len() {
        return Err(PackingError::BufferSizeMismatch {
            expected: *offset + 4,
            actual: data.len(),
        });
    }
    let value = u32::from_le_bytes([
        data[*offset],
        data[*offset + 1],
        data[*offset + 2],
        data[*offset + 3],
    ]);
    *offset += 4;
    Ok(value)
}

fn read_bytes(data: &[u8], offset: &mut usize, len: usize) -> Result<Vec<u8>, PackingError> {
    if *offset + len > data.len() {
        return Err(PackingError::BufferSizeMismatch {
            expected: *offset + len,
            actual: data.len(),
        });
    }
    let bytes = data[*offset..*offset + len].to_vec();
    *offset += len;
    Ok(bytes)
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspBfConfig {
    pub mixer_configuration: u8,
    pub features: u32,
    pub serial_rx_provider: u8,
    pub board_align_roll: i16,
    pub board_align_pitch: i16,
    pub board_align_yaw: i16,
    pub current_scale: i16,
    pub current_offset: i16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspRawImu {
    pub acc_x: i16,
    pub acc_y: i16,
    pub acc_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
    pub mag_x: i16,
    pub mag_y: i16,
    pub mag_z: i16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspDataFlashSummaryReply {
    #[packed_field(bits = "6")]
    pub supported: bool,
    #[packed_field(bits = "7")]
    pub ready: bool,
    pub sectors: u32,
    pub total_size_bytes: u32,
    pub used_size_bytes: u32,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspDataFlashReply {
    pub read_address: u32,
    // pub payload: Vec<u8>, // TODO: packed_struct should support dynamic size too the end
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "6", endian = "lsb", bit_numbering = "msb0")]
pub struct MspDataFlashRead {
    pub read_address: u32,
    pub read_length: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspAccTrim {
    pub pitch: u16,
    pub roll: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspIdent {
    pub version: u8,
    pub mixer_mode: u8,
    pub protocol_version: u8,
    pub capability: u32,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspMisc {
    pub rx_mid_rc: u16,
    pub min_throttle: u16,
    pub max_throttle: u16,
    pub min_command: u16,
    pub failsafe_throttle: u16,

    pub gps_type: u8,
    pub gps_baudrate: u8,
    pub gps_sbas_mode: u8,

    pub current_meter_output: u8,
    pub rssi_channel: u8,
    pub null1: u8,

    pub compass_mag_declination: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspAttitude {
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspAltitude {
    /// [centimeters]
    pub altitude: i32,
    /// variometer [cm/s]
    pub vario: i16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspBatteryConfig {
    pub vbat_min_cell_voltage: u8,
    pub vbat_max_cell_voltage: u8,
    pub vbat_warning_cell_voltage: u8,
    pub battery_capacity: u16,
    pub voltage_meter_source: u8,
    pub current_meter_source: u8,
    pub vbat_min_cell_voltage_mv: u16,
    pub vbat_max_cell_voltage_mv: u16,
    pub vbat_warning_cell_voltage_mv: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspVoltageMeterConfig {
    pub sensor_count: u8,
    pub subframe_len: u8,
    pub id: u8,
    pub sensor_type: u8,
    pub vbat_scale: u8,
    pub vbat_res_div_val: u8,
    pub vbat_res_div_mult: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspAnalog {
    pub battery_voltage: u8,
    pub mah_drawn: u16,
    pub rssi: u16,
    /// Current in 0.01A steps, range is -320A to 320A
    pub amperage: i16,
    /// Battery voltage in 0.01V steps
    pub battery_voltage_mv: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspRssiConfig {
    pub rssi_channel: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
pub struct MspVoltageMeter {
    pub id: u8,
    pub value: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspCurrentMeter {
    pub id: u8,
    pub mah_drawn: u16,
    /// In 0.001A steps (mA)
    pub amperage: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspBatteryState {
    pub battery_cell_count: u8,
    /// mAh
    pub battery_capacity: u16,

    pub battery_voltage: u8,
    pub mah_drawn: u16,
    /// 0.01A
    pub amperage: i16,

    pub alerts: u8,
    /// Battery voltage in 0.01V steps
    pub battery_voltage_mv: u16,
}

impl MspBatteryState {
    pub fn cell_voltage(&self) -> f32 {
        self.battery_voltage as f32 / (10 * self.battery_cell_count) as f32
    }
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspRcTuning {
    pub rc_rate8: u8,
    pub rc_expo8: u8,

    pub rate_roll: u8,
    pub rate_pitch: u8,
    pub rate_yaw: u8,

    pub dyn_thr_pid: u8,
    pub thr_mid8: u8,
    pub thr_expo8: u8,
    pub tpa_breakpoint: u16,
    pub rc_yaw_expo8: u8,
    pub rc_yaw_rate8: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspRxConfig {
    pub serialrx_provider: u8,
    pub maxcheck: u16,
    pub midrc: u16,
    pub mincheck: u16,
    pub spektrum_sat_bind: u8,
    pub rx_min_usec: u16,
    pub rx_max_usec: u16,
    pub rc_interpolation: u8,
    pub rc_interpolation_interval: u8,
    pub air_mode_activate_threshold: u16,
    pub rx_spi_protocol: u8,
    pub rx_spi_id: u32,
    pub rx_spi_rf_channel_count: u8,
    pub fpv_cam_angle_degrees: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspRcChannelValue {
    pub value: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq, Default)]
pub enum MspRcChannel {
    /// Ailerons
    #[default]
    Roll = 0,
    /// Elevators
    Pitch = 1,
    /// Rudder
    Yaw = 2,
    Throttle = 3,
    Aux1 = 4,
    Aux2 = 5,
    Aux3 = 6,
    Aux4 = 7,
    Aux5 = 8,
    Aux6 = 9,
    Aux7 = 10,
    Aux8 = 11,
    Aux9 = 12,
    Aux10 = 13,
    Aux11 = 14,
    Aux12 = 15,
    Aux13 = 16,
    Aux14 = 17,
    Aux15 = 18,
    Aux16 = 19,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
pub struct MspRcMappedChannel {
    #[packed_field(size_bits = "8", ty = "enum")]
    pub channel: MspRcChannel,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
pub struct MspFeatures {
    pub features: [bool; 32],
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspMotor {
    pub motors: [u16; 8],
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspMotor3DConfig {
    pub deadband_3d_low: u16,
    pub deadband_3d_high: u16,
    pub neutral_3d: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspMotorConfig {
    pub min_throttle: u16,
    pub max_throttle: u16,
    pub min_command: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspRcDeadband {
    pub deadband: u8,
    pub yaw_deadband: u8,
    pub alt_hold_deadband: u8,
    pub deadband_3d_throttle: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspSensorAlignment {
    pub gyro_alignment: u8,
    pub acc_alignment: u8,
    pub mag_alignment: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspAdvancedConfig {
    pub gyro_sync_denom: u8,
    pub pid_process_denom: u8,
    pub use_unsynced_pwm: u8,
    pub motor_pwm_protocol: u8,
    pub motor_pwm_rate: u16,
    pub digital_idle_offset_percent: u16,
    pub gyro_use_32khz: u8,
    pub motor_pwm_inversion: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspFilterConfig {
    pub gyro_soft_lpf_hz: u8,
    pub dterm_lpf_hz: u16,
    pub yaw_lpf_hz: u16,
    pub gyro_soft_notch_hz_1: u16,
    pub gyro_soft_notch_cutoff_1: u16,
    pub dterm_notch_hz: u16,
    pub dterm_notch_cutoff: u16,
    pub gyro_soft_notch_hz_2: u16,
    pub gyro_soft_notch_cutoff_2: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspPidAdvanced {
    pub _r1: u16,
    pub _r2: u16,
    pub _r3: u16,
    pub _r4: u8,
    pub vbat_pid_compensation: u8,
    pub setpoint_relax_ratio: u8,
    pub dterm_setpoint_weight: u8,
    pub _r5: u8,
    pub _r6: u8,
    pub _r7: u8,
    pub rate_accel_limit: u16,
    pub yaw_rate_accel_limit: u16,
    pub level_angle_limit: u8,
    pub level_sensitivity: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspSensorConfig {
    pub acc_hardware: u8,
    pub baro_hardware: u8,
    pub mag_hardware: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspServos {
    pub servos: [u16; 8],
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "14", endian = "lsb", bit_numbering = "msb0")]
pub struct MspServoConfig {
    pub min: u16,
    pub max: u16,
    pub middle: u16,
    pub rate: i8,
    pub unused1: u8,
    pub unused2: u8,
    pub forward_from_channel: u8, // Deprecated, set to 255 for backward compatibility
    pub reverse_input: u32, // Deprecated, Input reversing is not required since it can be done on mixer level
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetServoConfig {
    pub index: u8,
    #[packed_field(size_bytes = "14")]
    pub servo_config: MspServoConfig,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspMixerConfig {
    #[packed_field(size_bits = "8", ty = "enum")]
    pub mixer_mode: MixerMode,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "4", endian = "lsb", bit_numbering = "msb0")]
pub struct MspModeRange {
    pub box_id: u8,
    #[packed_field(size_bits = "8", ty = "enum")]
    pub aux_channel_index: MspRcChannel,
    pub start_step: u8,
    pub end_step: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode, Default))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(bytes = "5", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetModeRange {
    pub index: u8,
    #[packed_field(size_bytes = "4")]
    pub mode_range: MspModeRange,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq, Default)]
pub enum MixerMode {
    Tri = 1,
    QuadPlus = 2,
    #[default]
    QuadX = 3,
    Bicopter = 4,
    Gimbal = 5,
    Y6 = 6,
    Hex6 = 7,
    FlyingWing = 8,
    Y4 = 9,
    Hex6X = 10,
    OctoX8 = 11,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "8", endian = "lsb", bit_numbering = "msb0")]
pub struct MspMotorMixer {
    pub throttle: u16,
    pub roll: u16,
    pub pitch: u16,
    pub yaw: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "9", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetMotorMixer {
    pub index: u8,
    #[packed_field(size_bytes = "8")]
    pub motor_mixer: MspMotorMixer,
}

pub const MSP_DP_HEARTBEAT: u8 = 0;
pub const MSP_DP_RELEASE: u8 = 1;
pub const MSP_DP_CLEAR_SCREEN: u8 = 2;
pub const MSP_DP_WRITE_STRING: u8 = 3;
pub const MSP_DP_DRAW_SCREEN: u8 = 4;
pub const MSP_DP_OPTIONS: u8 = 5;
pub const MSP_DP_SYS: u8 = 6;
pub const MSP_DP_FONTCHAR_WRITE: u8 = 7;

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(Debug, Serialize, Deserialize, Clone, Default, PartialEq, Eq)]
pub struct MspDisplayPort {
    pub payload: Vec<u8>,
}

impl MspDisplayPort {
    pub fn new(payload: Vec<u8>) -> Self {
        Self { payload }
    }

    pub fn heartbeat() -> Self {
        Self {
            payload: Vec::from([MSP_DP_HEARTBEAT]),
        }
    }

    pub fn release() -> Self {
        Self {
            payload: Vec::from([MSP_DP_RELEASE]),
        }
    }

    pub fn clear_screen() -> Self {
        Self {
            payload: Vec::from([MSP_DP_CLEAR_SCREEN]),
        }
    }

    pub fn draw_screen() -> Self {
        Self {
            payload: Vec::from([MSP_DP_DRAW_SCREEN]),
        }
    }

    pub fn write_string(row: u8, col: u8, attr: u8, text: &str) -> Self {
        let mut payload = Vec::with_capacity(4 + text.len());
        payload.push(MSP_DP_WRITE_STRING);
        payload.push(row);
        payload.push(col);
        payload.push(attr);
        payload.extend_from_slice(text.as_bytes());
        Self { payload }
    }

    pub fn sys(row: u8, col: u8, element: u8) -> Self {
        Self {
            payload: Vec::from([MSP_DP_SYS, row, col, element]),
        }
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.payload
    }
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "13", endian = "lsb", bit_numbering = "msb0")]
pub struct MspOsdConfig {
    pub video_system: u8,
    pub units: u8,
    pub rssi_alarm: u8,
    pub capacity_warning: u16,
    pub time_alarm: u16,
    pub alt_alarm: u16,
    pub dist_alarm: u16,
    pub neg_alt_alarm: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetGetOsdConfig {
    pub item_index: u8,
    #[packed_field(size_bytes = "13")]
    pub config: MspOsdConfig,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "2", endian = "lsb", bit_numbering = "msb0")]
pub struct MspOsdItemPosition {
    pub col: u8,
    pub row: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetOsdLayout {
    pub item_index: u8,
    #[packed_field(size_bytes = "2")]
    pub item: MspOsdItemPosition,
}

// inav msp layout item
#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetOsdLayoutItem {
    pub layout_index: u8,
    #[packed_field(size_bytes = "3")]
    pub item: MspSetOsdLayout,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(Debug, Serialize, Deserialize, Clone, Default)]
pub struct MspOsdSettings {
    pub osd_support: u8,
    pub config: MspOsdConfig,
    pub item_positions: Vec<MspOsdItemPosition>,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "2", endian = "lsb", bit_numbering = "msb0")]
pub struct MspOsdLayouts {
    pub layout_count: u8,
    pub item_count: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq, Default)]
pub enum SerialIdentifier {
    #[default]
    None = 255,
    USART1 = 0,
    USART2 = 1,
    USART3 = 2,
    USART4 = 3,
    USART5 = 4,
    USART6 = 5,
    USART7 = 6,
    USART8 = 7,
    UsbVcp = 20,
    SoftSerial1 = 30,
    SoftSerial2 = 31,
}

impl TryFrom<u8> for SerialIdentifier {
    type Error = &'static str;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        let serial = match value {
            255 => SerialIdentifier::None,
            0 => SerialIdentifier::USART1,
            1 => SerialIdentifier::USART2,
            2 => SerialIdentifier::USART3,
            3 => SerialIdentifier::USART4,
            4 => SerialIdentifier::USART5,
            5 => SerialIdentifier::USART6,
            6 => SerialIdentifier::USART7,
            7 => SerialIdentifier::USART8,
            20 => SerialIdentifier::UsbVcp,
            30 => SerialIdentifier::SoftSerial1,
            31 => SerialIdentifier::SoftSerial2,
            _ => return Err("Serial identifier not found"),
        };

        Ok(serial)
    }
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq, Default)]
pub enum Baudrate {
    #[default]
    BaudAuto = 0,
    Baud1200 = 1,
    Baud2400 = 2,
    Baud4800 = 3,
    Baud9600 = 4,
    Baud19200 = 5,
    Baud38400 = 6,
    Baud57600 = 7,
    Baud115200 = 8,
    Baud230400 = 9,
    Baud250000 = 10,
    Baud460800 = 11,
    Baud921600 = 12,
    Baud1000000 = 13,
    Baud1500000 = 14,
    Baud2000000 = 15,
    Baud2470000 = 16,
}

impl TryFrom<&str> for Baudrate {
    type Error = &'static str;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        let baudrate = match value {
            "0" => Baudrate::BaudAuto,
            "1200" => Baudrate::Baud1200,
            "2400" => Baudrate::Baud2400,
            "4800" => Baudrate::Baud4800,
            "9600" => Baudrate::Baud9600,
            "19200" => Baudrate::Baud19200,
            "38400" => Baudrate::Baud38400,
            "57600" => Baudrate::Baud57600,
            "115200" => Baudrate::Baud115200,
            "230400" => Baudrate::Baud230400,
            "250000" => Baudrate::Baud250000,
            "460800" => Baudrate::Baud460800,
            "921600" => Baudrate::Baud921600,
            "1000000" => Baudrate::Baud1000000,
            "1500000" => Baudrate::Baud1500000,
            "2000000" => Baudrate::Baud2000000,
            "2470000" => Baudrate::Baud2470000,
            _ => return Err("Baudrate identifier not found"),
        };

        Ok(baudrate)
    }
}

impl From<Baudrate> for String {
    fn from(value: Baudrate) -> Self {
        match value {
            Baudrate::BaudAuto => "0",
            Baudrate::Baud1200 => "1200",
            Baudrate::Baud2400 => "2400",
            Baudrate::Baud4800 => "4800",
            Baudrate::Baud9600 => "9600",
            Baudrate::Baud19200 => "19200",
            Baudrate::Baud38400 => "38400",
            Baudrate::Baud57600 => "57600",
            Baudrate::Baud115200 => "115200",
            Baudrate::Baud230400 => "230400",
            Baudrate::Baud250000 => "250000",
            Baudrate::Baud460800 => "460800",
            Baudrate::Baud921600 => "921600",
            Baudrate::Baud1000000 => "1000000",
            Baudrate::Baud1500000 => "1500000",
            Baudrate::Baud2000000 => "2000000",
            Baudrate::Baud2470000 => "2470000",
        }
        .to_owned()
    }
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb", bit_numbering = "msb0")]
pub struct MspSerialSetting {
    #[packed_field(size_bits = "8", ty = "enum")]
    pub identifier: SerialIdentifier,
    pub function_mask: u32,
    #[packed_field(size_bits = "8", ty = "enum")]
    pub msp_baudrate_index: Baudrate,
    #[packed_field(size_bits = "8", ty = "enum")]
    pub gps_baudrate_index: Baudrate,
    #[packed_field(size_bits = "8", ty = "enum")]
    pub telemetry_baudrate_index: Baudrate,
    #[packed_field(size_bits = "8", ty = "enum")]
    pub peripheral_baudrate_index: Baudrate,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetServoMixRule {
    pub index: u8,
    #[packed_field(size_bytes = "8")]
    pub servo_rule: MspServoMixRule,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "8", endian = "lsb", bit_numbering = "msb0")]
pub struct MspServoMixRule {
    pub target_channel: u8,
    pub input_source: u8,
    pub rate: u16,
    pub speed: u8,
    pub min: u8,
    pub max: u8,
    pub box_id: u8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetServoMixer {
    pub index: u8,
    #[packed_field(size_bytes = "6")]
    pub servo_rule: MspServoMixer,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "6", endian = "lsb", bit_numbering = "msb0")]
pub struct MspServoMixer {
    pub target_channel: u8,
    pub input_source: u8,
    pub rate: i16,
    pub speed: u8,
    pub condition_id: i8,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb", bit_numbering = "msb0")]
pub struct MspRxMap {
    pub map: [u8; 4], // MAX_MAPPABLE_RX_INPUTS
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb", bit_numbering = "msb0")]
pub struct MspSettingGroup {
    pub group_id: u16,
    pub start_id: u16,
    pub end_id: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb", bit_numbering = "msb0")]
pub struct MspSettingInfoRequest {
    pub null: u8,
    pub id: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq, Default)]
pub enum SettingMode {
    #[default]
    ModeDirect = 0,
    ModeLookup = 0x40,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq, Default)]
pub enum SettingType {
    #[default]
    VarUint8 = 0,
    VarInt8,
    VarUint16,
    VarInt16,
    VarUint32,
    VarInt32,
    VarFloat,
    VarString,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(bytes = "15", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSettingInfo {
    // pub name: [u8; ?], null terminated strings

    // Parameter Group ID
    pub group_id: u16,

    // Type, section and mode
    #[packed_field(size_bits = "8", ty = "enum")]
    pub setting_type: SettingType,
    pub setting_section: u8,
    #[packed_field(size_bits = "8", ty = "enum")]
    pub setting_mode: SettingMode,

    pub min: u32,
    pub max: u32,

    // Absolute setting index
    pub absolute_index: u16,

    // If the setting is profile based, send the current one
    // and the count, both as uint8_t. For MASTER_VALUE, we
    // send two zeroes, so the MSP client can assume there
    pub profile_id: u8,
    pub profile_count: u8,
    // if setting uses enum values, it will be written here
    // pub enum_names: [String; ?] // TODO: packed_struct should support null terminated string parsing
    // pub value: [u8; ?]
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspRc {
    pub channels: [u16; 16], // 16 RC channels
}

impl Default for MspRc {
    fn default() -> Self {
        Self::new()
    }
}

impl MspRc {
    pub fn new() -> Self {
        MspRc { channels: [0; 16] }
    }

    pub fn set_roll(&mut self, value: u16) {
        self.channels[0] = value;
    }

    pub fn set_pitch(&mut self, value: u16) {
        self.channels[1] = value;
    }

    pub fn set_throttle(&mut self, value: u16) {
        self.channels[2] = value;
    }

    pub fn set_yaw(&mut self, value: u16) {
        self.channels[3] = value;
    }

    pub fn set_aux1(&mut self, value: u16) {
        self.channels[4] = value;
    }
    pub fn set_aux2(&mut self, value: u16) {
        self.channels[5] = value;
    }
    pub fn set_aux3(&mut self, value: u16) {
        self.channels[6] = value;
    }
    pub fn set_aux4(&mut self, value: u16) {
        self.channels[7] = value;
    }
}

// Gather all the commands in a common enum we can use as a higher level protocol
#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub enum MspRequest {
    #[default]
    Unknown,
    MspBatteryConfigRequest,
    MspBatteryConfig(MspBatteryConfig),
    MspBatteryStateRequest,
    MspBatteryState(MspBatteryState),
    MspAnalogRequest,
    MspAnalog(MspAnalog),
    MspVoltageMeterConfigRequest,
    MspVoltageMeterConfig(MspVoltageMeterConfig),
    MspVoltageMetersRequest,
    MspVoltageMeter(MspVoltageMeter),
    MspRc,
    MspSetRawRc(MspRc),
    MspRawImu,
    MspStatus(MspStatus),
    MspStatusEx(MspStatusEx),
    MspDisplayPort(MspDisplayPort),
}

impl MspRequest {
    pub fn command_code(&self) -> MspCommandCode {
        match self {
            MspRequest::MspBatteryConfigRequest => MspCommandCode::MSP_BATTERY_CONFIG,
            MspRequest::MspBatteryConfig(_) => MspCommandCode::MSP_BATTERY_CONFIG,
            MspRequest::MspBatteryStateRequest => MspCommandCode::MSP_BATTERY_STATE,
            MspRequest::MspBatteryState(_) => MspCommandCode::MSP_BATTERY_STATE,
            MspRequest::MspAnalogRequest => MspCommandCode::MSP_ANALOG,
            MspRequest::MspAnalog(_) => MspCommandCode::MSP_ANALOG,
            MspRequest::MspVoltageMeterConfigRequest => MspCommandCode::MSP_VOLTAGE_METER_CONFIG,
            MspRequest::MspVoltageMeterConfig(_) => MspCommandCode::MSP_VOLTAGE_METER_CONFIG,
            MspRequest::MspVoltageMetersRequest => MspCommandCode::MSP_VOLTAGE_METERS,
            MspRequest::MspVoltageMeter(_) => MspCommandCode::MSP_VOLTAGE_METERS,
            MspRequest::MspRc => MspCommandCode::MSP_RC,
            MspRequest::MspSetRawRc(_) => MspCommandCode::MSP_SET_RAW_RC,
            MspRequest::MspRawImu => MspCommandCode::MSP_RAW_IMU,
            MspRequest::MspStatus(_) => MspCommandCode::MSP_STATUS,
            MspRequest::MspStatusEx(_) => MspCommandCode::MSP_STATUS_EX,
            MspRequest::MspDisplayPort(_) => MspCommandCode::MSP_DISPLAYPORT,
            _ => MspCommandCode::MSP_API_VERSION,
        }
    }

    pub fn from_command_code(cmd: MspCommandCode) -> Option<Self> {
        match cmd {
            MspCommandCode::MSP_BATTERY_CONFIG => Some(MspRequest::MspBatteryConfigRequest),
            MspCommandCode::MSP_BATTERY_STATE => Some(MspRequest::MspBatteryStateRequest),
            MspCommandCode::MSP_ANALOG => Some(MspRequest::MspAnalogRequest),
            MspCommandCode::MSP_VOLTAGE_METER_CONFIG => {
                Some(MspRequest::MspVoltageMeterConfigRequest)
            }
            MspCommandCode::MSP_VOLTAGE_METERS => Some(MspRequest::MspVoltageMetersRequest),
            MspCommandCode::MSP_RC => Some(MspRequest::MspRc),
            MspCommandCode::MSP_RAW_IMU => Some(MspRequest::MspRawImu),
            _ => None,
        }
    }

    pub fn from_command_id(cmd: u16) -> Option<Self> {
        let cmd = MspCommandCode::from_primitive(cmd)?;
        Self::from_command_code(cmd)
    }
}

impl From<MspRequest> for MspPacket {
    fn from(request: MspRequest) -> Self {
        match request {
            MspRequest::MspBatteryConfigRequest => MspPacket {
                cmd: MspCommandCode::MSP_BATTERY_CONFIG.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(),
            },
            MspRequest::MspBatteryConfig(config) => MspPacket {
                cmd: MspCommandCode::MSP_BATTERY_CONFIG.to_primitive(),
                direction: FromFlightController,
                data: config.pack().map_or_else(|_| MspPacketData::new(), |data| {
                    MspPacketData(SmallVec::from_slice(data.as_slice()))
                }),
            },
            MspRequest::MspBatteryStateRequest => MspPacket {
                cmd: MspCommandCode::MSP_BATTERY_STATE.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
            MspRequest::MspBatteryState(state) => MspPacket {
                cmd: MspCommandCode::MSP_BATTERY_STATE.to_primitive(),
                direction: FromFlightController,
                data: state.pack().map_or_else(|_| MspPacketData::new(), |data| {
                    MspPacketData(SmallVec::from_slice(data.as_slice()))
                }),
            },
            MspRequest::MspAnalogRequest => MspPacket {
                cmd: MspCommandCode::MSP_ANALOG.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
            MspRequest::MspAnalog(analog) => MspPacket {
                cmd: MspCommandCode::MSP_ANALOG.to_primitive(),
                direction: FromFlightController,
                data: analog.pack().map_or_else(|_| MspPacketData::new(), |data| {
                    MspPacketData(SmallVec::from_slice(data.as_slice()))
                }),
            },
            MspRequest::MspVoltageMeterConfigRequest => MspPacket {
                cmd: MspCommandCode::MSP_VOLTAGE_METER_CONFIG.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(),
            },
            MspRequest::MspVoltageMeterConfig(config) => MspPacket {
                cmd: MspCommandCode::MSP_VOLTAGE_METER_CONFIG.to_primitive(),
                direction: FromFlightController,
                data: config.pack().map_or_else(|_| MspPacketData::new(), |data| {
                    MspPacketData(SmallVec::from_slice(data.as_slice()))
                }),
            },
            MspRequest::MspVoltageMetersRequest => MspPacket {
                cmd: MspCommandCode::MSP_VOLTAGE_METERS.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
            MspRequest::MspVoltageMeter(meter) => MspPacket {
                cmd: MspCommandCode::MSP_VOLTAGE_METERS.to_primitive(),
                direction: FromFlightController,
                data: meter.pack().map_or_else(|_| MspPacketData::new(), |data| {
                    MspPacketData(SmallVec::from_slice(data.as_slice()))
                }),
            },
            MspRequest::MspRc => MspPacket {
                cmd: MspCommandCode::MSP_RC.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
            MspRequest::MspSetRawRc(rc) => {
                let data = rc.pack().unwrap();
                MspPacket {
                    cmd: MspCommandCode::MSP_SET_RAW_RC.to_primitive(),
                    direction: ToFlightController,
                    data: MspPacketData(SmallVec::from_slice(data.as_slice())),
                }
            }
            MspRequest::MspRawImu => MspPacket {
                cmd: MspCommandCode::MSP_RAW_IMU.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
            MspRequest::MspStatus(status) => MspPacket {
                cmd: MspCommandCode::MSP_STATUS.to_primitive(),
                direction: FromFlightController,
                data: status.to_packet_data().unwrap(),
            },
            MspRequest::MspStatusEx(status) => MspPacket {
                cmd: MspCommandCode::MSP_STATUS_EX.to_primitive(),
                direction: FromFlightController,
                data: status.to_packet_data().unwrap(),
            },
            MspRequest::MspDisplayPort(displayport) => MspPacket {
                cmd: MspCommandCode::MSP_DISPLAYPORT.to_primitive(),
                direction: FromFlightController,
                data: MspPacketData::from(displayport.as_bytes()),
            },
            _ => MspPacket {
                cmd: MspCommandCode::MSP_API_VERSION.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
        }
    }
}

impl From<&MspRequest> for MspPacket {
    fn from(request: &MspRequest) -> Self {
        match request {
            MspRequest::MspBatteryConfigRequest => MspPacket {
                cmd: MspCommandCode::MSP_BATTERY_CONFIG.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(),
            },
            MspRequest::MspBatteryConfig(config) => MspPacket {
                cmd: MspCommandCode::MSP_BATTERY_CONFIG.to_primitive(),
                direction: FromFlightController,
                data: config.pack().map_or_else(|_| MspPacketData::new(), |data| {
                    MspPacketData::from(data.as_slice())
                }),
            },
            MspRequest::MspBatteryStateRequest => MspPacket {
                cmd: MspCommandCode::MSP_BATTERY_STATE.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
            MspRequest::MspBatteryState(state) => MspPacket {
                cmd: MspCommandCode::MSP_BATTERY_STATE.to_primitive(),
                direction: FromFlightController,
                data: state.pack().map_or_else(|_| MspPacketData::new(), |data| {
                    MspPacketData::from(data.as_slice())
                }),
            },
            MspRequest::MspAnalogRequest => MspPacket {
                cmd: MspCommandCode::MSP_ANALOG.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
            MspRequest::MspAnalog(analog) => MspPacket {
                cmd: MspCommandCode::MSP_ANALOG.to_primitive(),
                direction: FromFlightController,
                data: analog.pack().map_or_else(|_| MspPacketData::new(), |data| {
                    MspPacketData::from(data.as_slice())
                }),
            },
            MspRequest::MspVoltageMeterConfigRequest => MspPacket {
                cmd: MspCommandCode::MSP_VOLTAGE_METER_CONFIG.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(),
            },
            MspRequest::MspVoltageMeterConfig(config) => MspPacket {
                cmd: MspCommandCode::MSP_VOLTAGE_METER_CONFIG.to_primitive(),
                direction: FromFlightController,
                data: config.pack().map_or_else(|_| MspPacketData::new(), |data| {
                    MspPacketData::from(data.as_slice())
                }),
            },
            MspRequest::MspVoltageMetersRequest => MspPacket {
                cmd: MspCommandCode::MSP_VOLTAGE_METERS.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
            MspRequest::MspVoltageMeter(meter) => MspPacket {
                cmd: MspCommandCode::MSP_VOLTAGE_METERS.to_primitive(),
                direction: FromFlightController,
                data: meter.pack().map_or_else(|_| MspPacketData::new(), |data| {
                    MspPacketData::from(data.as_slice())
                }),
            },
            MspRequest::MspRc => MspPacket {
                cmd: MspCommandCode::MSP_RC.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
            MspRequest::MspSetRawRc(rc) => MspPacket {
                cmd: MspCommandCode::MSP_SET_RAW_RC.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::from(rc.pack().unwrap().as_slice()),
            },
            MspRequest::MspRawImu => MspPacket {
                cmd: MspCommandCode::MSP_RAW_IMU.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
            MspRequest::MspStatus(status) => MspPacket {
                cmd: MspCommandCode::MSP_STATUS.to_primitive(),
                direction: FromFlightController,
                data: status.to_packet_data().unwrap(),
            },
            MspRequest::MspStatusEx(status) => MspPacket {
                cmd: MspCommandCode::MSP_STATUS_EX.to_primitive(),
                direction: FromFlightController,
                data: status.to_packet_data().unwrap(),
            },
            MspRequest::MspDisplayPort(displayport) => MspPacket {
                cmd: MspCommandCode::MSP_DISPLAYPORT.to_primitive(),
                direction: FromFlightController,
                data: MspPacketData::from(displayport.as_bytes()),
            },
            _ => MspPacket {
                cmd: MspCommandCode::MSP_API_VERSION.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
            },
        }
    }
}

// Gather all the commands in a common enum we can use as a higher level protocol
#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub enum MspResponse {
    #[default]
    Unknown,
    MspApiVersion(MspApiVersion),
    MspFlightControllerVariant(MspFlightControllerVariant),
    MspStatus(MspStatus),
    MspStatusEx(MspStatusEx),
    MspBfConfig(MspBfConfig),
    MspRawImu(MspRawImu),
    MspDataFlashSummaryReply(MspDataFlashSummaryReply),
    MspDataFlashReply(MspDataFlashReply),
    MspDataFlashRead(MspDataFlashRead),
    MspAccTrim(MspAccTrim),
    MspIdent(MspIdent),
    MspMisc(MspMisc),
    MspAttitude(MspAttitude),
    MspAltitude(MspAltitude),
    MspBatteryConfig(MspBatteryConfig),
    MspVoltageMeterConfig(MspVoltageMeterConfig),
    MspAnalog(MspAnalog),
    MspRssiConfig(MspRssiConfig),
    MspVoltageMeter(MspVoltageMeter),
    MspCurrentMeter(MspCurrentMeter),
    MspBatteryState(MspBatteryState),
    MspRcTuning(MspRcTuning),
    MspRxConfig(MspRxConfig),
    MspRcChannelValue(MspRcChannelValue),
    MspRcMappedChannel(MspRcMappedChannel),
    MspFeatures(MspFeatures),
    MspMotor(MspMotor),
    MspMotor3DConfig(MspMotor3DConfig),
    MspMotorConfig(MspMotorConfig),
    MspRcDeadband(MspRcDeadband),
    MspSensorAlignment(MspSensorAlignment),
    MspAdvancedConfig(MspAdvancedConfig),
    MspFilterConfig(MspFilterConfig),
    MspPidAdvanced(MspPidAdvanced),
    MspSensorConfig(MspSensorConfig),
    MspServos(MspServos),
    MspMixerConfig(MspMixerConfig),
    MspModeRange(MspModeRange),
    MspSetModeRange(MspSetModeRange),
    MspOsdConfig(MspOsdConfig),
    MspSetGetOsdConfig(MspSetGetOsdConfig),
    MspSetOsdLayout(MspSetOsdLayout),
    MspSetOsdLayoutItem(MspSetOsdLayoutItem),
    MspOsdLayouts(MspOsdLayouts),
    MspSerialSetting(MspSerialSetting),
    MspSettingInfoRequest(MspSettingInfoRequest),
    MspSettingInfo(MspSettingInfo),
    MspRc(MspRc),
}

impl MspResponse {
    pub fn command_code(&self) -> MspCommandCode {
        // TODO: Not sure about all those mapping recheck them.
        match self {
            MspResponse::MspApiVersion(_) => MspCommandCode::MSP_API_VERSION,
            MspResponse::MspFlightControllerVariant(_) => MspCommandCode::MSP_FC_VARIANT,
            MspResponse::MspStatus(_) => MspCommandCode::MSP_STATUS,
            MspResponse::MspStatusEx(_) => MspCommandCode::MSP_STATUS_EX,
            MspResponse::MspBfConfig(_) => MspCommandCode::MSP_BF_CONFIG,
            MspResponse::MspRawImu(_) => MspCommandCode::MSP_RAW_IMU,
            MspResponse::MspDataFlashSummaryReply(_) => MspCommandCode::MSP_DATAFLASH_SUMMARY,
            MspResponse::MspDataFlashReply(_) => MspCommandCode::MSP_DATAFLASH_READ,
            MspResponse::MspDataFlashRead(_) => MspCommandCode::MSP_DATAFLASH_READ,
            MspResponse::MspAccTrim(_) => MspCommandCode::MSP_ACC_TRIM,
            MspResponse::MspIdent(_) => MspCommandCode::MSP_IDENT,
            MspResponse::MspMisc(_) => MspCommandCode::MSP_MISC,
            MspResponse::MspAttitude(_) => MspCommandCode::MSP_ATTITUDE,
            MspResponse::MspAltitude(_) => MspCommandCode::MSP_ALTITUDE,
            MspResponse::MspBatteryConfig(_) => MspCommandCode::MSP_BATTERY_CONFIG,
            MspResponse::MspVoltageMeterConfig(_) => MspCommandCode::MSP_VOLTAGE_METER_CONFIG,
            MspResponse::MspAnalog(_) => MspCommandCode::MSP_ANALOG,
            MspResponse::MspRssiConfig(_) => MspCommandCode::MSP_RSSI_CONFIG,
            MspResponse::MspVoltageMeter(_) => MspCommandCode::MSP_VOLTAGE_METERS,
            MspResponse::MspCurrentMeter(_) => MspCommandCode::MSP_AMPERAGE_METER_CONFIG,
            MspResponse::MspBatteryState(_) => MspCommandCode::MSP_BATTERY_STATE,
            MspResponse::MspRcTuning(_) => MspCommandCode::MSP_RC_TUNING,
            MspResponse::MspRxConfig(_) => MspCommandCode::MSP_RX_CONFIG,
            MspResponse::MspRcChannelValue(_) => MspCommandCode::MSP_RX_MAP,
            MspResponse::MspRcMappedChannel(_) => MspCommandCode::MSP_SET_RX_MAP,
            MspResponse::MspFeatures(_) => MspCommandCode::MSP_FEATURE,
            MspResponse::MspMotor(_) => MspCommandCode::MSP_MOTOR,
            MspResponse::MspMotor3DConfig(_) => MspCommandCode::MSP_MOTOR_3D_CONFIG,
            MspResponse::MspMotorConfig(_) => MspCommandCode::MSP_MOTOR_CONFIG,
            MspResponse::MspRcDeadband(_) => MspCommandCode::MSP_RC_DEADBAND,
            MspResponse::MspSensorAlignment(_) => MspCommandCode::MSP_BOARD_ALIGNMENT,
            MspResponse::MspAdvancedConfig(_) => MspCommandCode::MSP_ADVANCED_CONFIG,
            MspResponse::MspFilterConfig(_) => MspCommandCode::MSP_FILTER_CONFIG,
            MspResponse::MspPidAdvanced(_) => MspCommandCode::MSP_PID_ADVANCED,
            MspResponse::MspSensorConfig(_) => MspCommandCode::MSP_SENSOR_CONFIG,
            MspResponse::MspServos(_) => MspCommandCode::MSP_SERVO,
            MspResponse::MspMixerConfig(_) => MspCommandCode::MSP_MIXER,
            MspResponse::MspModeRange(_) => MspCommandCode::MSP_MODE_RANGES,
            MspResponse::MspSetModeRange(_) => MspCommandCode::MSP_SET_MODE_RANGE,
            MspResponse::MspOsdConfig(_) => MspCommandCode::MSP_OSD_CONFIG,
            MspResponse::MspSetGetOsdConfig(_) => MspCommandCode::MSP_OSD_CONFIG,
            MspResponse::MspSetOsdLayout(_) => MspCommandCode::MSP_OSD_LAYOUT_CONFIG,
            MspResponse::MspSetOsdLayoutItem(_) => MspCommandCode::MSP2_INAV_OSD_SET_LAYOUT_ITEM,
            MspResponse::MspOsdLayouts(_) => MspCommandCode::MSP2_INAV_OSD_LAYOUTS,
            MspResponse::MspSerialSetting(_) => MspCommandCode::MSP2_SET_SERIAL_CONFIG,
            MspResponse::MspSettingInfoRequest(_) => MspCommandCode::MSP2_COMMON_SETTING,
            MspResponse::MspSettingInfo(_) => MspCommandCode::MSP2_COMMON_SETTING_INFO,
            MspResponse::MspRc(_) => MspCommandCode::MSP_RC,
            MspResponse::Unknown => MspCommandCode::MSP_API_VERSION,
        }
    }

    pub fn to_bytes(&self) -> Result<MspPacketData, PackingError> {
        fn pack_into_packet_data<T: PackedStruct>(
            value: &T,
        ) -> Result<MspPacketData, PackingError> {
            let packed = value.pack()?;
            Ok(MspPacketData(SmallVec::from_slice(packed.as_bytes_slice())))
        }

        match self {
            MspResponse::MspApiVersion(data) => pack_into_packet_data(data),
            MspResponse::MspFlightControllerVariant(data) => pack_into_packet_data(data),
            MspResponse::MspStatus(data) => data.to_packet_data(),
            MspResponse::MspStatusEx(data) => data.to_packet_data(),
            MspResponse::MspBfConfig(data) => pack_into_packet_data(data),
            MspResponse::MspRawImu(data) => pack_into_packet_data(data),
            MspResponse::MspDataFlashSummaryReply(data) => pack_into_packet_data(data),
            MspResponse::MspDataFlashReply(data) => pack_into_packet_data(data),
            MspResponse::MspDataFlashRead(data) => pack_into_packet_data(data),
            MspResponse::MspAccTrim(data) => pack_into_packet_data(data),
            MspResponse::MspIdent(data) => pack_into_packet_data(data),
            MspResponse::MspMisc(data) => pack_into_packet_data(data),
            MspResponse::MspAttitude(data) => pack_into_packet_data(data),
            MspResponse::MspAltitude(data) => pack_into_packet_data(data),
            MspResponse::MspBatteryConfig(data) => pack_into_packet_data(data),
            MspResponse::MspVoltageMeterConfig(data) => pack_into_packet_data(data),
            MspResponse::MspAnalog(data) => pack_into_packet_data(data),
            MspResponse::MspRssiConfig(data) => pack_into_packet_data(data),
            MspResponse::MspVoltageMeter(data) => pack_into_packet_data(data),
            MspResponse::MspCurrentMeter(data) => pack_into_packet_data(data),
            MspResponse::MspBatteryState(data) => pack_into_packet_data(data),
            MspResponse::MspRcTuning(data) => pack_into_packet_data(data),
            MspResponse::MspRxConfig(data) => pack_into_packet_data(data),
            MspResponse::MspRcChannelValue(data) => pack_into_packet_data(data),
            MspResponse::MspRcMappedChannel(data) => pack_into_packet_data(data),
            MspResponse::MspFeatures(data) => pack_into_packet_data(data),
            MspResponse::MspMotor(data) => pack_into_packet_data(data),
            MspResponse::MspMotor3DConfig(data) => pack_into_packet_data(data),
            MspResponse::MspMotorConfig(data) => pack_into_packet_data(data),
            MspResponse::MspRcDeadband(data) => pack_into_packet_data(data),
            MspResponse::MspSensorAlignment(data) => pack_into_packet_data(data),
            MspResponse::MspAdvancedConfig(data) => pack_into_packet_data(data),
            MspResponse::MspFilterConfig(data) => pack_into_packet_data(data),
            MspResponse::MspPidAdvanced(data) => pack_into_packet_data(data),
            MspResponse::MspSensorConfig(data) => pack_into_packet_data(data),
            MspResponse::MspServos(data) => pack_into_packet_data(data),
            MspResponse::MspMixerConfig(data) => pack_into_packet_data(data),
            MspResponse::MspModeRange(data) => pack_into_packet_data(data),
            MspResponse::MspSetModeRange(data) => pack_into_packet_data(data),
            MspResponse::MspOsdConfig(data) => pack_into_packet_data(data),
            MspResponse::MspSetGetOsdConfig(data) => pack_into_packet_data(data),
            MspResponse::MspSetOsdLayout(data) => pack_into_packet_data(data),
            MspResponse::MspSetOsdLayoutItem(data) => pack_into_packet_data(data),
            MspResponse::MspOsdLayouts(data) => pack_into_packet_data(data),
            MspResponse::MspSerialSetting(data) => pack_into_packet_data(data),
            MspResponse::MspSettingInfoRequest(data) => pack_into_packet_data(data),
            MspResponse::MspSettingInfo(data) => pack_into_packet_data(data),
            MspResponse::MspRc(data) => pack_into_packet_data(data),
            MspResponse::Unknown => Err(PackingError::InvalidValue),
        }
    }
}

impl From<MspResponse> for MspPacket {
    fn from(command: MspResponse) -> Self {
        MspPacket {
            cmd: command.command_code().to_primitive(),
            direction: FromFlightController,
            data: command.to_bytes().unwrap(), // should always be able to serialize a constructed MSPCommand
        }
    }
}
impl From<MspPacket> for MspResponse {
    fn from(packet: MspPacket) -> Self {
        let cmd = match MspCommandCode::from_primitive(packet.cmd) {
            Some(cmd) => cmd,
            None => return MspResponse::Unknown,
        };
        match cmd {
            MspCommandCode::MSP_API_VERSION => packet
                .decode_as::<MspApiVersion>()
                .map(MspResponse::MspApiVersion)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_FC_VARIANT => packet
                .decode_as::<MspFlightControllerVariant>()
                .map(MspResponse::MspFlightControllerVariant)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_STATUS => MspStatus::from_bytes(packet.data.as_slice())
                .map(MspResponse::MspStatus)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_STATUS_EX => MspStatusEx::from_bytes(packet.data.as_slice())
                .map(MspResponse::MspStatusEx)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_BF_CONFIG => packet
                .decode_as::<MspBfConfig>()
                .map(MspResponse::MspBfConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_RAW_IMU => packet
                .decode_as::<MspRawImu>()
                .map(MspResponse::MspRawImu)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_DATAFLASH_SUMMARY => packet
                .decode_as::<MspDataFlashSummaryReply>()
                .map(MspResponse::MspDataFlashSummaryReply)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_DATAFLASH_READ => packet
                .decode_as::<MspDataFlashReply>()
                .map(MspResponse::MspDataFlashReply)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_ACC_TRIM => packet
                .decode_as::<MspAccTrim>()
                .map(MspResponse::MspAccTrim)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_IDENT => packet
                .decode_as::<MspIdent>()
                .map(MspResponse::MspIdent)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_MISC => packet
                .decode_as::<MspMisc>()
                .map(MspResponse::MspMisc)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_ATTITUDE => packet
                .decode_as::<MspAttitude>()
                .map(MspResponse::MspAttitude)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_ALTITUDE => packet
                .decode_as::<MspAltitude>()
                .map(MspResponse::MspAltitude)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_BATTERY_CONFIG => packet
                .decode_as::<MspBatteryConfig>()
                .map(MspResponse::MspBatteryConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_VOLTAGE_METER_CONFIG => packet
                .decode_as::<MspVoltageMeterConfig>()
                .map(MspResponse::MspVoltageMeterConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_ANALOG => packet
                .decode_as::<MspAnalog>()
                .map(MspResponse::MspAnalog)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_RSSI_CONFIG => packet
                .decode_as::<MspRssiConfig>()
                .map(MspResponse::MspRssiConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_VOLTAGE_METERS => packet
                .decode_as::<MspVoltageMeter>()
                .map(MspResponse::MspVoltageMeter)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_AMPERAGE_METER_CONFIG => packet
                .decode_as::<MspCurrentMeter>()
                .map(MspResponse::MspCurrentMeter)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_BATTERY_STATE => packet
                .decode_as::<MspBatteryState>()
                .map(MspResponse::MspBatteryState)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_RC_TUNING => packet
                .decode_as::<MspRcTuning>()
                .map(MspResponse::MspRcTuning)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_RX_CONFIG => packet
                .decode_as::<MspRxConfig>()
                .map(MspResponse::MspRxConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_RX_MAP => packet
                .decode_as::<MspRcChannelValue>()
                .map(MspResponse::MspRcChannelValue)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_SET_RX_MAP => packet
                .decode_as::<MspRcMappedChannel>()
                .map(MspResponse::MspRcMappedChannel)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_FEATURE => packet
                .decode_as::<MspFeatures>()
                .map(MspResponse::MspFeatures)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_MOTOR => packet
                .decode_as::<MspMotor>()
                .map(MspResponse::MspMotor)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_MOTOR_3D_CONFIG => packet
                .decode_as::<MspMotor3DConfig>()
                .map(MspResponse::MspMotor3DConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_MOTOR_CONFIG => packet
                .decode_as::<MspMotorConfig>()
                .map(MspResponse::MspMotorConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_RC_DEADBAND => packet
                .decode_as::<MspRcDeadband>()
                .map(MspResponse::MspRcDeadband)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_BOARD_ALIGNMENT => packet
                .decode_as::<MspSensorAlignment>()
                .map(MspResponse::MspSensorAlignment)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_ADVANCED_CONFIG => packet
                .decode_as::<MspAdvancedConfig>()
                .map(MspResponse::MspAdvancedConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_FILTER_CONFIG => packet
                .decode_as::<MspFilterConfig>()
                .map(MspResponse::MspFilterConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_PID_ADVANCED => packet
                .decode_as::<MspPidAdvanced>()
                .map(MspResponse::MspPidAdvanced)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_SENSOR_CONFIG => packet
                .decode_as::<MspSensorConfig>()
                .map(MspResponse::MspSensorConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_SERVO => packet
                .decode_as::<MspServos>()
                .map(MspResponse::MspServos)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_MIXER => packet
                .decode_as::<MspMixerConfig>()
                .map(MspResponse::MspMixerConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_MODE_RANGES => packet
                .decode_as::<MspModeRange>()
                .map(MspResponse::MspModeRange)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_SET_MODE_RANGE => packet
                .decode_as::<MspSetModeRange>()
                .map(MspResponse::MspSetModeRange)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_OSD_CONFIG => packet
                .decode_as::<MspOsdConfig>()
                .map(MspResponse::MspOsdConfig)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_OSD_LAYOUT_CONFIG => packet
                .decode_as::<MspSetOsdLayout>()
                .map(MspResponse::MspSetOsdLayout)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP2_INAV_OSD_SET_LAYOUT_ITEM => packet
                .decode_as::<MspSetOsdLayoutItem>()
                .map(MspResponse::MspSetOsdLayoutItem)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP2_INAV_OSD_LAYOUTS => packet
                .decode_as::<MspOsdLayouts>()
                .map(MspResponse::MspOsdLayouts)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP2_SET_SERIAL_CONFIG => packet
                .decode_as::<MspSerialSetting>()
                .map(MspResponse::MspSerialSetting)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP2_COMMON_SETTING => packet
                .decode_as::<MspSettingInfoRequest>()
                .map(MspResponse::MspSettingInfoRequest)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP2_COMMON_SETTING_INFO => packet
                .decode_as::<MspSettingInfo>()
                .map(MspResponse::MspSettingInfo)
                .unwrap_or(MspResponse::Unknown),
            MspCommandCode::MSP_RC => packet
                .decode_as::<MspRc>()
                .map(MspResponse::MspRc)
                .unwrap_or(MspResponse::Unknown),
            _ => MspResponse::Unknown,
        }
    }
}

#[cfg(test)]
mod msp_status_tests {
    use super::*;

    #[test]
    fn msp_status_serialization_roundtrip() {
        let status = MspStatus {
            cycle_time: 1234,
            i2c_errors: 5,
            sensors: MspStatusSensors {
                acc: true,
                baro: true,
                mag: false,
                gps: false,
                rangefinder: false,
                gyro: true,
                optical_flow: false,
            },
            flight_mode_flags: 0x11223344,
            current_pid_profile_index: 2,
            average_system_load_percent: 100,
            gyro_cycle_time: 250,
            extra_flight_mode_flags: vec![0xAA, 0xBB],
            arming_disable_flags_count: 29,
            arming_disable_flags: 0xDEADBEEF,
            config_state_flags: 1,
            core_temp_celsius: 42,
            control_rate_profile_count: 3,
        };

        let data = status.to_packet_data().expect("serialize MSP_STATUS");
        let expected = vec![
            0xD2, 0x04, // cycle_time
            0x05, 0x00, // i2c_errors
            0x23, 0x00, // sensors
            0x44, 0x33, 0x22, 0x11, // flight_mode_flags
            0x02, // current_pid_profile_index
            0x64, 0x00, // average_system_load_percent
            0xFA, 0x00, // gyro_cycle_time
            0x02, // extra_flight_mode_flags_len
            0xAA, 0xBB, // extra_flight_mode_flags
            0x1D, // arming_disable_flags_count
            0xEF, 0xBE, 0xAD, 0xDE, // arming_disable_flags
            0x01, // config_state_flags
            0x2A, 0x00, // core_temp_celsius
            0x03, // control_rate_profile_count
        ];

        assert_eq!(data.as_slice(), expected.as_slice());

        let decoded = MspStatus::from_bytes(data.as_slice()).expect("decode MSP_STATUS");
        assert_eq!(decoded, status);
    }

    #[test]
    fn msp_status_ex_serialization_roundtrip() {
        let status = MspStatusEx {
            cycle_time: 321,
            i2c_errors: 1,
            sensors: MspStatusSensors {
                acc: true,
                baro: false,
                mag: true,
                gps: false,
                rangefinder: false,
                gyro: true,
                optical_flow: true,
            },
            flight_mode_flags: 0xAABBCCDD,
            current_pid_profile_index: 1,
            average_system_load_percent: 250,
            max_profile_count: 3,
            current_control_rate_profile_index: 2,
            extra_flight_mode_flags: vec![],
            arming_disable_flags_count: 29,
            arming_disable_flags: 0,
            config_state_flags: 0,
            core_temp_celsius: 0,
            control_rate_profile_count: 3,
        };

        let data = status.to_packet_data().expect("serialize MSP_STATUS_EX");
        let decoded = MspStatusEx::from_bytes(data.as_slice()).expect("decode MSP_STATUS_EX");
        assert_eq!(decoded, status);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_mixer() {
        use packed_struct::prelude::*;

        let m = MspMixerConfig {
            mixer_mode: MixerMode::QuadX,
        };
        assert_eq!(3, m.mixer_mode.to_primitive());
        let r = m.pack().unwrap();
        assert_eq!(3, r.as_slice()[0]);
    }

    #[test]
    #[cfg(feature = "bincode")]
    fn test_command_enum_with_bincode() {
        let command = MspResponse::MspApiVersion(MspApiVersion {
            protocol_version: 1,
            api_version_major: 2,
            api_version_minor: 3,
        });

        let encoded = bincode::encode_to_vec(&command, bincode::config::standard()).unwrap();
        let decoded: MspResponse =
            bincode::decode_from_slice(&encoded, bincode::config::standard())
                .unwrap()
                .0;

        match (command, decoded) {
            (MspResponse::MspApiVersion(c1), MspResponse::MspApiVersion(c2)) => {
                assert_eq!(c1.protocol_version, c2.protocol_version);
                assert_eq!(c1.api_version_major, c2.api_version_major);
                assert_eq!(c1.api_version_minor, c2.api_version_minor);
            }
            _ => panic!("Decoded command does not match the original command"),
        }
    }
}
