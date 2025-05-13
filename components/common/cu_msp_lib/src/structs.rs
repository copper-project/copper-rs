//! MSP structures

use packed_struct::derive::{PackedStruct, PrimitiveEnum};
use serde::{Deserialize, Serialize};

use crate::commands::MspCommandCode;
use crate::MspPacketDirection::{FromFlightController, ToFlightController};
use crate::{MspPacket, MspPacketData};
#[cfg(feature = "bincode")]
use bincode::{Decode, Encode};
use packed_struct::{PackedStruct, PackedStructSlice, PackingError, PrimitiveEnum};
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
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspStatus {
    pub cycle_time: u16,
    pub i2c_errors: u16,
    #[packed_field(size_bits = "8")]
    pub sensors: MspAvailableSensors,
    pub null1: u8,
    pub flight_mode: u32,
    pub profile: u8,
    pub system_load: u16,
}

#[cfg_attr(feature = "bincode", derive(Decode, Encode))]
#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspStatusEx {
    pub cycle_time: u16,
    pub i2c_errors: u16,
    #[packed_field(size_bits = "8")]
    pub sensors: MspAvailableSensors,
    pub null1: u8,
    pub flight_mode: u32,
    pub current_pid_profile_index: u8,
    pub average_system_load_percent: u16,
    pub max_profile_count: u8,
    pub current_control_rate_profile_index: u8,
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
    MspBatteryState,
    MspRc,
    MspSetRawRc(MspRc),
    MspRawImu,
}

impl MspRequest {
    pub fn command_code(&self) -> MspCommandCode {
        match self {
            MspRequest::MspBatteryState => MspCommandCode::MSP_BATTERY_STATE,
            MspRequest::MspRc => MspCommandCode::MSP_RC,
            MspRequest::MspSetRawRc(_) => MspCommandCode::MSP_SET_RAW_RC,
            MspRequest::MspRawImu => MspCommandCode::MSP_RAW_IMU,
            _ => MspCommandCode::MSP_API_VERSION,
        }
    }
}

impl From<MspRequest> for MspPacket {
    fn from(request: MspRequest) -> Self {
        match request {
            MspRequest::MspBatteryState => MspPacket {
                cmd: MspCommandCode::MSP_BATTERY_STATE.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
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
            MspRequest::MspBatteryState => MspPacket {
                cmd: MspCommandCode::MSP_BATTERY_STATE.to_primitive(),
                direction: ToFlightController,
                data: MspPacketData::new(), // empty
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
        let mut result = MspPacketData::new();
        match self {
            MspResponse::MspApiVersion(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspFlightControllerVariant(data) => {
                data.pack_to_slice(result.as_mut_slice())?
            }
            MspResponse::MspStatus(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspStatusEx(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspBfConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspRawImu(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspDataFlashSummaryReply(data) => {
                data.pack_to_slice(result.as_mut_slice())?
            }
            MspResponse::MspDataFlashReply(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspDataFlashRead(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspAccTrim(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspIdent(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspMisc(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspAttitude(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspAltitude(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspBatteryConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspAnalog(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspRssiConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspVoltageMeter(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspCurrentMeter(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspBatteryState(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspRcTuning(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspRxConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspRcChannelValue(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspRcMappedChannel(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspFeatures(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspMotor(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspMotor3DConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspMotorConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspRcDeadband(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspSensorAlignment(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspAdvancedConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspFilterConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspPidAdvanced(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspSensorConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspServos(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspMixerConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspModeRange(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspSetModeRange(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspOsdConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspSetGetOsdConfig(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspSetOsdLayout(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspSetOsdLayoutItem(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspOsdLayouts(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspSerialSetting(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspSettingInfoRequest(data) => {
                data.pack_to_slice(result.as_mut_slice())?
            }
            MspResponse::MspSettingInfo(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::MspRc(data) => data.pack_to_slice(result.as_mut_slice())?,
            MspResponse::Unknown => return Err(PackingError::InvalidValue),
        }
        Ok(result)
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
        match packet.cmd.into() {
            MspCommandCode::MSP_API_VERSION => {
                MspResponse::MspApiVersion(packet.decode_as::<MspApiVersion>().unwrap())
            }
            MspCommandCode::MSP_FC_VARIANT => MspResponse::MspFlightControllerVariant(
                packet.decode_as::<MspFlightControllerVariant>().unwrap(),
            ),
            MspCommandCode::MSP_STATUS => {
                MspResponse::MspStatus(packet.decode_as::<MspStatus>().unwrap())
            }
            MspCommandCode::MSP_STATUS_EX => {
                MspResponse::MspStatusEx(packet.decode_as::<MspStatusEx>().unwrap())
            }
            MspCommandCode::MSP_BF_CONFIG => {
                MspResponse::MspBfConfig(packet.decode_as::<MspBfConfig>().unwrap())
            }
            MspCommandCode::MSP_RAW_IMU => {
                MspResponse::MspRawImu(packet.decode_as::<MspRawImu>().unwrap())
            }
            MspCommandCode::MSP_DATAFLASH_SUMMARY => MspResponse::MspDataFlashSummaryReply(
                packet.decode_as::<MspDataFlashSummaryReply>().unwrap(),
            ),
            MspCommandCode::MSP_DATAFLASH_READ => {
                MspResponse::MspDataFlashReply(packet.decode_as::<MspDataFlashReply>().unwrap())
            }
            MspCommandCode::MSP_ACC_TRIM => {
                MspResponse::MspAccTrim(packet.decode_as::<MspAccTrim>().unwrap())
            }
            MspCommandCode::MSP_IDENT => {
                MspResponse::MspIdent(packet.decode_as::<MspIdent>().unwrap())
            }
            MspCommandCode::MSP_MISC => {
                MspResponse::MspMisc(packet.decode_as::<MspMisc>().unwrap())
            }
            MspCommandCode::MSP_ATTITUDE => {
                MspResponse::MspAttitude(packet.decode_as::<MspAttitude>().unwrap())
            }
            MspCommandCode::MSP_ALTITUDE => {
                MspResponse::MspAltitude(packet.decode_as::<MspAltitude>().unwrap())
            }
            MspCommandCode::MSP_BATTERY_CONFIG => {
                MspResponse::MspBatteryConfig(packet.decode_as::<MspBatteryConfig>().unwrap())
            }
            MspCommandCode::MSP_ANALOG => {
                MspResponse::MspAnalog(packet.decode_as::<MspAnalog>().unwrap())
            }
            MspCommandCode::MSP_RSSI_CONFIG => {
                MspResponse::MspRssiConfig(packet.decode_as::<MspRssiConfig>().unwrap())
            }
            MspCommandCode::MSP_VOLTAGE_METERS => {
                MspResponse::MspVoltageMeter(packet.decode_as::<MspVoltageMeter>().unwrap())
            }
            MspCommandCode::MSP_AMPERAGE_METER_CONFIG => {
                MspResponse::MspCurrentMeter(packet.decode_as::<MspCurrentMeter>().unwrap())
            }
            MspCommandCode::MSP_BATTERY_STATE => {
                MspResponse::MspBatteryState(packet.decode_as::<MspBatteryState>().unwrap())
            }
            MspCommandCode::MSP_RC_TUNING => {
                MspResponse::MspRcTuning(packet.decode_as::<MspRcTuning>().unwrap())
            }
            MspCommandCode::MSP_RX_CONFIG => {
                MspResponse::MspRxConfig(packet.decode_as::<MspRxConfig>().unwrap())
            }
            MspCommandCode::MSP_RX_MAP => {
                MspResponse::MspRcChannelValue(packet.decode_as::<MspRcChannelValue>().unwrap())
            }
            MspCommandCode::MSP_SET_RX_MAP => {
                MspResponse::MspRcMappedChannel(packet.decode_as::<MspRcMappedChannel>().unwrap())
            }
            MspCommandCode::MSP_FEATURE => {
                MspResponse::MspFeatures(packet.decode_as::<MspFeatures>().unwrap())
            }
            MspCommandCode::MSP_MOTOR => {
                MspResponse::MspMotor(packet.decode_as::<MspMotor>().unwrap())
            }
            MspCommandCode::MSP_MOTOR_3D_CONFIG => {
                MspResponse::MspMotor3DConfig(packet.decode_as::<MspMotor3DConfig>().unwrap())
            }
            MspCommandCode::MSP_MOTOR_CONFIG => {
                MspResponse::MspMotorConfig(packet.decode_as::<MspMotorConfig>().unwrap())
            }
            MspCommandCode::MSP_RC_DEADBAND => {
                MspResponse::MspRcDeadband(packet.decode_as::<MspRcDeadband>().unwrap())
            }
            MspCommandCode::MSP_BOARD_ALIGNMENT => {
                MspResponse::MspSensorAlignment(packet.decode_as::<MspSensorAlignment>().unwrap())
            }
            MspCommandCode::MSP_ADVANCED_CONFIG => {
                MspResponse::MspAdvancedConfig(packet.decode_as::<MspAdvancedConfig>().unwrap())
            }
            MspCommandCode::MSP_FILTER_CONFIG => {
                MspResponse::MspFilterConfig(packet.decode_as::<MspFilterConfig>().unwrap())
            }
            MspCommandCode::MSP_PID_ADVANCED => {
                MspResponse::MspPidAdvanced(packet.decode_as::<MspPidAdvanced>().unwrap())
            }
            MspCommandCode::MSP_SENSOR_CONFIG => {
                MspResponse::MspSensorConfig(packet.decode_as::<MspSensorConfig>().unwrap())
            }
            MspCommandCode::MSP_SERVO => {
                MspResponse::MspServos(packet.decode_as::<MspServos>().unwrap())
            }
            MspCommandCode::MSP_MIXER => {
                MspResponse::MspMixerConfig(packet.decode_as::<MspMixerConfig>().unwrap())
            }
            MspCommandCode::MSP_MODE_RANGES => {
                MspResponse::MspModeRange(packet.decode_as::<MspModeRange>().unwrap())
            }
            MspCommandCode::MSP_SET_MODE_RANGE => {
                MspResponse::MspSetModeRange(packet.decode_as::<MspSetModeRange>().unwrap())
            }
            MspCommandCode::MSP_OSD_CONFIG => {
                MspResponse::MspOsdConfig(packet.decode_as::<MspOsdConfig>().unwrap())
            }
            MspCommandCode::MSP_OSD_LAYOUT_CONFIG => {
                MspResponse::MspSetOsdLayout(packet.decode_as::<MspSetOsdLayout>().unwrap())
            }
            MspCommandCode::MSP2_INAV_OSD_SET_LAYOUT_ITEM => {
                MspResponse::MspSetOsdLayoutItem(packet.decode_as::<MspSetOsdLayoutItem>().unwrap())
            }
            MspCommandCode::MSP2_INAV_OSD_LAYOUTS => {
                MspResponse::MspOsdLayouts(packet.decode_as::<MspOsdLayouts>().unwrap())
            }
            MspCommandCode::MSP2_SET_SERIAL_CONFIG => {
                MspResponse::MspSerialSetting(packet.decode_as::<MspSerialSetting>().unwrap())
            }
            MspCommandCode::MSP2_COMMON_SETTING => MspResponse::MspSettingInfoRequest(
                packet.decode_as::<MspSettingInfoRequest>().unwrap(),
            ),
            MspCommandCode::MSP2_COMMON_SETTING_INFO => {
                MspResponse::MspSettingInfo(packet.decode_as::<MspSettingInfo>().unwrap())
            }
            MspCommandCode::MSP_RC => MspResponse::MspRc(packet.decode_as::<MspRc>().unwrap()),
            _ => MspResponse::Unknown,
        }
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
