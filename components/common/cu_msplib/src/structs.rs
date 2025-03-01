//! MSP structures

use packed_struct::derive::{PackedStruct, PrimitiveEnum};
use serde::{Deserialize, Serialize};

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
pub struct MspApiVersion {
    pub protocol_version: u8,
    pub api_version_major: u8,
    pub api_version_minor: u8,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
pub struct MspFlightControllerVariant {
    pub identifier: [u8; 4],
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
pub struct MspFlightControllerVersion {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspBoardInfo {
    pub board_id: [u8; 4],
    pub hardware_revision: u16,
    pub fc_type: u8,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
pub struct MspBuildInfo {
    pub date_str: [u8; 11],
    pub time_str: [u8; 8],
    pub git_str: [u8; 7],
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
pub struct MspUniqueId {
    pub uid: [u8; 12],
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
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

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
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

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
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

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
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

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
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

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspDataFlashReply {
    pub read_address: u32,
    // pub payload: Vec<u8>, // TODO: packed_struct should support dynamic size too the end
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "6", endian = "lsb", bit_numbering = "msb0")]
pub struct MspDataFlashRead {
    pub read_address: u32,
    pub read_length: u16,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspAccTrim {
    pub pitch: u16,
    pub roll: u16,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspIdent {
    pub version: u8,
    pub mixer_mode: u8,
    pub protocol_version: u8,
    pub capability: u32,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
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

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspAttitude {
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspAltitude {
    /// [centimeters]
    pub altitude: i32,
    /// variometer [cm/s]
    pub vario: i16,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspBatteryConfig {
    pub vbat_min_cell_voltage: u8,
    pub vbat_max_cell_voltage: u8,
    pub vbat_warning_cell_voltage: u8,
    pub battery_capacity: u16,
    pub voltage_meter_source: u8,
    pub current_meter_source: u8,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspAnalog {
    pub battery_voltage: u8,
    pub mah_drawn: u16,
    pub rssi: u16,
    /// Current in 0.01A steps, range is -320A to 320A
    pub amperage: i16,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspRssiConfig {
    pub rssi_channel: u8,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
pub struct MspVoltageMeter {
    pub id: u8,
    pub value: u8,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspCurrentMeter {
    pub id: u8,
    pub mah_drawn: u16,
    /// In 0.001A steps (mA)
    pub amperage: u16,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
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

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspRcChannelValue {
    pub value: u16,
}

#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq)]
pub enum MspRcChannel {
    /// Ailerons
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

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
pub struct MspRcMappedChannel {
    #[packed_field(size_bits = "8", ty = "enum")]
    pub channel: MspRcChannel,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
pub struct MspFeatures {
    pub features: [bool; 32],
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspMotor {
    pub motors: [u16; 8],
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspMotor3DConfig {
    pub deadband_3d_low: u16,
    pub deadband_3d_high: u16,
    pub neutral_3d: u16,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspMotorConfig {
    pub min_throttle: u16,
    pub max_throttle: u16,
    pub min_command: u16,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspRcDeadband {
    pub deadband: u8,
    pub yaw_deadband: u8,
    pub alt_hold_deadband: u8,
    pub deadband_3d_throttle: u16,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspSensorAlignment {
    pub gyro_alignment: u8,
    pub acc_alignment: u8,
    pub mag_alignment: u8,
}

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

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspSensorConfig {
    pub acc_hardware: u8,
    pub baro_hardware: u8,
    pub mag_hardware: u8,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone, Default)]
#[packed_struct(endian = "lsb")]
pub struct MspServos {
    pub servos: [u16; 8],
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "14", endian = "lsb", bit_numbering = "msb0")]
pub struct MspServoConfig {
    pub min: u16,
    pub max: u16,
    pub middle: u16,
    pub rate: i8,
    pub unused1: u8,
    pub unused2: u8,
    pub forward_from_channel: u8, // Depracted, set to 255 for backward compatibility
    pub reverse_input: u32, // Depracted, Input reversing is not required since it can be done on mixer level
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetServoConfig {
    pub index: u8,
    #[packed_field(size_bytes = "14")]
    pub servo_config: MspServoConfig,
}

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspMixerConfig {
    #[packed_field(size_bits = "8", ty = "enum")]
    pub mixer_mode: MixerMode,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "4", endian = "lsb", bit_numbering = "msb0")]
pub struct MspModeRange {
    pub box_id: u8,
    #[packed_field(size_bits = "8", ty = "enum")]
    pub aux_channel_index: MspRcChannel,
    pub start_step: u8,
    pub end_step: u8,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "5", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetModeRange {
    pub index: u8,
    #[packed_field(size_bytes = "4")]
    pub mode_range: MspModeRange,
}

#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq)]
pub enum MixerMode {
    Tri = 1,
    QuadPlus = 2,
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

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "8", endian = "lsb", bit_numbering = "msb0")]
pub struct MspMotorMixer {
    pub throttle: u16,
    pub roll: u16,
    pub pitch: u16,
    pub yaw: u16,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "9", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetMotorMixer {
    pub index: u8,
    #[packed_field(size_bytes = "8")]
    pub motor_mixer: MspMotorMixer,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
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

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetGetOsdConfig {
    pub item_index: u8,
    #[packed_field(size_bytes = "13")]
    pub config: MspOsdConfig,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "2", endian = "lsb", bit_numbering = "msb0")]
pub struct MspOsdItemPosition {
    pub col: u8,
    pub row: u8,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetOsdLayout {
    pub item_index: u8,
    #[packed_field(size_bytes = "2")]
    pub item: MspOsdItemPosition,
}

// inav msp layout item
#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetOsdLayoutItem {
    pub layout_index: u8,
    #[packed_field(size_bytes = "3")]
    pub item: MspSetOsdLayout,
}

#[derive(Debug)]
pub struct MspOsdSettings {
    pub osd_support: u8,
    pub config: MspOsdConfig,
    pub item_positions: Vec<MspOsdItemPosition>,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "2", endian = "lsb", bit_numbering = "msb0")]
pub struct MspOsdLayouts {
    pub layout_count: u8,
    pub item_count: u8,
}

#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq)]
pub enum SerialIdentifier {
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

#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq)]
pub enum Baudrate {
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

#[derive(PackedStruct, Debug, Copy, Clone)]
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

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetServoMixRule {
    pub index: u8,
    #[packed_field(size_bytes = "8")]
    pub servo_rule: MspServoMixRule,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
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

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetServoMixer {
    pub index: u8,
    #[packed_field(size_bytes = "6")]
    pub servo_rule: MspServoMixer,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "6", endian = "lsb", bit_numbering = "msb0")]
pub struct MspServoMixer {
    pub target_channel: u8,
    pub input_source: u8,
    pub rate: i16,
    pub speed: u8,
    pub condition_id: i8,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb", bit_numbering = "msb0")]
pub struct MspRxMap {
    pub map: [u8; 4], // MAX_MAPPABLE_RX_INPUTS
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb", bit_numbering = "msb0")]
pub struct MspSettingGroup {
    pub group_id: u16,
    pub start_id: u16,
    pub end_id: u16,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb", bit_numbering = "msb0")]
pub struct MspSettingInfoRequest {
    pub null: u8,
    pub id: u16,
}

#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq)]
pub enum SettingMode {
    ModeDirect = 0,
    ModeLookup = 0x40,
}

#[derive(PrimitiveEnum, Serialize, Deserialize, Debug, Copy, Clone, PartialEq)]
pub enum SettingType {
    VarUint8 = 0,
    VarInt8,
    VarUint16,
    VarInt16,
    VarUint32,
    VarInt32,
    VarFloat,
    VarString,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
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

#[derive(PackedStruct, Serialize, Deserialize, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb")]
pub struct MspRc {
    pub channels: [u16; 16], // 16 RC channels
}

impl MspRc {
    pub fn new() -> Self {
        MspRc {
            channels: [0; 16],
        }
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
