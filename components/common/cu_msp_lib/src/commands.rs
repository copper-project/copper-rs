use packed_struct::derive::PrimitiveEnum;
use packed_struct::PrimitiveEnum;

#[derive(PrimitiveEnum, Debug, Copy, Clone, PartialEq)]
#[allow(non_camel_case_types)]
/// MSP command values, used for command encapsulation
pub enum MspCommandCode {
    MSP_API_VERSION = 1,
    MSP_FC_VARIANT = 2,
    MSP_FC_VERSION = 3,
    MSP_BOARD_INFO = 4,
    MSP_BUILD_INFO = 5,

    // MSP commands for Cleanflight original features
    MSP_BATTERY_CONFIG = 32,
    MSP_SET_BATTERY_CONFIG = 33,
    MSP_MODE_RANGES = 34,
    MSP_SET_MODE_RANGE = 35,
    MSP_FEATURE = 36,
    MSP_SET_FEATURE = 37,
    MSP_BOARD_ALIGNMENT = 38,
    MSP_SET_BOARD_ALIGNMENT = 39,
    MSP_AMPERAGE_METER_CONFIG = 40,
    MSP_SET_AMPERAGE_METER_CONFIG = 41,
    MSP_MIXER = 42,
    MSP_SET_MIXER = 43,
    MSP_RX_CONFIG = 44,
    MSP_SET_RX_CONFIG = 45,
    MSP_LED_COLORS = 46,
    MSP_SET_LED_COLORS = 47,
    MSP_LED_STRIP_CONFIG = 48,
    MSP_SET_LED_STRIP_CONFIG = 49,
    MSP_RSSI_CONFIG = 50,
    MSP_SET_RSSI_CONFIG = 51,
    MSP_ADJUSTMENT_RANGES = 52,
    MSP_SET_ADJUSTMENT_RANGE = 53,
    MSP_CF_SERIAL_CONFIG = 54,
    MSP_SET_CF_SERIAL_CONFIG = 55,
    MSP_VOLTAGE_METER_CONFIG = 56,
    MSP_SET_VOLTAGE_METER_CONFIG = 57,
    MSP_SONAR = 58,
    MSP_PID_CONTROLLER = 59,
    MSP_SET_PID_CONTROLLER = 60,
    MSP_ARMING_CONFIG = 61,
    MSP_SET_ARMING_CONFIG = 62,
    MSP_DATAFLASH_SUMMARY = 70,
    MSP_DATAFLASH_READ = 71,
    MSP_DATAFLASH_ERASE = 72,
    MSP_LOOP_TIME = 73,
    MSP_SET_LOOP_TIME = 74,
    MSP_FAILSAFE_CONFIG = 75,
    MSP_SET_FAILSAFE_CONFIG = 76,
    MSP_RXFAIL_CONFIG = 77,
    MSP_SET_RXFAIL_CONFIG = 78,
    MSP_SDCARD_SUMMARY = 79,
    MSP_BLACKBOX_CONFIG = 80,
    MSP_SET_BLACKBOX_CONFIG = 81,
    MSP_TRANSPONDER_CONFIG = 82,
    MSP_SET_TRANSPONDER_CONFIG = 83,

    MSP_OSD_CONFIG = 84,     //out message         Get osd settings - baseflight
    MSP_SET_OSD_CONFIG = 85, //in message          Set osd settings - baseflight

    MSP_OSD_CHAR_READ = 86, //out message         Get osd settings - betaflight
    MSP_OSD_CHAR_WRITE = 87, //in message          Set osd settings - betaflight

    MSP_LED_STRIP_MODECOLOR = 127,
    MSP_SET_LED_STRIP_MODECOLOR = 221,

    MSP_VOLTAGE_METERS = 128,
    MSP_AMPERAGE_METERS = 129,
    MSP_BATTERY_STATE = 130,

    MSP_MOTOR_CONFIG = 131,

    // OSD commands
    MSP_OSD_VIDEO_CONFIG = 180,
    MSP_SET_OSD_VIDEO_CONFIG = 181,
    MSP_OSD_VIDEO_STATUS = 182,
    MSP_OSD_ELEMENT_SUMMARY = 183,
    MSP_OSD_LAYOUT_CONFIG = 184,
    MSP_SET_OSD_LAYOUT_CONFIG = 185,

    // Multiwii MSP commands
    MSP_IDENT = 100,
    MSP_STATUS = 101,
    MSP_RAW_IMU = 102,
    MSP_SERVO = 103,
    MSP_MOTOR = 104,
    MSP_RC = 105,
    MSP_RAW_GPS = 106,
    MSP_COMP_GPS = 107,
    MSP_ATTITUDE = 108,
    MSP_ALTITUDE = 109,
    MSP_ANALOG = 110,
    MSP_RC_TUNING = 111,
    MSP_PID = 112,
    MSP_BOX = 113,
    MSP_MISC = 114,
    MSP_MOTOR_PINS = 115,
    MSP_BOXNAMES = 116,
    MSP_PIDNAMES = 117,
    MSP_WP = 118,
    MSP_BOXIDS = 119,
    MSP_SERVO_CONFIGURATIONS = 120,
    MSP_MOTOR_3D_CONFIG = 124,
    MSP_RC_DEADBAND = 125,
    MSP_SENSOR_ALIGNMENT = 126,

    MSP_SET_RAW_RC = 200,
    MSP_SET_RAW_GPS = 201,
    MSP_SET_PID = 202,
    MSP_SET_BOX = 203,
    MSP_SET_RC_TUNING = 204,
    MSP_ACC_CALIBRATION = 205,
    MSP_MAG_CALIBRATION = 206,
    MSP_SET_MISC = 207,
    MSP_RESET_CONF = 208,
    MSP_SET_WP = 209,
    MSP_SELECT_SETTING = 210,
    MSP_SET_HEAD = 211,
    MSP_SET_SERVO_CONFIGURATION = 212,
    MSP_SET_MOTOR = 214,
    MSP_SET_3D = 217,
    MSP_SET_RC_DEADBAND = 218,
    MSP_SET_RESET_CURR_PID = 219,
    MSP_SET_SENSOR_ALIGNMENT = 220,

    // MSP_BIND =               240,
    MSP_SERVO_MIX_RULES = 241,
    MSP_SET_SERVO_MIX_RULE = 242,

    MSP_EEPROM_WRITE = 250,

    MSP_DEBUGMSG = 253,
    MSP_DEBUG = 254,

    MSP_BF_CONFIG = 66,

    // Additional baseflight commands that are not compatible with MultiWii
    MSP_UID = 160,          // Unique device ID
    MSP_STATUS_EX = 150,    // cycletime, errors_count, CPU load, sensor present etc
    MSP_ACC_TRIM = 240,     // get acc angle trim values
    MSP_SET_ACC_TRIM = 239, // set acc angle trim values
    MSP_GPS_SV_INFO = 164,  // get Signal Strength

    // Additional private MSP for baseflight configurator
    MSP_RX_MAP = 64,     // get channel map (also returns number of channels total)
    MSP_SET_RX_MAP = 65, // set rc map, numchannels to set comes from MSP_RX_MAP

    MSP_SET_REBOOT = 68,    // reboot settings
    MSP_BF_BUILD_INFO = 69, // build date as well as some space for future expansion,

    // Betaflight
    MSP_ADVANCED_CONFIG = 90,
    MSP_SET_ADVANCED_CONFIG = 91,

    MSP_FILTER_CONFIG = 92,
    MSP_SET_FILTER_CONFIG = 93,

    MSP_PID_ADVANCED = 94,
    MSP_SET_PID_ADVANCED = 95,

    MSP_SENSOR_CONFIG = 96,
    MSP_SET_SENSOR_CONFIG = 97,

    // Inav
    MSP2_COMMON_SETTING = 0x1003, //in/out message    Returns the value for a setting
    MSP2_COMMON_SET_SETTING = 0x1004, //in message        Sets the value for a setting

    MSP2_MOTOR_MIXER = 0x1005,
    MSP2_SET_MOTOR_MIXER = 0x1006,

    MSP2_COMMON_SETTING_INFO = 0x1007,
    MSP2_COMMON_PG_LIST = 0x1008,

    MSP2_SERIAL_CONFIG = 0x1009,
    MSP2_SET_SERIAL_CONFIG = 0x100A,

    MSP2_INAV_OSD_LAYOUTS = 0x2012,
    MSP2_INAV_OSD_SET_LAYOUT_ITEM = 0x2013,
    MSP2_INAV_OSD_ALARMS = 0x2014,
    MSP2_INAV_OSD_SET_ALARMS = 0x2015,
    MSP2_INAV_OSD_PREFERENCES = 0x2016,
    MSP2_INAV_OSD_SET_PREFERENCES = 0x2017,

    MSP2_INAV_SERVO_MIXER = 0x2020,
    MSP2_INAV_SET_SERVO_MIXER = 0x2021,
}

impl From<u16> for MspCommandCode {
    fn from(value: u16) -> Self {
        Self::from_primitive(value).unwrap_or_else(|| panic!("Invalid MSP command code: {value}"))
    }
}
