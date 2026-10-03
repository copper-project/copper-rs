use core::ffi::{c_char, c_int, c_uchar, c_uint, c_void};

pub const MAX_CAMERA_PLUGIN: usize = 20;

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_Quaternion {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_Vector2 {
    pub x: f32,
    pub y: f32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_Uint2 {
    pub x: c_uint,
    pub y: c_uint,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_Vector4 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_Uchar2 {
    pub x: c_uchar,
    pub y: c_uchar,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_Uchar3 {
    pub x: c_uchar,
    pub y: c_uchar,
    pub z: c_uchar,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_Uchar4 {
    pub x: c_uchar,
    pub y: c_uchar,
    pub z: c_uchar,
    pub w: c_uchar,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_Matrix4f {
    pub p: [f32; 16],
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_Matrix3f {
    pub p: [f32; 9],
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct SL_PoseData {
    pub valid: bool,
    pub timestamp: u64,
    pub rotation: SL_Quaternion,
    pub translation: SL_Vector3,
    pub pose_confidence: c_int,
    pub pose_covariance: [f32; 36],
    pub twist: [f32; 6],
    pub twist_covariance: [f32; 36],
}

impl Default for SL_PoseData {
    fn default() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_IMUData {
    pub is_available: bool,
    pub timestamp_ns: u64,
    pub angular_velocity: SL_Vector3,
    pub linear_acceleration: SL_Vector3,
    pub angular_velocity_unc: SL_Vector3,
    pub linear_acceleration_unc: SL_Vector3,
    pub orientation: SL_Quaternion,
    pub orientation_covariance: SL_Matrix3f,
    pub angular_velocity_convariance: SL_Matrix3f,
    pub linear_acceleration_convariance: SL_Matrix3f,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_BarometerData {
    pub is_available: bool,
    pub timestamp_ns: u64,
    pub pressure: f32,
    pub relative_altitude: f32,
}

pub type SL_HEADING_STATE = c_uint;
pub const SL_HEADING_STATE_SL_HEADING_STATE_GOOD: SL_HEADING_STATE = 0;
pub const SL_HEADING_STATE_SL_HEADING_STATE_OK: SL_HEADING_STATE = 1;
pub const SL_HEADING_STATE_SL_HEADING_STATE_NOT_GOOD: SL_HEADING_STATE = 2;
pub const SL_HEADING_STATE_SL_HEADING_STATE_NOT_CALIBRATED: SL_HEADING_STATE = 3;
pub const SL_HEADING_STATE_SL_HEADING_STATE_MAG_NOT_AVAILABLE: SL_HEADING_STATE = 4;

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_MagnetometerData {
    pub is_available: bool,
    pub timestamp_ns: u64,
    pub magnetic_field_c: SL_Vector3,
    pub magnetic_field_unc: SL_Vector3,
    pub magnetic_heading: f32,
    pub magnetic_heading_state: SL_HEADING_STATE,
    pub magnetic_heading_accuracy: f32,
    pub effective_rate: f32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_TemperatureData {
    pub imu_temp: f32,
    pub barometer_temp: f32,
    pub onboard_left_temp: f32,
    pub onboard_right_temp: f32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_SensorsData {
    pub imu: SL_IMUData,
    pub barometer: SL_BarometerData,
    pub magnetometer: SL_MagnetometerData,
    pub temperature: SL_TemperatureData,
    pub camera_moving_state: c_int,
    pub image_sync_trigger: c_int,
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_ERROR_CODE {
    SL_ERROR_CODE_SENSOR_CONFIGURATION_CHANGED = -6,
    SL_ERROR_CODE_POTENTIAL_CALIBRATION_ISSUE = -5,
    SL_ERROR_CODE_CONFIGURATION_FALLBACK = -4,
    SL_ERROR_CODE_SENSORS_DATA_REQUIRED = -3,
    SL_ERROR_CODE_CORRUPTED_FRAME = -2,
    SL_ERROR_CODE_CAMERA_REBOOTING = -1,
    SL_ERROR_CODE_SUCCESS = 0,
    SL_ERROR_CODE_FAILURE = 1,
    SL_ERROR_CODE_NO_GPU_COMPATIBLE = 2,
    SL_ERROR_CODE_NOT_ENOUGH_GPU_MEMORY = 3,
    SL_ERROR_CODE_CAMERA_NOT_DETECTED = 4,
    SL_ERROR_CODE_SENSORS_NOT_INITIALIZED = 5,
    SL_ERROR_CODE_SENSORS_NOT_AVAILABLE = 6,
    SL_ERROR_CODE_INVALID_RESOLUTION = 7,
    SL_ERROR_CODE_LOW_USB_BANDWIDTH = 8,
    SL_ERROR_CODE_CALIBRATION_FILE_NOT_AVAILABLE = 9,
    SL_ERROR_CODE_INVALID_CALIBRATION_FILE = 10,
    SL_ERROR_CODE_INVALID_SVO_FILE = 11,
    SL_ERROR_CODE_SVO_RECORDING_ERROR = 12,
    SL_ERROR_CODE_SVO_UNSUPPORTED_COMPRESSION = 13,
    SL_ERROR_CODE_END_OF_SVOFILE_REACHED = 14,
    SL_ERROR_CODE_INVALID_COORDINATE_SYSTEM = 15,
    SL_ERROR_CODE_INVALID_FIRMWARE = 16,
    SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS = 17,
    SL_ERROR_CODE_CUDA_ERROR = 18,
    SL_ERROR_CODE_CAMERA_NOT_INITIALIZED = 19,
    SL_ERROR_CODE_NVIDIA_DRIVER_OUT_OF_DATE = 20,
    SL_ERROR_CODE_INVALID_FUNCTION_CALL = 21,
    SL_ERROR_CODE_CORRUPTED_SDK_INSTALLATION = 22,
    SL_ERROR_CODE_INCOMPATIBLE_SDK_VERSION = 23,
    SL_ERROR_CODE_INVALID_AREA_FILE = 24,
    SL_ERROR_CODE_INCOMPATIBLE_AREA_FILE = 25,
    SL_ERROR_CODE_CAMERA_FAILED_TO_SETUP = 26,
    SL_ERROR_CODE_CAMERA_DETECTION_ISSUE = 27,
    SL_ERROR_CODE_CANNOT_START_CAMERA_STREAM = 28,
    SL_ERROR_CODE_NO_GPU_DETECTED = 29,
    SL_ERROR_CODE_PLANE_NOT_FOUND = 30,
    SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CAMERA = 31,
    SL_ERROR_CODE_MOTION_SENSORS_REQUIRED = 32,
    SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CUDA_VERSION = 33,
    SL_ERROR_CODE_DRIVER_FAILURE = 34,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_RESOLUTION {
    SL_RESOLUTION_HD4K = 0,
    SL_RESOLUTION_QHDPLUS = 1,
    SL_RESOLUTION_HD2K = 2,
    SL_RESOLUTION_HD1536 = 3,
    SL_RESOLUTION_HD1080 = 4,
    SL_RESOLUTION_HD1200 = 5,
    SL_RESOLUTION_HD720 = 6,
    SL_RESOLUTION_SVGA = 7,
    SL_RESOLUTION_VGA = 8,
    SL_RESOLUTION_AUTO = 9,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_UNIT {
    SL_UNIT_MILLIMETER = 0,
    SL_UNIT_CENTIMETER = 1,
    SL_UNIT_METER = 2,
    SL_UNIT_INCH = 3,
    SL_UNIT_FOOT = 4,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_COORDINATE_SYSTEM {
    SL_COORDINATE_SYSTEM_IMAGE = 0,
    SL_COORDINATE_SYSTEM_LEFT_HANDED_Y_UP = 1,
    SL_COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP = 2,
    SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP = 3,
    SL_COORDINATE_SYSTEM_LEFT_HANDED_Z_UP = 4,
    SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD = 5,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_CAMERA_STATE {
    SL_CAMERA_STATE_AVAILABLE = 0,
    SL_CAMERA_STATE_NOT_AVAILABLE = 1,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_MODEL {
    SL_MODEL_ZED = 0,
    SL_MODEL_ZED_M = 1,
    SL_MODEL_ZED2 = 2,
    SL_MODEL_ZED2i = 3,
    SL_MODEL_ZED_X = 4,
    SL_MODEL_ZED_XM = 5,
    SL_MODEL_ZED_X_HDR = 6,
    SL_MODEL_ZED_X_HDR_MINI = 7,
    SL_MODEL_ZED_X_HDR_MAX = 8,
    SL_MODEL_VIRTUAL_ZED_X = 11,
    SL_MODEL_ZED_XONE_GS = 30,
    SL_MODEL_ZED_XONE_UHD = 31,
    SL_MODEL_ZED_XONE_HDR = 32,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_MEM {
    SL_MEM_CPU = 0,
    SL_MEM_GPU = 1,
    SL_MEM_BOTH = 2,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_SENSOR_TYPE {
    SL_SENSOR_TYPE_ACCELEROMETER = 0,
    SL_SENSOR_TYPE_GYROSCOPE = 1,
    SL_SENSOR_TYPE_MAGNETOMETER = 2,
    SL_SENSOR_TYPE_BAROMETER = 3,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_SENSORS_UNIT {
    SL_SENSORS_UNIT_M_SEC_2 = 0,
    SL_SENSORS_UNIT_DEG_SEC = 1,
    SL_SENSORS_UNIT_U_T = 2,
    SL_SENSORS_UNIT_HPA = 3,
    SL_SENSORS_UNIT_CELSIUS = 4,
    SL_SENSORS_UNIT_HERTZ = 5,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_INPUT_TYPE {
    SL_INPUT_TYPE_USB = 0,
    SL_INPUT_TYPE_SVO = 1,
    SL_INPUT_TYPE_STREAM = 2,
    SL_INPUT_TYPE_GMSL = 3,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_REFERENCE_FRAME {
    SL_REFERENCE_FRAME_WORLD = 0,
    SL_REFERENCE_FRAME_CAMERA = 1,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_TIME_REFERENCE {
    SL_TIME_REFERENCE_IMAGE = 0,
    SL_TIME_REFERENCE_CURRENT = 1,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_MEASURE {
    SL_MEASURE_DISPARITY = 0,
    SL_MEASURE_DEPTH = 1,
    SL_MEASURE_CONFIDENCE = 2,
    SL_MEASURE_XYZ = 3,
    SL_MEASURE_XYZRGBA = 4,
    SL_MEASURE_XYZBGRA = 5,
    SL_MEASURE_XYZARGB = 6,
    SL_MEASURE_XYZABGR = 7,
    SL_MEASURE_NORMALS = 8,
    SL_MEASURE_DISPARITY_RIGHT = 9,
    SL_MEASURE_DEPTH_RIGHT = 10,
    SL_MEASURE_XYZ_RIGHT = 11,
    SL_MEASURE_XYZRGBA_RIGHT = 12,
    SL_MEASURE_XYZBGRA_RIGHT = 13,
    SL_MEASURE_XYZARGB_RIGHT = 14,
    SL_MEASURE_XYZABGR_RIGHT = 15,
    SL_MEASURE_NORMALS_RIGHT = 16,
    SL_MEASURE_DEPTH_U16_MM = 17,
    SL_MEASURE_DEPTH_U16_MM_RIGHT = 18,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_VIEW {
    SL_VIEW_LEFT = 0,
    SL_VIEW_RIGHT = 1,
    SL_VIEW_LEFT_GRAY = 2,
    SL_VIEW_RIGHT_GRAY = 3,
    SL_VIEW_LEFT_NV12_UNRECTIFIED = 4,
    SL_VIEW_RIGHT_NV12_UNRECTIFIED = 5,
    SL_VIEW_LEFT_UNRECTIFIED = 6,
    SL_VIEW_RIGHT_UNRECTIFIED = 7,
    SL_VIEW_LEFT_UNRECTIFIED_GRAY = 8,
    SL_VIEW_RIGHT_UNRECTIFIED_GRAY = 9,
    SL_VIEW_SIDE_BY_SIDE = 10,
    SL_VIEW_DEPTH = 11,
    SL_VIEW_CONFIDENCE = 12,
    SL_VIEW_NORMALS = 13,
    SL_VIEW_DEPTH_RIGHT = 14,
    SL_VIEW_NORMALS_RIGHT = 15,
    SL_VIEW_LEFT_BGRA = 16,
    SL_VIEW_LEFT_BGR = 17,
    SL_VIEW_RIGHT_BGRA = 18,
    SL_VIEW_RIGHT_BGR = 19,
    SL_VIEW_LEFT_UNRECTIFIED_BGRA = 20,
    SL_VIEW_LEFT_UNRECTIFIED_BGR = 21,
    SL_VIEW_RIGHT_UNRECTIFIED_BGRA = 22,
    SL_VIEW_RIGHT_UNRECTIFIED_BGR = 23,
    SL_VIEW_SIDE_BY_SIDE_BGRA = 24,
    SL_VIEW_SIDE_BY_SIDE_BGR = 25,
    SL_VIEW_SIDE_BY_SIDE_GRAY = 26,
    SL_VIEW_SIDE_BY_SIDE_UNRECTIFIED_BGRA = 27,
    SL_VIEW_SIDE_BY_SIDE_UNRECTIFIED_BGR = 28,
    SL_VIEW_SIDE_BY_SIDE_UNRECTIFIED_GRAY = 29,
    SL_VIEW_DEPTH_BGRA = 30,
    SL_VIEW_DEPTH_BGR = 31,
    SL_VIEW_DEPTH_GRAY = 32,
    SL_VIEW_CONFIDENCE_BGRA = 33,
    SL_VIEW_CONFIDENCE_BGR = 34,
    SL_VIEW_CONFIDENCE_GRAY = 35,
    SL_VIEW_NORMALS_BGRA = 36,
    SL_VIEW_NORMALS_BGR = 37,
    SL_VIEW_NORMALS_GRAY = 38,
    SL_VIEW_DEPTH_RIGHT_BGRA = 39,
    SL_VIEW_DEPTH_RIGHT_BGR = 40,
    SL_VIEW_DEPTH_RIGHT_GRAY = 41,
    SL_VIEW_NORMALS_RIGHT_BGRA = 42,
    SL_VIEW_NORMALS_RIGHT_BGR = 43,
    SL_VIEW_NORMALS_RIGHT_GRAY = 44,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_DEPTH_MODE {
    SL_DEPTH_MODE_NONE = 0,
    SL_DEPTH_MODE_PERFORMANCE = 1,
    SL_DEPTH_MODE_QUALITY = 2,
    SL_DEPTH_MODE_ULTRA = 3,
    SL_DEPTH_MODE_NEURAL_LIGHT = 4,
    SL_DEPTH_MODE_NEURAL = 5,
    SL_DEPTH_MODE_NEURAL_PLUS = 6,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_FLIP_MODE {
    SL_FLIP_MODE_OFF = 0,
    SL_FLIP_MODE_ON = 1,
    SL_FLIP_MODE_AUTO = 2,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum SL_MAT_TYPE {
    SL_MAT_TYPE_F32_C1 = 0,
    SL_MAT_TYPE_F32_C2 = 1,
    SL_MAT_TYPE_F32_C3 = 2,
    SL_MAT_TYPE_F32_C4 = 3,
    SL_MAT_TYPE_U8_C1 = 4,
    SL_MAT_TYPE_U8_C2 = 5,
    SL_MAT_TYPE_U8_C3 = 6,
    SL_MAT_TYPE_U8_C4 = 7,
    SL_MAT_TYPE_U16_C1 = 8,
    SL_MAT_TYPE_S8_C4 = 9,
}

macro_rules! impl_default_enum {
    ($ty:ty, $value:path) => {
        impl Default for $ty {
            fn default() -> Self {
                $value
            }
        }
    };
}

impl_default_enum!(SL_ERROR_CODE, SL_ERROR_CODE::SL_ERROR_CODE_SUCCESS);
impl_default_enum!(SL_RESOLUTION, SL_RESOLUTION::SL_RESOLUTION_HD4K);
impl_default_enum!(SL_UNIT, SL_UNIT::SL_UNIT_MILLIMETER);
impl_default_enum!(
    SL_COORDINATE_SYSTEM,
    SL_COORDINATE_SYSTEM::SL_COORDINATE_SYSTEM_IMAGE
);
impl_default_enum!(SL_CAMERA_STATE, SL_CAMERA_STATE::SL_CAMERA_STATE_AVAILABLE);
impl_default_enum!(SL_MODEL, SL_MODEL::SL_MODEL_ZED);
impl_default_enum!(SL_MEM, SL_MEM::SL_MEM_CPU);
impl_default_enum!(SL_SENSOR_TYPE, SL_SENSOR_TYPE::SL_SENSOR_TYPE_ACCELEROMETER);
impl_default_enum!(SL_SENSORS_UNIT, SL_SENSORS_UNIT::SL_SENSORS_UNIT_M_SEC_2);
impl_default_enum!(SL_INPUT_TYPE, SL_INPUT_TYPE::SL_INPUT_TYPE_USB);
impl_default_enum!(
    SL_REFERENCE_FRAME,
    SL_REFERENCE_FRAME::SL_REFERENCE_FRAME_WORLD
);
impl_default_enum!(
    SL_TIME_REFERENCE,
    SL_TIME_REFERENCE::SL_TIME_REFERENCE_IMAGE
);
impl_default_enum!(SL_MEASURE, SL_MEASURE::SL_MEASURE_DISPARITY);
impl_default_enum!(SL_VIEW, SL_VIEW::SL_VIEW_LEFT);
impl_default_enum!(SL_DEPTH_MODE, SL_DEPTH_MODE::SL_DEPTH_MODE_NONE);
impl_default_enum!(SL_FLIP_MODE, SL_FLIP_MODE::SL_FLIP_MODE_OFF);
impl_default_enum!(SL_MAT_TYPE, SL_MAT_TYPE::SL_MAT_TYPE_F32_C1);

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_Resolution {
    pub width: c_int,
    pub height: c_int,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_InitParameters {
    pub input_type: SL_INPUT_TYPE,
    pub resolution: SL_RESOLUTION,
    pub camera_fps: c_int,
    pub camera_device_id: c_int,
    pub camera_image_flip: SL_FLIP_MODE,
    pub camera_disable_self_calib: bool,
    pub enable_right_side_measure: bool,
    pub svo_real_time_mode: bool,
    pub depth_mode: SL_DEPTH_MODE,
    pub depth_stabilization: c_int,
    pub depth_minimum_distance: f32,
    pub depth_maximum_distance: f32,
    pub coordinate_unit: SL_UNIT,
    pub coordinate_system: SL_COORDINATE_SYSTEM,
    pub sdk_gpu_id: c_int,
    pub sdk_verbose: c_int,
    pub sensors_required: bool,
    pub enable_image_enhancement: bool,
    pub open_timeout_sec: f32,
    pub async_grab_camera_recovery: bool,
    pub grab_compute_capping_fps: f32,
    pub enable_image_validity_check: bool,
    pub maximum_working_resolution: SL_Resolution,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_RuntimeParameters {
    pub reference_frame: SL_REFERENCE_FRAME,
    pub enable_depth: bool,
    pub enable_fill_mode: bool,
    pub confidence_threshold: c_int,
    pub texture_confidence_threshold: c_int,
    pub remove_saturated_areas: bool,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct SL_DeviceProperties {
    pub camera_state: SL_CAMERA_STATE,
    pub id: c_int,
    pub path: [c_char; 512],
    pub video_device: [c_char; 512],
    pub i2c_port: c_int,
    pub camera_model: SL_MODEL,
    pub sn: c_uint,
    pub gmsl_port: c_int,
    pub identifier: [c_uchar; 3],
    pub camera_badge: [c_char; 128],
    pub camera_sensor_model: [c_char; 128],
    pub camera_name: [c_char; 128],
    pub input_type: SL_INPUT_TYPE,
    pub sensor_address_left: c_uchar,
    pub sensor_address_right: c_uchar,
}

impl Default for SL_DeviceProperties {
    fn default() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_SensorParameters {
    pub type_: SL_SENSOR_TYPE,
    pub resolution: f32,
    pub sampling_rate: f32,
    pub range: SL_Vector2,
    pub noise_density: f32,
    pub random_walk: f32,
    pub sensor_unit: SL_SENSORS_UNIT,
    pub is_available: bool,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_CameraParameters {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub disto: [f64; 12],
    pub v_fov: f32,
    pub h_fov: f32,
    pub d_fov: f32,
    pub image_size: SL_Resolution,
    pub focal_length_metric: f32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_CalibrationParameters {
    pub left_cam: SL_CameraParameters,
    pub right_cam: SL_CameraParameters,
    pub rotation: SL_Vector4,
    pub translation: SL_Vector3,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_CameraConfiguration {
    pub calibration_parameters: SL_CalibrationParameters,
    pub calibration_parameters_raw: SL_CalibrationParameters,
    pub firmware_version: c_uint,
    pub fps: f32,
    pub resolution: SL_Resolution,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_SensorsConfiguration {
    pub firmware_version: c_uint,
    pub camera_ium_rotation: SL_Vector4,
    pub camera_imu_translation: SL_Vector3,
    pub ium_magnetometer_rotation: SL_Vector4,
    pub ium_magnetometer_translation: SL_Vector3,
    pub accelerometer_parameters: SL_SensorParameters,
    pub gyroscope_parameters: SL_SensorParameters,
    pub magnetometer_parameters: SL_SensorParameters,
    pub barometer_parameters: SL_SensorParameters,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct SL_CameraInformation {
    pub serial_number: c_uint,
    pub camera_model: SL_MODEL,
    pub input_type: SL_INPUT_TYPE,
    pub camera_configuration: SL_CameraConfiguration,
    pub sensors_configuration: SL_SensorsConfiguration,
}

unsafe extern "C" {
    pub fn sl_free(ptr: *mut c_void);
    pub fn sl_unload_all_instances();
    pub fn sl_unload_instance(camera_id: c_int);

    pub fn sl_create_camera(camera_id: c_int) -> bool;
    pub fn sl_is_opened(camera_id: c_int) -> bool;

    pub fn sl_open_camera(
        camera_id: c_int,
        init_parameters: *mut SL_InitParameters,
        serial_number: c_uint,
        path_svo: *const c_char,
        ip: *const c_char,
        stream_port: c_int,
        gmsl_port: c_int,
        output_file: *const c_char,
        opt_settings_path: *const c_char,
        opencv_calib_path: *const c_char,
    ) -> c_int;

    pub fn sl_open_camera_from_camera_id(
        camera_id: c_int,
        init_parameters: *mut SL_InitParameters,
        output_file: *const c_char,
        opt_settings_path: *const c_char,
        opencv_calib_path: *const c_char,
    ) -> c_int;

    pub fn sl_open_camera_from_serial_number(
        camera_id: c_int,
        init_parameters: *mut SL_InitParameters,
        serial_number: c_uint,
        output_file: *const c_char,
        opt_settings_path: *const c_char,
        opencv_calib_path: *const c_char,
    ) -> c_int;

    pub fn sl_open_camera_from_svo_file(
        camera_id: c_int,
        init_parameters: *mut SL_InitParameters,
        path_svo: *const c_char,
        output_file: *const c_char,
        opt_settings_path: *const c_char,
        opencv_calib_path: *const c_char,
    ) -> c_int;

    pub fn sl_open_camera_from_stream(
        camera_id: c_int,
        init_parameters: *mut SL_InitParameters,
        ip: *const c_char,
        stream_port: c_int,
        output_file: *const c_char,
        opt_settings_path: *const c_char,
        opencv_calib_path: *const c_char,
    ) -> c_int;

    pub fn sl_get_init_parameters(camera_id: c_int) -> *mut SL_InitParameters;
    pub fn sl_get_runtime_parameters(camera_id: c_int) -> *mut SL_RuntimeParameters;
    pub fn sl_close_camera(camera_id: c_int);

    pub fn sl_grab(camera_id: c_int, runtime: *mut SL_RuntimeParameters) -> c_int;
    pub fn sl_read(camera_id: c_int) -> c_int;

    pub fn sl_get_device_list(device_list: *mut SL_DeviceProperties, nb_devices: *mut c_int);
    pub fn sl_get_number_zed_connected() -> c_int;
    pub fn sl_get_sdk_version() -> *mut c_char;

    pub fn sl_get_camera_fps(camera_id: c_int) -> f32;
    pub fn sl_get_current_fps(camera_id: c_int) -> f32;
    pub fn sl_get_width(camera_id: c_int) -> c_int;
    pub fn sl_get_height(camera_id: c_int) -> c_int;

    pub fn sl_get_camera_information(
        camera_id: c_int,
        res_width: c_int,
        res_height: c_int,
    ) -> *mut SL_CameraInformation;

    pub fn sl_get_calibration_parameters(
        camera_id: c_int,
        raw_params: bool,
    ) -> *mut SL_CalibrationParameters;

    pub fn sl_get_sensors_configuration(camera_id: c_int) -> *mut SL_SensorsConfiguration;

    pub fn sl_get_camera_imu_transform(
        camera_id: c_int,
        translation: *mut SL_Vector3,
        rotation: *mut SL_Quaternion,
    );

    pub fn sl_get_input_type(camera_id: c_int) -> c_int;
    pub fn sl_get_zed_serial(camera_id: c_int) -> c_int;
    pub fn sl_get_camera_firmware(camera_id: c_int) -> c_int;
    pub fn sl_get_sensors_firmware(camera_id: c_int) -> c_int;
    pub fn sl_get_camera_model(camera_id: c_int) -> c_int;
    pub fn sl_get_image_timestamp(camera_id: c_int) -> u64;
    pub fn sl_get_current_timestamp(camera_id: c_int) -> u64;

    pub fn sl_get_sensors_data(
        camera_id: c_int,
        data: *mut SL_SensorsData,
        time_reference: SL_TIME_REFERENCE,
    ) -> c_int;

    pub fn sl_retrieve_measure(
        camera_id: c_int,
        measure_ptr: *mut c_void,
        type_: SL_MEASURE,
        mem: SL_MEM,
        width: c_int,
        height: c_int,
        custream: *mut c_void,
    ) -> c_int;

    pub fn sl_retrieve_image(
        camera_id: c_int,
        image_ptr: *mut c_void,
        type_: SL_VIEW,
        mem: SL_MEM,
        width: c_int,
        height: c_int,
        custream: *mut c_void,
    ) -> c_int;

    pub fn sl_mat_create_new(
        width: c_int,
        height: c_int,
        type_: SL_MAT_TYPE,
        mem: SL_MEM,
    ) -> *mut c_void;

    pub fn sl_mat_create_new_empty() -> *mut c_void;
    pub fn sl_mat_is_init(ptr: *mut c_void) -> bool;
    pub fn sl_mat_free(ptr: *mut c_void, mem: SL_MEM);
    pub fn sl_mat_get_infos(ptr: *mut c_void, buffer: *mut c_char);

    pub fn sl_mat_get_value_uchar(
        ptr: *mut c_void,
        col: c_int,
        row: c_int,
        value: *mut c_uchar,
        mem: SL_MEM,
    ) -> c_int;

    pub fn sl_mat_get_value_uchar2(
        ptr: *mut c_void,
        col: c_int,
        row: c_int,
        value: *mut SL_Uchar2,
        mem: SL_MEM,
    ) -> c_int;

    pub fn sl_mat_get_value_uchar3(
        ptr: *mut c_void,
        col: c_int,
        row: c_int,
        value: *mut SL_Uchar3,
        mem: SL_MEM,
    ) -> c_int;

    pub fn sl_mat_get_value_uchar4(
        ptr: *mut c_void,
        col: c_int,
        row: c_int,
        value: *mut SL_Uchar4,
        mem: SL_MEM,
    ) -> c_int;

    pub fn sl_mat_get_value_float(
        ptr: *mut c_void,
        col: c_int,
        row: c_int,
        value: *mut f32,
        mem: SL_MEM,
    ) -> c_int;

    pub fn sl_mat_get_value_float2(
        ptr: *mut c_void,
        col: c_int,
        row: c_int,
        value: *mut SL_Vector2,
        mem: SL_MEM,
    ) -> c_int;

    pub fn sl_mat_get_value_float3(
        ptr: *mut c_void,
        col: c_int,
        row: c_int,
        value: *mut SL_Vector3,
        mem: SL_MEM,
    ) -> c_int;

    pub fn sl_mat_get_value_float4(
        ptr: *mut c_void,
        col: c_int,
        row: c_int,
        value: *mut SL_Vector4,
        mem: SL_MEM,
    ) -> c_int;

    pub fn sl_mat_update_cpu_from_gpu(ptr: *mut c_void) -> c_int;
    pub fn sl_mat_update_gpu_from_cpu(ptr: *mut c_void) -> c_int;
    pub fn sl_mat_get_width(ptr: *mut c_void) -> c_int;
    pub fn sl_mat_get_height(ptr: *mut c_void) -> c_int;
    pub fn sl_mat_get_channels(ptr: *mut c_void) -> c_int;
    pub fn sl_mat_get_memory_type(ptr: *mut c_void) -> c_int;
    pub fn sl_mat_get_data_type(ptr: *mut c_void) -> c_int;
    pub fn sl_mat_get_pixel_bytes(ptr: *mut c_void) -> c_int;
    pub fn sl_mat_get_step(ptr: *mut c_void, mem: SL_MEM) -> c_int;
    pub fn sl_mat_get_step_bytes(ptr: *mut c_void, mem: SL_MEM) -> c_int;
    pub fn sl_mat_get_width_bytes(ptr: *mut c_void) -> c_int;
    pub fn sl_mat_get_resolution(ptr: *mut c_void) -> SL_Resolution;
    pub fn sl_mat_get_ptr(ptr: *mut c_void, mem: SL_MEM) -> *mut c_int;
}
