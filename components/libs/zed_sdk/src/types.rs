use std::fmt::{self, Display, Formatter};
use std::path::PathBuf;
use std::str::FromStr;
use std::time::Duration;

use crate::error::{Error, Result};
use crate::sys;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Depth reconstruction mode used by the SDK during camera open and frame grab.
pub enum DepthMode {
    /// Disable depth processing.
    None,
    /// Fastest classic stereo mode.
    Performance,
    /// Higher-quality classic stereo mode.
    Quality,
    /// Highest-quality classic stereo mode.
    Ultra,
    /// Lightweight neural depth mode.
    NeuralLight,
    /// Neural depth mode.
    Neural,
    /// Highest-quality neural depth mode.
    NeuralPlus,
}

impl DepthMode {
    pub(crate) fn as_raw(self) -> sys::SL_DEPTH_MODE {
        match self {
            Self::None => sys::SL_DEPTH_MODE::SL_DEPTH_MODE_NONE,
            Self::Performance => sys::SL_DEPTH_MODE::SL_DEPTH_MODE_PERFORMANCE,
            Self::Quality => sys::SL_DEPTH_MODE::SL_DEPTH_MODE_QUALITY,
            Self::Ultra => sys::SL_DEPTH_MODE::SL_DEPTH_MODE_ULTRA,
            Self::NeuralLight => sys::SL_DEPTH_MODE::SL_DEPTH_MODE_NEURAL_LIGHT,
            Self::Neural => sys::SL_DEPTH_MODE::SL_DEPTH_MODE_NEURAL,
            Self::NeuralPlus => sys::SL_DEPTH_MODE::SL_DEPTH_MODE_NEURAL_PLUS,
        }
    }
}

impl Default for DepthMode {
    fn default() -> Self {
        Self::Performance
    }
}

impl FromStr for DepthMode {
    type Err = &'static str;

    fn from_str(value: &str) -> std::result::Result<Self, Self::Err> {
        match value {
            "NONE" => Ok(Self::None),
            "PERFORMANCE" => Ok(Self::Performance),
            "QUALITY" => Ok(Self::Quality),
            "ULTRA" => Ok(Self::Ultra),
            "NEURAL_LIGHT" => Ok(Self::NeuralLight),
            "NEURAL" => Ok(Self::Neural),
            "NEURAL_PLUS" => Ok(Self::NeuralPlus),
            _ => Err("unsupported depth mode"),
        }
    }
}

impl Display for DepthMode {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let value = match self {
            Self::None => "NONE",
            Self::Performance => "PERFORMANCE",
            Self::Quality => "QUALITY",
            Self::Ultra => "ULTRA",
            Self::NeuralLight => "NEURAL_LIGHT",
            Self::Neural => "NEURAL",
            Self::NeuralPlus => "NEURAL_PLUS",
        };
        f.write_str(value)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Output image resolution preset requested during camera open.
pub enum ResolutionPreset {
    /// 4K HD preset.
    Hd4k,
    /// QHD+ preset.
    QhdPlus,
    /// 2K HD preset.
    Hd2k,
    /// 1536p preset.
    Hd1536,
    /// 1080p preset.
    Hd1080,
    /// 1200p preset.
    Hd1200,
    /// 720p preset.
    Hd720,
    /// SVGA preset.
    Svga,
    /// VGA preset.
    Vga,
    /// Let the SDK choose automatically.
    Auto,
}

impl ResolutionPreset {
    pub(crate) fn as_raw(self) -> sys::SL_RESOLUTION {
        match self {
            Self::Hd4k => sys::SL_RESOLUTION::SL_RESOLUTION_HD4K,
            Self::QhdPlus => sys::SL_RESOLUTION::SL_RESOLUTION_QHDPLUS,
            Self::Hd2k => sys::SL_RESOLUTION::SL_RESOLUTION_HD2K,
            Self::Hd1536 => sys::SL_RESOLUTION::SL_RESOLUTION_HD1536,
            Self::Hd1080 => sys::SL_RESOLUTION::SL_RESOLUTION_HD1080,
            Self::Hd1200 => sys::SL_RESOLUTION::SL_RESOLUTION_HD1200,
            Self::Hd720 => sys::SL_RESOLUTION::SL_RESOLUTION_HD720,
            Self::Svga => sys::SL_RESOLUTION::SL_RESOLUTION_SVGA,
            Self::Vga => sys::SL_RESOLUTION::SL_RESOLUTION_VGA,
            Self::Auto => sys::SL_RESOLUTION::SL_RESOLUTION_AUTO,
        }
    }
}

impl Default for ResolutionPreset {
    fn default() -> Self {
        Self::Hd720
    }
}

impl Display for ResolutionPreset {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let value = match self {
            Self::Hd4k => "HD4K",
            Self::QhdPlus => "QHDPLUS",
            Self::Hd2k => "HD2K",
            Self::Hd1536 => "HD1536",
            Self::Hd1080 => "HD1080",
            Self::Hd1200 => "HD1200",
            Self::Hd720 => "HD720",
            Self::Svga => "SVGA",
            Self::Vga => "VGA",
            Self::Auto => "AUTO",
        };
        f.write_str(value)
    }
}

impl FromStr for ResolutionPreset {
    type Err = &'static str;

    fn from_str(value: &str) -> std::result::Result<Self, Self::Err> {
        match value {
            "HD4K" => Ok(Self::Hd4k),
            "QHDPLUS" => Ok(Self::QhdPlus),
            "HD2K" => Ok(Self::Hd2k),
            "HD1536" => Ok(Self::Hd1536),
            "HD1080" => Ok(Self::Hd1080),
            "HD1200" => Ok(Self::Hd1200),
            "HD720" => Ok(Self::Hd720),
            "SVGA" => Ok(Self::Svga),
            "VGA" => Ok(Self::Vga),
            "AUTO" => Ok(Self::Auto),
            _ => Err("unsupported resolution preset"),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Coordinate system used for depth, point cloud, and pose outputs.
pub enum CoordinateSystem {
    /// Image-space coordinates.
    Image,
    /// Left-handed coordinates with +Y pointing up.
    LeftHandedYUp,
    /// Right-handed coordinates with +Y pointing up.
    RightHandedYUp,
    /// Right-handed coordinates with +Z pointing up.
    RightHandedZUp,
    /// Left-handed coordinates with +Z pointing up.
    LeftHandedZUp,
    /// Right-handed coordinates with +Z up and +X forward.
    RightHandedZUpXForward,
}

impl CoordinateSystem {
    pub(crate) fn as_raw(self) -> sys::SL_COORDINATE_SYSTEM {
        match self {
            Self::Image => sys::SL_COORDINATE_SYSTEM::SL_COORDINATE_SYSTEM_IMAGE,
            Self::LeftHandedYUp => sys::SL_COORDINATE_SYSTEM::SL_COORDINATE_SYSTEM_LEFT_HANDED_Y_UP,
            Self::RightHandedYUp => {
                sys::SL_COORDINATE_SYSTEM::SL_COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP
            }
            Self::RightHandedZUp => {
                sys::SL_COORDINATE_SYSTEM::SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP
            }
            Self::LeftHandedZUp => sys::SL_COORDINATE_SYSTEM::SL_COORDINATE_SYSTEM_LEFT_HANDED_Z_UP,
            Self::RightHandedZUpXForward => {
                sys::SL_COORDINATE_SYSTEM::SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD
            }
        }
    }
}

impl Default for CoordinateSystem {
    fn default() -> Self {
        Self::LeftHandedYUp
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Unit used for SDK distance values.
pub enum Unit {
    /// Millimeters.
    Millimeter,
    /// Centimeters.
    Centimeter,
    /// Meters.
    Meter,
    /// Inches.
    Inch,
    /// Feet.
    Foot,
}

impl Unit {
    pub(crate) fn as_raw(self) -> sys::SL_UNIT {
        match self {
            Self::Millimeter => sys::SL_UNIT::SL_UNIT_MILLIMETER,
            Self::Centimeter => sys::SL_UNIT::SL_UNIT_CENTIMETER,
            Self::Meter => sys::SL_UNIT::SL_UNIT_METER,
            Self::Inch => sys::SL_UNIT::SL_UNIT_INCH,
            Self::Foot => sys::SL_UNIT::SL_UNIT_FOOT,
        }
    }
}

impl Default for Unit {
    fn default() -> Self {
        Self::Meter
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Reference frame used when retrieving runtime measurements.
pub enum ReferenceFrame {
    /// Measurements are expressed in the world frame.
    World,
    /// Measurements are expressed in the current camera frame.
    Camera,
}

impl ReferenceFrame {
    pub(crate) fn as_raw(self) -> sys::SL_REFERENCE_FRAME {
        match self {
            Self::World => sys::SL_REFERENCE_FRAME::SL_REFERENCE_FRAME_WORLD,
            Self::Camera => sys::SL_REFERENCE_FRAME::SL_REFERENCE_FRAME_CAMERA,
        }
    }
}

impl Default for ReferenceFrame {
    fn default() -> Self {
        Self::Camera
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Memory placement for matrix allocations.
pub enum MemoryType {
    /// CPU-visible memory only.
    Cpu,
    /// GPU memory only.
    Gpu,
    /// Memory accessible from both CPU and GPU.
    Both,
}

impl MemoryType {
    pub(crate) fn as_raw(self) -> sys::SL_MEM {
        match self {
            Self::Cpu => sys::SL_MEM::SL_MEM_CPU,
            Self::Gpu => sys::SL_MEM::SL_MEM_GPU,
            Self::Both => sys::SL_MEM::SL_MEM_BOTH,
        }
    }

    pub(crate) fn from_raw(raw: i32) -> Option<Self> {
        match raw {
            0 => Some(Self::Cpu),
            1 => Some(Self::Gpu),
            2 => Some(Self::Both),
            _ => None,
        }
    }

    pub(crate) fn exposes_cpu(self) -> bool {
        matches!(self, Self::Cpu | Self::Both)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// High-level category of input selected for camera open.
pub enum InputType {
    Usb,
    Svo,
    Stream,
    Gmsl,
}

impl InputType {
    pub(crate) fn from_raw(raw: sys::SL_INPUT_TYPE) -> Self {
        match raw {
            sys::SL_INPUT_TYPE::SL_INPUT_TYPE_USB => Self::Usb,
            sys::SL_INPUT_TYPE::SL_INPUT_TYPE_SVO => Self::Svo,
            sys::SL_INPUT_TYPE::SL_INPUT_TYPE_STREAM => Self::Stream,
            sys::SL_INPUT_TYPE::SL_INPUT_TYPE_GMSL => Self::Gmsl,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
/// Concrete source used to open the SDK.
pub enum InputSource {
    /// Local USB camera selected by device id.
    Usb { device_id: i32 },
    /// Local camera selected by serial number.
    SerialNumber(u32),
    /// Recorded SVO file on disk.
    SvoFile(PathBuf),
    /// Network stream source.
    Stream { ip: String, port: i32 },
    /// GMSL camera selected by serial number and port.
    Gmsl { serial_number: u32, port: i32 },
}

impl InputSource {
    pub(crate) fn input_type(&self) -> InputType {
        match self {
            Self::Usb { .. } | Self::SerialNumber(_) => InputType::Usb,
            Self::SvoFile(_) => InputType::Svo,
            Self::Stream { .. } => InputType::Stream,
            Self::Gmsl { .. } => InputType::Gmsl,
        }
    }

    pub(crate) fn camera_device_id(&self) -> i32 {
        match self {
            Self::Usb { device_id } => *device_id,
            _ => 0,
        }
    }
}

impl Default for InputSource {
    fn default() -> Self {
        Self::Usb { device_id: 0 }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Concrete width and height pair.
pub struct Resolution {
    width: u32,
    height: u32,
}

impl Resolution {
    /// Creates a resolution from width and height in pixels.
    pub const fn new(width: u32, height: u32) -> Self {
        Self { width, height }
    }

    /// Returns the width in pixels.
    pub const fn width(self) -> u32 {
        self.width
    }

    /// Returns the height in pixels.
    pub const fn height(self) -> u32 {
        self.height
    }

    pub(crate) fn width_i32(self) -> Result<i32> {
        i32::try_from(self.width).map_err(|_| Error::DimensionOutOfRange {
            field: "width",
            value: self.width,
        })
    }

    pub(crate) fn height_i32(self) -> Result<i32> {
        i32::try_from(self.height).map_err(|_| Error::DimensionOutOfRange {
            field: "height",
            value: self.height,
        })
    }

    pub(crate) fn try_from_raw(width: i32, height: i32, context: &'static str) -> Result<Self> {
        if width <= 0 || height <= 0 {
            return Err(Error::InvalidResolution {
                context,
                width,
                height,
            });
        }

        Ok(Self::new(width as u32, height as u32))
    }
}

#[derive(Clone, Debug)]
/// Builder-style camera open options.
///
/// Start from [`OpenOptions::default`] and adjust only the fields you care about.
pub struct OpenOptions {
    pub(crate) source: InputSource,
    pub(crate) resolution: ResolutionPreset,
    pub(crate) fps: i32,
    pub(crate) depth_mode: DepthMode,
    pub(crate) enable_right_side_measure: bool,
    pub(crate) depth_stabilization: i32,
    pub(crate) depth_minimum_distance_m: Option<f32>,
    pub(crate) depth_maximum_distance_m: f32,
    pub(crate) coordinate_unit: Unit,
    pub(crate) coordinate_system: CoordinateSystem,
    pub(crate) sdk_gpu_id: i32,
    pub(crate) sdk_verbose: i32,
    pub(crate) sensors_required: bool,
    pub(crate) enable_image_enhancement: bool,
    pub(crate) open_timeout: Duration,
    pub(crate) async_grab_camera_recovery: bool,
    pub(crate) grab_compute_capping_fps: Option<f32>,
    pub(crate) enable_image_validity_check: bool,
    pub(crate) output_log_file: Option<PathBuf>,
    pub(crate) settings_path: Option<PathBuf>,
    pub(crate) opencv_calibration_path: Option<PathBuf>,
}

impl OpenOptions {
    /// Replaces the input source.
    pub fn source(mut self, source: InputSource) -> Self {
        self.source = source;
        self
    }

    /// Selects a USB camera by device id.
    pub fn camera_device_id(mut self, device_id: i32) -> Self {
        self.source = InputSource::Usb { device_id };
        self
    }

    /// Sets the requested output resolution preset.
    pub fn resolution(mut self, resolution: ResolutionPreset) -> Self {
        self.resolution = resolution;
        self
    }

    /// Sets the requested frame rate.
    pub fn fps(mut self, fps: i32) -> Self {
        self.fps = fps;
        self
    }

    /// Sets the depth reconstruction mode.
    pub fn depth_mode(mut self, depth_mode: DepthMode) -> Self {
        self.depth_mode = depth_mode;
        self
    }

    /// Sets the unit used for distances returned by the SDK.
    pub fn coordinate_unit(mut self, coordinate_unit: Unit) -> Self {
        self.coordinate_unit = coordinate_unit;
        self
    }

    /// Sets the coordinate system used for depth and point cloud outputs.
    pub fn coordinate_system(mut self, coordinate_system: CoordinateSystem) -> Self {
        self.coordinate_system = coordinate_system;
        self
    }

    /// Sets the minimum and maximum depth range in meters.
    pub fn depth_range_m(mut self, min: Option<f32>, max: f32) -> Self {
        self.depth_minimum_distance_m = min;
        self.depth_maximum_distance_m = max;
        self
    }

    /// Sets the open timeout used by the SDK.
    pub fn open_timeout(mut self, open_timeout: Duration) -> Self {
        self.open_timeout = open_timeout;
        self
    }

    /// Enables retrieval of the right-side depth measure when supported.
    pub fn enable_right_side_measure(mut self, enable_right_side_measure: bool) -> Self {
        self.enable_right_side_measure = enable_right_side_measure;
        self
    }

    /// Requires motion sensors to be available during open.
    pub fn sensors_required(mut self, sensors_required: bool) -> Self {
        self.sensors_required = sensors_required;
        self
    }

    /// Sets the SDK verbosity level.
    pub fn sdk_verbose(mut self, sdk_verbose: i32) -> Self {
        self.sdk_verbose = sdk_verbose;
        self
    }

    /// Writes the SDK log output to a file.
    pub fn output_log_file(mut self, output_log_file: impl Into<PathBuf>) -> Self {
        self.output_log_file = Some(output_log_file.into());
        self
    }

    /// Uses an alternate SDK settings directory or file path.
    pub fn settings_path(mut self, settings_path: impl Into<PathBuf>) -> Self {
        self.settings_path = Some(settings_path.into());
        self
    }

    /// Uses an alternate OpenCV calibration file path.
    pub fn opencv_calibration_path(mut self, opencv_calibration_path: impl Into<PathBuf>) -> Self {
        self.opencv_calibration_path = Some(opencv_calibration_path.into());
        self
    }

    /// Returns the configured source.
    pub fn source_ref(&self) -> &InputSource {
        &self.source
    }

    /// Returns the configured resolution preset.
    pub fn resolution_preset(&self) -> ResolutionPreset {
        self.resolution
    }

    /// Returns the configured frame rate.
    pub fn fps_value(&self) -> i32 {
        self.fps
    }

    /// Returns the configured depth mode.
    pub fn depth_mode_value(&self) -> DepthMode {
        self.depth_mode
    }

    /// Returns the configured open timeout.
    pub fn open_timeout_value(&self) -> Duration {
        self.open_timeout
    }

    /// Returns the configured maximum depth distance in meters.
    pub fn depth_maximum_distance_m_value(&self) -> f32 {
        self.depth_maximum_distance_m
    }

    pub(crate) fn to_raw(&self) -> sys::SL_InitParameters {
        sys::SL_InitParameters {
            input_type: match self.source.input_type() {
                InputType::Usb => sys::SL_INPUT_TYPE::SL_INPUT_TYPE_USB,
                InputType::Svo => sys::SL_INPUT_TYPE::SL_INPUT_TYPE_SVO,
                InputType::Stream => sys::SL_INPUT_TYPE::SL_INPUT_TYPE_STREAM,
                InputType::Gmsl => sys::SL_INPUT_TYPE::SL_INPUT_TYPE_GMSL,
            },
            resolution: self.resolution.as_raw(),
            camera_fps: self.fps,
            camera_device_id: self.source.camera_device_id(),
            camera_image_flip: sys::SL_FLIP_MODE::SL_FLIP_MODE_AUTO,
            camera_disable_self_calib: false,
            enable_right_side_measure: self.enable_right_side_measure,
            svo_real_time_mode: false,
            depth_mode: self.depth_mode.as_raw(),
            depth_stabilization: self.depth_stabilization,
            depth_minimum_distance: self.depth_minimum_distance_m.unwrap_or(-1.0),
            depth_maximum_distance: self.depth_maximum_distance_m,
            coordinate_unit: self.coordinate_unit.as_raw(),
            coordinate_system: self.coordinate_system.as_raw(),
            sdk_gpu_id: self.sdk_gpu_id,
            sdk_verbose: self.sdk_verbose,
            sensors_required: self.sensors_required,
            enable_image_enhancement: self.enable_image_enhancement,
            open_timeout_sec: self.open_timeout.as_secs_f32(),
            async_grab_camera_recovery: self.async_grab_camera_recovery,
            grab_compute_capping_fps: self.grab_compute_capping_fps.unwrap_or(0.0),
            enable_image_validity_check: self.enable_image_validity_check,
            maximum_working_resolution: Default::default(),
        }
    }
}

impl Default for OpenOptions {
    fn default() -> Self {
        Self {
            source: InputSource::default(),
            resolution: ResolutionPreset::default(),
            fps: 30,
            depth_mode: DepthMode::default(),
            enable_right_side_measure: false,
            depth_stabilization: 30,
            depth_minimum_distance_m: None,
            depth_maximum_distance_m: 40.0,
            coordinate_unit: Unit::default(),
            coordinate_system: CoordinateSystem::default(),
            sdk_gpu_id: -1,
            sdk_verbose: 0,
            sensors_required: false,
            enable_image_enhancement: true,
            open_timeout: Duration::from_secs(20),
            async_grab_camera_recovery: false,
            grab_compute_capping_fps: None,
            enable_image_validity_check: false,
            output_log_file: None,
            settings_path: None,
            opencv_calibration_path: None,
        }
    }
}

#[derive(Clone, Copy, Debug)]
/// Per-grab runtime tuning options.
pub struct RuntimeParameters {
    pub(crate) reference_frame: ReferenceFrame,
    pub(crate) enable_depth: bool,
    pub(crate) enable_fill_mode: bool,
    pub(crate) confidence_threshold: i32,
    pub(crate) texture_confidence_threshold: i32,
    pub(crate) remove_saturated_areas: bool,
}

impl RuntimeParameters {
    /// Sets the reference frame for runtime measurements.
    pub fn reference_frame(mut self, reference_frame: ReferenceFrame) -> Self {
        self.reference_frame = reference_frame;
        self
    }

    /// Enables or disables depth computation during `grab`.
    pub fn enable_depth(mut self, enable_depth: bool) -> Self {
        self.enable_depth = enable_depth;
        self
    }

    /// Enables hole filling in depth results.
    pub fn enable_fill_mode(mut self, enable_fill_mode: bool) -> Self {
        self.enable_fill_mode = enable_fill_mode;
        self
    }

    /// Sets the depth confidence threshold.
    pub fn confidence_threshold(mut self, confidence_threshold: i32) -> Self {
        self.confidence_threshold = confidence_threshold;
        self
    }

    /// Sets the texture confidence threshold.
    pub fn texture_confidence_threshold(mut self, texture_confidence_threshold: i32) -> Self {
        self.texture_confidence_threshold = texture_confidence_threshold;
        self
    }

    pub(crate) fn to_raw(self) -> sys::SL_RuntimeParameters {
        sys::SL_RuntimeParameters {
            reference_frame: self.reference_frame.as_raw(),
            enable_depth: self.enable_depth,
            enable_fill_mode: self.enable_fill_mode,
            confidence_threshold: self.confidence_threshold,
            texture_confidence_threshold: self.texture_confidence_threshold,
            remove_saturated_areas: self.remove_saturated_areas,
        }
    }
}

impl Default for RuntimeParameters {
    fn default() -> Self {
        Self {
            reference_frame: ReferenceFrame::default(),
            enable_depth: true,
            enable_fill_mode: false,
            confidence_threshold: 95,
            texture_confidence_threshold: 100,
            remove_saturated_areas: true,
        }
    }
}

#[derive(Clone, Copy, Debug)]
/// High-level metadata for an opened camera or playback source.
pub struct CameraInformation {
    pub serial_number: u32,
    pub model: sys::SL_MODEL,
    pub input_type: InputType,
    pub resolution: Resolution,
    pub fps: f32,
}

impl CameraInformation {
    pub(crate) fn from_raw(raw: sys::SL_CameraInformation) -> Result<Self> {
        Ok(Self {
            serial_number: raw.serial_number,
            model: raw.camera_model,
            input_type: InputType::from_raw(raw.input_type),
            resolution: Resolution::try_from_raw(
                raw.camera_configuration.resolution.width,
                raw.camera_configuration.resolution.height,
                "camera information",
            )?,
            fps: raw.camera_configuration.fps,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// Intrinsic parameters for one camera eye.
pub struct CameraParameters {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub disto: [f64; 12],
    pub v_fov: f32,
    pub h_fov: f32,
    pub d_fov: f32,
    pub image_size: Resolution,
    pub focal_length_metric: f32,
}

impl CameraParameters {
    pub(crate) fn from_raw(raw: sys::SL_CameraParameters) -> Result<Self> {
        Ok(Self {
            fx: raw.fx,
            fy: raw.fy,
            cx: raw.cx,
            cy: raw.cy,
            disto: raw.disto,
            v_fov: raw.v_fov,
            h_fov: raw.h_fov,
            d_fov: raw.d_fov,
            image_size: Resolution::try_from_raw(
                raw.image_size.width,
                raw.image_size.height,
                "camera parameters",
            )?,
            focal_length_metric: raw.focal_length_metric,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// Stereo calibration for the left and right cameras.
pub struct CalibrationParameters {
    pub left_cam: CameraParameters,
    pub right_cam: CameraParameters,
    pub rotation_rodrigues: [f32; 3],
    pub rotation_w: f32,
    pub translation: Vec3f,
}

impl CalibrationParameters {
    pub(crate) fn from_raw(raw: sys::SL_CalibrationParameters) -> Result<Self> {
        Ok(Self {
            left_cam: CameraParameters::from_raw(raw.left_cam)?,
            right_cam: CameraParameters::from_raw(raw.right_cam)?,
            rotation_rodrigues: [raw.rotation.x, raw.rotation.y, raw.rotation.z],
            rotation_w: raw.rotation.w,
            translation: vec3_from_raw(raw.translation),
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// Capability and noise data for one physical sensor.
pub struct SensorParameters {
    pub resolution: f32,
    pub sampling_rate: f32,
    pub range: Vec2f,
    pub noise_density: f32,
    pub random_walk: f32,
    pub is_available: bool,
}

impl SensorParameters {
    pub(crate) fn from_raw(raw: sys::SL_SensorParameters) -> Self {
        Self {
            resolution: raw.resolution,
            sampling_rate: raw.sampling_rate,
            range: Vec2f {
                x: raw.range.x,
                y: raw.range.y,
            },
            noise_density: raw.noise_density,
            random_walk: raw.random_walk,
            is_available: raw.is_available,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// Static configuration for the motion and environmental sensors.
pub struct SensorsConfiguration {
    pub camera_imu_rotation: Vec4f,
    pub camera_imu_translation: Vec3f,
    pub imu_magnetometer_rotation: Vec4f,
    pub imu_magnetometer_translation: Vec3f,
    pub accelerometer: SensorParameters,
    pub gyroscope: SensorParameters,
    pub magnetometer: SensorParameters,
    pub barometer: SensorParameters,
}

impl SensorsConfiguration {
    pub(crate) fn from_raw(raw: sys::SL_SensorsConfiguration) -> Self {
        Self {
            camera_imu_rotation: vec4_from_raw(raw.camera_ium_rotation),
            camera_imu_translation: vec3_from_raw(raw.camera_imu_translation),
            imu_magnetometer_rotation: vec4_from_raw(raw.ium_magnetometer_rotation),
            imu_magnetometer_translation: vec3_from_raw(raw.ium_magnetometer_translation),
            accelerometer: SensorParameters::from_raw(raw.accelerometer_parameters),
            gyroscope: SensorParameters::from_raw(raw.gyroscope_parameters),
            magnetometer: SensorParameters::from_raw(raw.magnetometer_parameters),
            barometer: SensorParameters::from_raw(raw.barometer_parameters),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// Camera-to-IMU extrinsic transform.
pub struct CameraImuTransform {
    pub translation: Vec3f,
    pub rotation_xyzw: [f32; 4],
}

impl CameraImuTransform {
    pub(crate) fn from_raw(translation: sys::SL_Vector3, rotation: sys::SL_Quaternion) -> Self {
        Self {
            translation: vec3_from_raw(translation),
            rotation_xyzw: [rotation.x, rotation.y, rotation.z, rotation.w],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// IMU sample synchronized to the image timeline.
pub struct ImuData {
    pub timestamp_ns: u64,
    pub angular_velocity: Vec3f,
    pub linear_acceleration: Vec3f,
    pub angular_velocity_unc: Vec3f,
    pub linear_acceleration_unc: Vec3f,
    pub orientation_xyzw: [f32; 4],
    pub orientation_covariance: [f32; 9],
    pub angular_velocity_covariance: [f32; 9],
    pub linear_acceleration_covariance: [f32; 9],
}

impl ImuData {
    pub(crate) fn from_raw(raw: sys::SL_IMUData) -> Option<Self> {
        raw.is_available.then_some(Self {
            timestamp_ns: raw.timestamp_ns,
            angular_velocity: vec3_from_raw(raw.angular_velocity),
            linear_acceleration: vec3_from_raw(raw.linear_acceleration),
            angular_velocity_unc: vec3_from_raw(raw.angular_velocity_unc),
            linear_acceleration_unc: vec3_from_raw(raw.linear_acceleration_unc),
            orientation_xyzw: [
                raw.orientation.x,
                raw.orientation.y,
                raw.orientation.z,
                raw.orientation.w,
            ],
            orientation_covariance: raw.orientation_covariance.p,
            angular_velocity_covariance: raw.angular_velocity_convariance.p,
            linear_acceleration_covariance: raw.linear_acceleration_convariance.p,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// Barometer sample synchronized to the image timeline.
pub struct BarometerData {
    pub timestamp_ns: u64,
    pub pressure_pa: f32,
    pub relative_altitude_m: f32,
}

impl BarometerData {
    pub(crate) fn from_raw(raw: sys::SL_BarometerData) -> Option<Self> {
        raw.is_available.then_some(Self {
            timestamp_ns: raw.timestamp_ns,
            pressure_pa: raw.pressure,
            relative_altitude_m: raw.relative_altitude,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// Magnetometer sample synchronized to the image timeline.
pub struct MagnetometerData {
    pub timestamp_ns: u64,
    pub magnetic_field_ut: Vec3f,
    pub magnetic_field_unc_ut: Vec3f,
    pub magnetic_heading: f32,
    pub magnetic_heading_state: u32,
    pub magnetic_heading_accuracy: f32,
    pub effective_rate_hz: f32,
}

impl MagnetometerData {
    pub(crate) fn from_raw(raw: sys::SL_MagnetometerData) -> Option<Self> {
        raw.is_available.then_some(Self {
            timestamp_ns: raw.timestamp_ns,
            magnetic_field_ut: vec3_from_raw(raw.magnetic_field_c),
            magnetic_field_unc_ut: vec3_from_raw(raw.magnetic_field_unc),
            magnetic_heading: raw.magnetic_heading,
            magnetic_heading_state: raw.magnetic_heading_state,
            magnetic_heading_accuracy: raw.magnetic_heading_accuracy,
            effective_rate_hz: raw.effective_rate,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// Temperature readings reported by the camera module.
pub struct TemperatureData {
    pub imu_temp_c: f32,
    pub barometer_temp_c: f32,
    pub onboard_left_temp_c: f32,
    pub onboard_right_temp_c: f32,
}

impl TemperatureData {
    pub(crate) fn from_raw(raw: sys::SL_TemperatureData) -> Self {
        Self {
            imu_temp_c: raw.imu_temp,
            barometer_temp_c: raw.barometer_temp,
            onboard_left_temp_c: raw.onboard_left_temp,
            onboard_right_temp_c: raw.onboard_right_temp,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// Collection of sensor samples returned by the SDK for one image time reference.
pub struct SensorsData {
    pub imu: Option<ImuData>,
    pub barometer: Option<BarometerData>,
    pub magnetometer: Option<MagnetometerData>,
    pub temperature: TemperatureData,
    pub camera_moving_state: i32,
    pub image_sync_trigger: i32,
}

impl SensorsData {
    pub(crate) fn from_raw(raw: sys::SL_SensorsData) -> Self {
        Self {
            imu: ImuData::from_raw(raw.imu),
            barometer: BarometerData::from_raw(raw.barometer),
            magnetometer: MagnetometerData::from_raw(raw.magnetometer),
            temperature: TemperatureData::from_raw(raw.temperature),
            camera_moving_state: raw.camera_moving_state,
            image_sync_trigger: raw.image_sync_trigger,
        }
    }
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, Eq, PartialEq)]
/// Packed BGR pixel.
pub struct Bgr8 {
    pub b: u8,
    pub g: u8,
    pub r: u8,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, Eq, PartialEq)]
/// Packed BGRA pixel.
pub struct Bgra8 {
    pub b: u8,
    pub g: u8,
    pub r: u8,
    pub a: u8,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, Eq, PartialEq)]
/// Packed RGBA pixel.
pub struct Rgba8 {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, PartialEq)]
/// Two-lane `f32` vector.
pub struct Vec2f {
    pub x: f32,
    pub y: f32,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, PartialEq)]
/// Three-lane `f32` vector.
pub struct Vec3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, PartialEq)]
/// Four-lane `f32` vector.
pub struct Vec4f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, PartialEq)]
/// Packed XYZ point with an SDK-encoded RGBA payload.
pub struct Point3Color {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    rgba: f32,
}

impl Point3Color {
    /// Returns the XYZ position as a plain array.
    pub fn position(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    /// Returns the packed RGBA value as raw bits.
    pub fn rgba_bits(self) -> u32 {
        self.rgba.to_bits()
    }

    /// Returns the packed RGBA value as native-endian bytes.
    pub fn rgba_bytes(self) -> [u8; 4] {
        self.rgba_bits().to_ne_bytes()
    }

    /// Returns `true` when all XYZ coordinates are finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

fn vec3_from_raw(raw: sys::SL_Vector3) -> Vec3f {
    Vec3f {
        x: raw.x,
        y: raw.y,
        z: raw.z,
    }
}

fn vec4_from_raw(raw: sys::SL_Vector4) -> Vec4f {
    Vec4f {
        x: raw.x,
        y: raw.y,
        z: raw.z,
        w: raw.w,
    }
}
