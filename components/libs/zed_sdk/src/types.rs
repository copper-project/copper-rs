use std::fmt::{self, Display, Formatter};
use std::path::PathBuf;
use std::str::FromStr;
use std::time::Duration;

use crate::error::{Error, Result};
use crate::sys;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DepthMode {
    None,
    Performance,
    Quality,
    Ultra,
    NeuralLight,
    Neural,
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
pub enum ResolutionPreset {
    Hd4k,
    QhdPlus,
    Hd2k,
    Hd1536,
    Hd1080,
    Hd1200,
    Hd720,
    Svga,
    Vga,
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoordinateSystem {
    Image,
    LeftHandedYUp,
    RightHandedYUp,
    RightHandedZUp,
    LeftHandedZUp,
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
pub enum Unit {
    Millimeter,
    Centimeter,
    Meter,
    Inch,
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
pub enum ReferenceFrame {
    World,
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
pub enum MemoryType {
    Cpu,
    Gpu,
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
pub enum InputSource {
    Usb { device_id: i32 },
    SerialNumber(u32),
    SvoFile(PathBuf),
    Stream { ip: String, port: i32 },
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
pub struct Resolution {
    width: u32,
    height: u32,
}

impl Resolution {
    pub const fn new(width: u32, height: u32) -> Self {
        Self { width, height }
    }

    pub const fn width(self) -> u32 {
        self.width
    }

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
    pub fn source(mut self, source: InputSource) -> Self {
        self.source = source;
        self
    }

    pub fn camera_device_id(mut self, device_id: i32) -> Self {
        self.source = InputSource::Usb { device_id };
        self
    }

    pub fn resolution(mut self, resolution: ResolutionPreset) -> Self {
        self.resolution = resolution;
        self
    }

    pub fn fps(mut self, fps: i32) -> Self {
        self.fps = fps;
        self
    }

    pub fn depth_mode(mut self, depth_mode: DepthMode) -> Self {
        self.depth_mode = depth_mode;
        self
    }

    pub fn coordinate_unit(mut self, coordinate_unit: Unit) -> Self {
        self.coordinate_unit = coordinate_unit;
        self
    }

    pub fn coordinate_system(mut self, coordinate_system: CoordinateSystem) -> Self {
        self.coordinate_system = coordinate_system;
        self
    }

    pub fn depth_range_m(mut self, min: Option<f32>, max: f32) -> Self {
        self.depth_minimum_distance_m = min;
        self.depth_maximum_distance_m = max;
        self
    }

    pub fn open_timeout(mut self, open_timeout: Duration) -> Self {
        self.open_timeout = open_timeout;
        self
    }

    pub fn enable_right_side_measure(mut self, enable_right_side_measure: bool) -> Self {
        self.enable_right_side_measure = enable_right_side_measure;
        self
    }

    pub fn sensors_required(mut self, sensors_required: bool) -> Self {
        self.sensors_required = sensors_required;
        self
    }

    pub fn sdk_verbose(mut self, sdk_verbose: i32) -> Self {
        self.sdk_verbose = sdk_verbose;
        self
    }

    pub fn output_log_file(mut self, output_log_file: impl Into<PathBuf>) -> Self {
        self.output_log_file = Some(output_log_file.into());
        self
    }

    pub fn settings_path(mut self, settings_path: impl Into<PathBuf>) -> Self {
        self.settings_path = Some(settings_path.into());
        self
    }

    pub fn opencv_calibration_path(mut self, opencv_calibration_path: impl Into<PathBuf>) -> Self {
        self.opencv_calibration_path = Some(opencv_calibration_path.into());
        self
    }

    pub fn source_ref(&self) -> &InputSource {
        &self.source
    }

    pub fn resolution_preset(&self) -> ResolutionPreset {
        self.resolution
    }

    pub fn fps_value(&self) -> i32 {
        self.fps
    }

    pub fn depth_mode_value(&self) -> DepthMode {
        self.depth_mode
    }

    pub fn open_timeout_value(&self) -> Duration {
        self.open_timeout
    }

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
pub struct RuntimeParameters {
    pub(crate) reference_frame: ReferenceFrame,
    pub(crate) enable_depth: bool,
    pub(crate) enable_fill_mode: bool,
    pub(crate) confidence_threshold: i32,
    pub(crate) texture_confidence_threshold: i32,
    pub(crate) remove_saturated_areas: bool,
}

impl RuntimeParameters {
    pub fn reference_frame(mut self, reference_frame: ReferenceFrame) -> Self {
        self.reference_frame = reference_frame;
        self
    }

    pub fn enable_depth(mut self, enable_depth: bool) -> Self {
        self.enable_depth = enable_depth;
        self
    }

    pub fn enable_fill_mode(mut self, enable_fill_mode: bool) -> Self {
        self.enable_fill_mode = enable_fill_mode;
        self
    }

    pub fn confidence_threshold(mut self, confidence_threshold: i32) -> Self {
        self.confidence_threshold = confidence_threshold;
        self
    }

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

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, Eq, PartialEq)]
pub struct Bgr8 {
    pub b: u8,
    pub g: u8,
    pub r: u8,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, Eq, PartialEq)]
pub struct Bgra8 {
    pub b: u8,
    pub g: u8,
    pub r: u8,
    pub a: u8,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, Eq, PartialEq)]
pub struct Rgba8 {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Vec2f {
    pub x: f32,
    pub y: f32,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Vec3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Vec4f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

#[repr(C)]
#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Point3Color {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    rgba: f32,
}

impl Point3Color {
    pub fn position(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    pub fn rgba_bits(self) -> u32 {
        self.rgba.to_bits()
    }

    pub fn rgba_bytes(self) -> [u8; 4] {
        self.rgba_bits().to_ne_bytes()
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}
