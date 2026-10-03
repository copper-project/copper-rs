#![deny(unsafe_op_in_unsafe_fn)]
#![doc = include_str!("../README.md")]

#[cfg(not(target_os = "linux"))]
compile_error!("zed-sdk currently supports Linux targets only");

mod camera;
mod error;
mod mat;
mod types;

pub use camera::Camera;
pub use error::{Error, ErrorCode, Result};
pub use mat::{Mat, MatElement, MatType, MatView, MatViewMut};
pub use types::{
    BarometerData, Bgr8, Bgra8, CalibrationParameters, CameraImuTransform, CameraInformation,
    CameraParameters, CoordinateSystem, DepthMode, ImuData, InputSource, InputType,
    MagnetometerData, MemoryType, OpenOptions, Point3Color, ReferenceFrame, Resolution,
    ResolutionPreset, Rgba8, RuntimeParameters, SensorParameters, SensorsConfiguration,
    SensorsData, TemperatureData, Unit, Vec2f, Vec3f, Vec4f,
};
pub use zed_sdk_sys as sys;

/// Reports whether a linkable native `sl_zed_c` wrapper was found or built at compile time.
///
/// When this is `false`, the Rust crate can still compile, but binaries that exercise the SDK
/// will need a native wrapper available at link/runtime.
pub const HAS_NATIVE_ZED_WRAPPER: bool = sys::HAS_NATIVE_ZED_WRAPPER;
