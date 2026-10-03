use std::error::Error as StdError;
use std::fmt::{self, Display, Formatter};

use crate::{MatType, MemoryType};

/// Crate-wide result type returned by safe ZED SDK operations.
pub type Result<T> = std::result::Result<T, Error>;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// SDK status codes returned by the native C API.
pub enum ErrorCode {
    SensorConfigurationChanged,
    PotentialCalibrationIssue,
    ConfigurationFallback,
    SensorsDataRequired,
    CorruptedFrame,
    CameraRebooting,
    Success,
    Failure,
    NoGpuCompatible,
    NotEnoughGpuMemory,
    CameraNotDetected,
    SensorsNotInitialized,
    SensorsNotAvailable,
    InvalidResolution,
    LowUsbBandwidth,
    CalibrationFileNotAvailable,
    InvalidCalibrationFile,
    InvalidSvoFile,
    SvoRecordingError,
    SvoUnsupportedCompression,
    EndOfSvoFileReached,
    InvalidCoordinateSystem,
    InvalidFirmware,
    InvalidFunctionParameters,
    CudaError,
    CameraNotInitialized,
    NvidiaDriverOutOfDate,
    InvalidFunctionCall,
    CorruptedSdkInstallation,
    IncompatibleSdkVersion,
    InvalidAreaFile,
    IncompatibleAreaFile,
    CameraFailedToSetup,
    CameraDetectionIssue,
    CannotStartCameraStream,
    NoGpuDetected,
    PlaneNotFound,
    ModuleNotCompatibleWithCamera,
    MotionSensorsRequired,
    ModuleNotCompatibleWithCudaVersion,
    DriverFailure,
    Unknown(i32),
}

impl ErrorCode {
    /// Converts a raw SDK status code into the typed Rust enum.
    pub fn from_raw(raw: i32) -> Self {
        match raw {
            -6 => Self::SensorConfigurationChanged,
            -5 => Self::PotentialCalibrationIssue,
            -4 => Self::ConfigurationFallback,
            -3 => Self::SensorsDataRequired,
            -2 => Self::CorruptedFrame,
            -1 => Self::CameraRebooting,
            0 => Self::Success,
            1 => Self::Failure,
            2 => Self::NoGpuCompatible,
            3 => Self::NotEnoughGpuMemory,
            4 => Self::CameraNotDetected,
            5 => Self::SensorsNotInitialized,
            6 => Self::SensorsNotAvailable,
            7 => Self::InvalidResolution,
            8 => Self::LowUsbBandwidth,
            9 => Self::CalibrationFileNotAvailable,
            10 => Self::InvalidCalibrationFile,
            11 => Self::InvalidSvoFile,
            12 => Self::SvoRecordingError,
            13 => Self::SvoUnsupportedCompression,
            14 => Self::EndOfSvoFileReached,
            15 => Self::InvalidCoordinateSystem,
            16 => Self::InvalidFirmware,
            17 => Self::InvalidFunctionParameters,
            18 => Self::CudaError,
            19 => Self::CameraNotInitialized,
            20 => Self::NvidiaDriverOutOfDate,
            21 => Self::InvalidFunctionCall,
            22 => Self::CorruptedSdkInstallation,
            23 => Self::IncompatibleSdkVersion,
            24 => Self::InvalidAreaFile,
            25 => Self::IncompatibleAreaFile,
            26 => Self::CameraFailedToSetup,
            27 => Self::CameraDetectionIssue,
            28 => Self::CannotStartCameraStream,
            29 => Self::NoGpuDetected,
            30 => Self::PlaneNotFound,
            31 => Self::ModuleNotCompatibleWithCamera,
            32 => Self::MotionSensorsRequired,
            33 => Self::ModuleNotCompatibleWithCudaVersion,
            34 => Self::DriverFailure,
            _ => Self::Unknown(raw),
        }
    }

    /// Returns the original numeric SDK status code.
    pub fn raw(self) -> i32 {
        match self {
            Self::SensorConfigurationChanged => -6,
            Self::PotentialCalibrationIssue => -5,
            Self::ConfigurationFallback => -4,
            Self::SensorsDataRequired => -3,
            Self::CorruptedFrame => -2,
            Self::CameraRebooting => -1,
            Self::Success => 0,
            Self::Failure => 1,
            Self::NoGpuCompatible => 2,
            Self::NotEnoughGpuMemory => 3,
            Self::CameraNotDetected => 4,
            Self::SensorsNotInitialized => 5,
            Self::SensorsNotAvailable => 6,
            Self::InvalidResolution => 7,
            Self::LowUsbBandwidth => 8,
            Self::CalibrationFileNotAvailable => 9,
            Self::InvalidCalibrationFile => 10,
            Self::InvalidSvoFile => 11,
            Self::SvoRecordingError => 12,
            Self::SvoUnsupportedCompression => 13,
            Self::EndOfSvoFileReached => 14,
            Self::InvalidCoordinateSystem => 15,
            Self::InvalidFirmware => 16,
            Self::InvalidFunctionParameters => 17,
            Self::CudaError => 18,
            Self::CameraNotInitialized => 19,
            Self::NvidiaDriverOutOfDate => 20,
            Self::InvalidFunctionCall => 21,
            Self::CorruptedSdkInstallation => 22,
            Self::IncompatibleSdkVersion => 23,
            Self::InvalidAreaFile => 24,
            Self::IncompatibleAreaFile => 25,
            Self::CameraFailedToSetup => 26,
            Self::CameraDetectionIssue => 27,
            Self::CannotStartCameraStream => 28,
            Self::NoGpuDetected => 29,
            Self::PlaneNotFound => 30,
            Self::ModuleNotCompatibleWithCamera => 31,
            Self::MotionSensorsRequired => 32,
            Self::ModuleNotCompatibleWithCudaVersion => 33,
            Self::DriverFailure => 34,
            Self::Unknown(raw) => raw,
        }
    }

    /// Returns `true` when the code represents a successful SDK call.
    pub fn is_success(self) -> bool {
        matches!(self, Self::Success)
    }

    /// Returns the canonical SDK constant name for this code.
    pub fn name(self) -> &'static str {
        match self {
            Self::SensorConfigurationChanged => "SENSOR_CONFIGURATION_CHANGED",
            Self::PotentialCalibrationIssue => "POTENTIAL_CALIBRATION_ISSUE",
            Self::ConfigurationFallback => "CONFIGURATION_FALLBACK",
            Self::SensorsDataRequired => "SENSORS_DATA_REQUIRED",
            Self::CorruptedFrame => "CORRUPTED_FRAME",
            Self::CameraRebooting => "CAMERA_REBOOTING",
            Self::Success => "SUCCESS",
            Self::Failure => "FAILURE",
            Self::NoGpuCompatible => "NO_GPU_COMPATIBLE",
            Self::NotEnoughGpuMemory => "NOT_ENOUGH_GPU_MEMORY",
            Self::CameraNotDetected => "CAMERA_NOT_DETECTED",
            Self::SensorsNotInitialized => "SENSORS_NOT_INITIALIZED",
            Self::SensorsNotAvailable => "SENSORS_NOT_AVAILABLE",
            Self::InvalidResolution => "INVALID_RESOLUTION",
            Self::LowUsbBandwidth => "LOW_USB_BANDWIDTH",
            Self::CalibrationFileNotAvailable => "CALIBRATION_FILE_NOT_AVAILABLE",
            Self::InvalidCalibrationFile => "INVALID_CALIBRATION_FILE",
            Self::InvalidSvoFile => "INVALID_SVO_FILE",
            Self::SvoRecordingError => "SVO_RECORDING_ERROR",
            Self::SvoUnsupportedCompression => "SVO_UNSUPPORTED_COMPRESSION",
            Self::EndOfSvoFileReached => "END_OF_SVOFILE_REACHED",
            Self::InvalidCoordinateSystem => "INVALID_COORDINATE_SYSTEM",
            Self::InvalidFirmware => "INVALID_FIRMWARE",
            Self::InvalidFunctionParameters => "INVALID_FUNCTION_PARAMETERS",
            Self::CudaError => "CUDA_ERROR",
            Self::CameraNotInitialized => "CAMERA_NOT_INITIALIZED",
            Self::NvidiaDriverOutOfDate => "NVIDIA_DRIVER_OUT_OF_DATE",
            Self::InvalidFunctionCall => "INVALID_FUNCTION_CALL",
            Self::CorruptedSdkInstallation => "CORRUPTED_SDK_INSTALLATION",
            Self::IncompatibleSdkVersion => "INCOMPATIBLE_SDK_VERSION",
            Self::InvalidAreaFile => "INVALID_AREA_FILE",
            Self::IncompatibleAreaFile => "INCOMPATIBLE_AREA_FILE",
            Self::CameraFailedToSetup => "CAMERA_FAILED_TO_SETUP",
            Self::CameraDetectionIssue => "CAMERA_DETECTION_ISSUE",
            Self::CannotStartCameraStream => "CANNOT_START_CAMERA_STREAM",
            Self::NoGpuDetected => "NO_GPU_DETECTED",
            Self::PlaneNotFound => "PLANE_NOT_FOUND",
            Self::ModuleNotCompatibleWithCamera => "MODULE_NOT_COMPATIBLE_WITH_CAMERA",
            Self::MotionSensorsRequired => "MOTION_SENSORS_REQUIRED",
            Self::ModuleNotCompatibleWithCudaVersion => "MODULE_NOT_COMPATIBLE_WITH_CUDA_VERSION",
            Self::DriverFailure => "DRIVER_FAILURE",
            Self::Unknown(_) => "UNKNOWN",
        }
    }
}

impl Display for ErrorCode {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.name())
    }
}

#[derive(Debug)]
/// Errors produced by the safe wrapper layer.
pub enum Error {
    Sdk {
        operation: &'static str,
        code: ErrorCode,
    },
    InteriorNul {
        field: &'static str,
    },
    InvalidResolution {
        context: &'static str,
        width: i32,
        height: i32,
    },
    DimensionOutOfRange {
        field: &'static str,
        value: u32,
    },
    NoFreeCameraSlot {
        capacity: usize,
    },
    CameraInformationUnavailable,
    NullPointer {
        operation: &'static str,
    },
    NullMatAllocation {
        width: u32,
        height: u32,
        mat_type: MatType,
        memory: MemoryType,
    },
    MatCpuAccessUnavailable {
        memory: MemoryType,
    },
    MatTypeMismatch {
        expected: MatType,
        actual: MatType,
    },
    MatStrideMisaligned {
        step_bytes: usize,
        element_size: usize,
    },
    MatStrideTooSmall {
        width_elems: usize,
        stride_elems: usize,
    },
    MatBufferTooSmall {
        required_elems: usize,
        actual_elems: usize,
    },
    MatPointerUnavailable,
}

impl Error {
    /// Returns the underlying SDK status code when the error came from a native call.
    pub fn sdk_code(&self) -> Option<ErrorCode> {
        match self {
            Self::Sdk { code, .. } => Some(*code),
            _ => None,
        }
    }
}

impl Display for Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::Sdk { operation, code } => {
                write!(f, "{operation} failed with {code} ({})", code.raw())
            }
            Self::InteriorNul { field } => write!(f, "{field} contains an interior NUL byte"),
            Self::InvalidResolution {
                context,
                width,
                height,
            } => write!(f, "{context} reported invalid resolution {width}x{height}"),
            Self::DimensionOutOfRange { field, value } => {
                write!(f, "{field} value {value} does not fit in i32")
            }
            Self::NoFreeCameraSlot { capacity } => {
                write!(f, "all {capacity} ZED camera slots are already reserved")
            }
            Self::CameraInformationUnavailable => {
                write!(f, "sl_get_camera_information returned a null pointer")
            }
            Self::NullPointer { operation } => {
                write!(f, "{operation} returned a null pointer")
            }
            Self::NullMatAllocation {
                width,
                height,
                mat_type,
                memory,
            } => write!(
                f,
                "failed to allocate {mat_type:?} mat in {memory:?} memory at {width}x{height}"
            ),
            Self::MatCpuAccessUnavailable { memory } => {
                write!(f, "mat does not expose CPU-accessible memory: {memory:?}")
            }
            Self::MatTypeMismatch { expected, actual } => {
                write!(f, "mat has element type {actual:?}, expected {expected:?}")
            }
            Self::MatStrideMisaligned {
                step_bytes,
                element_size,
            } => write!(
                f,
                "mat row stride {step_bytes} bytes is not aligned to element size {element_size}"
            ),
            Self::MatStrideTooSmall {
                width_elems,
                stride_elems,
            } => write!(
                f,
                "mat row stride {stride_elems} elements is smaller than width {width_elems}"
            ),
            Self::MatBufferTooSmall {
                required_elems,
                actual_elems,
            } => write!(
                f,
                "mat buffer needs {required_elems} elements but only {actual_elems} are available"
            ),
            Self::MatPointerUnavailable => write!(f, "mat returned a null data pointer"),
        }
    }
}

impl StdError for Error {}
