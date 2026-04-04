use std::ffi::{CString, c_char};
use std::path::Path;
use std::ptr;
use std::sync::{Mutex, MutexGuard};

use crate::error::{Error, ErrorCode, Result};
use crate::mat::{Mat, MatElement};
use crate::sys;
use crate::types::{
    CalibrationParameters, CameraImuTransform, CameraInformation, InputSource, OpenOptions,
    Point3Color, Resolution, Rgba8, RuntimeParameters, SensorsConfiguration, SensorsData, Vec3f,
};

static RESERVED_CAMERA_IDS: Mutex<[bool; sys::MAX_CAMERA_PLUGIN]> =
    Mutex::new([false; sys::MAX_CAMERA_PLUGIN]);

const EMPTY_CSTR: &[u8] = b"\0";

pub struct Camera {
    camera_id: i32,
}

impl Camera {
    pub fn open(options: OpenOptions) -> Result<Self> {
        let camera_id = reserve_camera_id()?;
        let mut init = options.to_raw();
        let output_log_file = options
            .output_log_file
            .as_deref()
            .map(|path| path_to_cstring(path, "output_log_file"))
            .transpose()?;
        let settings_path = options
            .settings_path
            .as_deref()
            .map(|path| path_to_cstring(path, "settings_path"))
            .transpose()?;
        let opencv_calibration_path = options
            .opencv_calibration_path
            .as_deref()
            .map(|path| path_to_cstring(path, "opencv_calibration_path"))
            .transpose()?;

        let open_result = match &options.source {
            InputSource::Usb { .. } => unsafe {
                sys::sl_open_camera_from_camera_id(
                    camera_id,
                    &mut init,
                    optional_cstring_ptr(output_log_file.as_ref()),
                    optional_cstring_ptr(settings_path.as_ref()),
                    optional_cstring_ptr(opencv_calibration_path.as_ref()),
                )
            },
            InputSource::SerialNumber(serial_number) => unsafe {
                sys::sl_open_camera_from_serial_number(
                    camera_id,
                    &mut init,
                    *serial_number,
                    optional_cstring_ptr(output_log_file.as_ref()),
                    optional_cstring_ptr(settings_path.as_ref()),
                    optional_cstring_ptr(opencv_calibration_path.as_ref()),
                )
            },
            InputSource::SvoFile(path) => {
                let path = path_to_cstring(path, "source.svo_file")?;
                unsafe {
                    sys::sl_open_camera_from_svo_file(
                        camera_id,
                        &mut init,
                        path.as_ptr(),
                        optional_cstring_ptr(output_log_file.as_ref()),
                        optional_cstring_ptr(settings_path.as_ref()),
                        optional_cstring_ptr(opencv_calibration_path.as_ref()),
                    )
                }
            }
            InputSource::Stream { ip, port } => {
                let ip = CString::new(ip.as_bytes())
                    .map_err(|_| Error::InteriorNul { field: "source.ip" })?;
                unsafe {
                    sys::sl_open_camera_from_stream(
                        camera_id,
                        &mut init,
                        ip.as_ptr(),
                        *port,
                        optional_cstring_ptr(output_log_file.as_ref()),
                        optional_cstring_ptr(settings_path.as_ref()),
                        optional_cstring_ptr(opencv_calibration_path.as_ref()),
                    )
                }
            }
            InputSource::Gmsl {
                serial_number,
                port,
            } => unsafe {
                sys::sl_open_camera(
                    camera_id,
                    &mut init,
                    *serial_number,
                    empty_cstring_ptr(),
                    empty_cstring_ptr(),
                    0,
                    *port,
                    optional_cstring_ptr(output_log_file.as_ref()),
                    optional_cstring_ptr(settings_path.as_ref()),
                    optional_cstring_ptr(opencv_calibration_path.as_ref()),
                )
            },
        };

        match ErrorCode::from_raw(open_result) {
            code if code.is_success() => Ok(Self { camera_id }),
            code => {
                release_camera_id(camera_id);
                Err(Error::Sdk {
                    operation: "sl_open_camera",
                    code,
                })
            }
        }
    }

    pub fn is_opened(&self) -> bool {
        unsafe { sys::sl_is_opened(self.camera_id) }
    }

    pub fn sdk_version() -> Option<String> {
        let ptr = unsafe { sys::sl_get_sdk_version() };
        if ptr.is_null() {
            return None;
        }

        Some(
            unsafe { std::ffi::CStr::from_ptr(ptr) }
                .to_string_lossy()
                .into_owned(),
        )
    }

    pub fn info(&self) -> Result<CameraInformation> {
        let ptr = unsafe { sys::sl_get_camera_information(self.camera_id, 0, 0) };
        if ptr.is_null() {
            return Err(Error::CameraInformationUnavailable);
        }

        let raw = unsafe { *ptr };
        CameraInformation::from_raw(raw)
    }

    pub fn resolution(&self) -> Result<Resolution> {
        let width = unsafe { sys::sl_get_width(self.camera_id) };
        let height = unsafe { sys::sl_get_height(self.camera_id) };
        Resolution::try_from_raw(width, height, "camera")
    }

    pub fn camera_fps(&self) -> f32 {
        unsafe { sys::sl_get_camera_fps(self.camera_id) }
    }

    pub fn current_fps(&self) -> f32 {
        unsafe { sys::sl_get_current_fps(self.camera_id) }
    }

    pub fn image_timestamp(&self) -> u64 {
        unsafe { sys::sl_get_image_timestamp(self.camera_id) }
    }

    pub fn current_timestamp(&self) -> u64 {
        unsafe { sys::sl_get_current_timestamp(self.camera_id) }
    }

    pub fn grab(&mut self, runtime: &RuntimeParameters) -> Result<()> {
        let mut raw = runtime.to_raw();
        check_code("sl_grab", unsafe { sys::sl_grab(self.camera_id, &mut raw) })
    }

    pub fn read(&mut self) -> Result<()> {
        check_code("sl_read", unsafe { sys::sl_read(self.camera_id) })
    }

    pub fn retrieve_left(&mut self, target: &mut Mat<Rgba8>) -> Result<()> {
        self.retrieve_image_raw(sys::SL_VIEW::SL_VIEW_LEFT, target)
    }

    pub fn retrieve_right(&mut self, target: &mut Mat<Rgba8>) -> Result<()> {
        self.retrieve_image_raw(sys::SL_VIEW::SL_VIEW_RIGHT, target)
    }

    pub fn retrieve_depth(&mut self, target: &mut Mat<f32>) -> Result<()> {
        self.retrieve_measure_raw(sys::SL_MEASURE::SL_MEASURE_DEPTH, target)
    }

    pub fn retrieve_confidence(&mut self, target: &mut Mat<f32>) -> Result<()> {
        self.retrieve_measure_raw(sys::SL_MEASURE::SL_MEASURE_CONFIDENCE, target)
    }

    pub fn retrieve_xyz(&mut self, target: &mut Mat<Vec3f>) -> Result<()> {
        self.retrieve_measure_raw(sys::SL_MEASURE::SL_MEASURE_XYZ, target)
    }

    pub fn retrieve_xyzrgba(&mut self, target: &mut Mat<Point3Color>) -> Result<()> {
        self.retrieve_measure_raw(sys::SL_MEASURE::SL_MEASURE_XYZRGBA, target)
    }

    pub fn retrieve_image_raw<T: MatElement>(
        &mut self,
        view: sys::SL_VIEW,
        target: &mut Mat<T>,
    ) -> Result<()> {
        let resolution = target.resolution()?;
        check_code("sl_retrieve_image", unsafe {
            sys::sl_retrieve_image(
                self.camera_id,
                target.as_raw_mut_ptr(),
                view,
                target.memory_type().as_raw(),
                resolution.width_i32()?,
                resolution.height_i32()?,
                ptr::null_mut(),
            )
        })
    }

    pub fn retrieve_measure_raw<T: MatElement>(
        &mut self,
        measure: sys::SL_MEASURE,
        target: &mut Mat<T>,
    ) -> Result<()> {
        let resolution = target.resolution()?;
        check_code("sl_retrieve_measure", unsafe {
            sys::sl_retrieve_measure(
                self.camera_id,
                target.as_raw_mut_ptr(),
                measure,
                target.memory_type().as_raw(),
                resolution.width_i32()?,
                resolution.height_i32()?,
                ptr::null_mut(),
            )
        })
    }

    pub fn calibration_parameters(&self, raw: bool) -> Result<CalibrationParameters> {
        let ptr = unsafe { sys::sl_get_calibration_parameters(self.camera_id, raw) };
        if ptr.is_null() {
            return Err(Error::NullPointer {
                operation: "sl_get_calibration_parameters",
            });
        }

        CalibrationParameters::from_raw(unsafe { *ptr })
    }

    pub fn sensors_configuration(&self) -> Result<SensorsConfiguration> {
        let ptr = unsafe { sys::sl_get_sensors_configuration(self.camera_id) };
        if ptr.is_null() {
            return Err(Error::NullPointer {
                operation: "sl_get_sensors_configuration",
            });
        }

        Ok(SensorsConfiguration::from_raw(unsafe { *ptr }))
    }

    pub fn camera_imu_transform(&self) -> Result<CameraImuTransform> {
        let mut translation = sys::SL_Vector3::default();
        let mut rotation = sys::SL_Quaternion::default();
        unsafe {
            sys::sl_get_camera_imu_transform(self.camera_id, &mut translation, &mut rotation);
        }
        Ok(CameraImuTransform::from_raw(translation, rotation))
    }

    pub fn sensors_data(&self) -> Result<SensorsData> {
        let mut raw = sys::SL_SensorsData::default();
        check_code("sl_get_sensors_data", unsafe {
            sys::sl_get_sensors_data(
                self.camera_id,
                &mut raw,
                sys::SL_TIME_REFERENCE::SL_TIME_REFERENCE_IMAGE,
            )
        })?;
        Ok(SensorsData::from_raw(raw))
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        unsafe { sys::sl_close_camera(self.camera_id) };
        release_camera_id(self.camera_id);
    }
}

fn check_code(operation: &'static str, raw_code: i32) -> Result<()> {
    let code = ErrorCode::from_raw(raw_code);
    if code.is_success() {
        Ok(())
    } else {
        Err(Error::Sdk { operation, code })
    }
}

fn reserve_camera_id() -> Result<i32> {
    let mut ids = id_pool();
    if let Some((index, slot)) = ids.iter_mut().enumerate().find(|(_, reserved)| !**reserved) {
        *slot = true;
        return Ok(index as i32);
    }

    Err(Error::NoFreeCameraSlot {
        capacity: sys::MAX_CAMERA_PLUGIN,
    })
}

fn release_camera_id(camera_id: i32) {
    if let Ok(index) = usize::try_from(camera_id) {
        let mut ids = id_pool();
        if index < ids.len() {
            ids[index] = false;
        }
    }
}

fn id_pool() -> MutexGuard<'static, [bool; sys::MAX_CAMERA_PLUGIN]> {
    RESERVED_CAMERA_IDS
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn optional_cstring_ptr(value: Option<&CString>) -> *const c_char {
    value.map_or_else(empty_cstring_ptr, |value| value.as_ptr())
}

fn empty_cstring_ptr() -> *const c_char {
    EMPTY_CSTR.as_ptr().cast()
}

fn path_to_cstring(path: &Path, field: &'static str) -> Result<CString> {
    use std::os::unix::ffi::OsStrExt;

    CString::new(path.as_os_str().as_bytes()).map_err(|_| Error::InteriorNul { field })
}
