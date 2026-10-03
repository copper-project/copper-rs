use std::env;
use std::ffi::c_void;
use std::process::ExitCode;
use std::ptr;
use std::thread;
use std::time::{Duration, Instant};

use zed_sdk_sys::{
    SL_COORDINATE_SYSTEM, SL_DEPTH_MODE, SL_ERROR_CODE, SL_FLIP_MODE, SL_InitParameters,
    SL_MAT_TYPE, SL_MEASURE, SL_MEM, SL_REFERENCE_FRAME, SL_RESOLUTION, SL_RuntimeParameters,
    SL_UNIT, SL_VIEW, sl_close_camera, sl_create_camera, sl_get_camera_information, sl_get_height,
    sl_get_width, sl_grab, sl_mat_create_new, sl_mat_free, sl_mat_get_height,
    sl_mat_get_value_float, sl_mat_get_width, sl_open_camera, sl_retrieve_image,
    sl_retrieve_measure,
};

const CAMERA_ID: i32 = 0;
const EMPTY_CSTR: &[u8] = b"\0";

macro_rules! error_code_names {
    ($($variant:ident),+ $(,)?) => {
        fn error_code_name(code: i32) -> &'static str {
            match code {
                $(
                    value if value == SL_ERROR_CODE::$variant as i32 => stringify!($variant),
                )+
                _ => "UNKNOWN",
            }
        }
    };
}

error_code_names!(
    SL_ERROR_CODE_SENSOR_CONFIGURATION_CHANGED,
    SL_ERROR_CODE_POTENTIAL_CALIBRATION_ISSUE,
    SL_ERROR_CODE_CONFIGURATION_FALLBACK,
    SL_ERROR_CODE_SENSORS_DATA_REQUIRED,
    SL_ERROR_CODE_CORRUPTED_FRAME,
    SL_ERROR_CODE_CAMERA_REBOOTING,
    SL_ERROR_CODE_SUCCESS,
    SL_ERROR_CODE_FAILURE,
    SL_ERROR_CODE_NO_GPU_COMPATIBLE,
    SL_ERROR_CODE_NOT_ENOUGH_GPU_MEMORY,
    SL_ERROR_CODE_CAMERA_NOT_DETECTED,
    SL_ERROR_CODE_SENSORS_NOT_INITIALIZED,
    SL_ERROR_CODE_SENSORS_NOT_AVAILABLE,
    SL_ERROR_CODE_INVALID_RESOLUTION,
    SL_ERROR_CODE_LOW_USB_BANDWIDTH,
    SL_ERROR_CODE_CALIBRATION_FILE_NOT_AVAILABLE,
    SL_ERROR_CODE_INVALID_CALIBRATION_FILE,
    SL_ERROR_CODE_INVALID_SVO_FILE,
    SL_ERROR_CODE_SVO_RECORDING_ERROR,
    SL_ERROR_CODE_SVO_UNSUPPORTED_COMPRESSION,
    SL_ERROR_CODE_END_OF_SVOFILE_REACHED,
    SL_ERROR_CODE_INVALID_COORDINATE_SYSTEM,
    SL_ERROR_CODE_INVALID_FIRMWARE,
    SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS,
    SL_ERROR_CODE_CUDA_ERROR,
    SL_ERROR_CODE_CAMERA_NOT_INITIALIZED,
    SL_ERROR_CODE_NVIDIA_DRIVER_OUT_OF_DATE,
    SL_ERROR_CODE_INVALID_FUNCTION_CALL,
    SL_ERROR_CODE_CORRUPTED_SDK_INSTALLATION,
    SL_ERROR_CODE_INCOMPATIBLE_SDK_VERSION,
    SL_ERROR_CODE_INVALID_AREA_FILE,
    SL_ERROR_CODE_INCOMPATIBLE_AREA_FILE,
    SL_ERROR_CODE_CAMERA_FAILED_TO_SETUP,
    SL_ERROR_CODE_CAMERA_DETECTION_ISSUE,
    SL_ERROR_CODE_CANNOT_START_CAMERA_STREAM,
    SL_ERROR_CODE_NO_GPU_DETECTED,
    SL_ERROR_CODE_PLANE_NOT_FOUND,
    SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CAMERA,
    SL_ERROR_CODE_MOTION_SENSORS_REQUIRED,
    SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CUDA_VERSION,
    SL_ERROR_CODE_DRIVER_FAILURE,
);

struct CameraGuard {
    id: i32,
}

impl Drop for CameraGuard {
    fn drop(&mut self) {
        unsafe { sl_close_camera(self.id) };
    }
}

struct MatHandle {
    ptr: *mut c_void,
    mem: SL_MEM,
}

impl MatHandle {
    fn new(width: i32, height: i32, ty: SL_MAT_TYPE, mem: SL_MEM) -> Result<Self, String> {
        let ptr = unsafe { sl_mat_create_new(width, height, ty, mem) };
        if ptr.is_null() {
            return Err(format!(
                "sl_mat_create_new({width}, {height}, {:?}, {:?}) returned null",
                ty, mem
            ));
        }

        Ok(Self { ptr, mem })
    }

    fn as_mut_ptr(&self) -> *mut c_void {
        self.ptr
    }

    fn width(&self) -> i32 {
        unsafe { sl_mat_get_width(self.ptr) }
    }

    fn height(&self) -> i32 {
        unsafe { sl_mat_get_height(self.ptr) }
    }
}

impl Drop for MatHandle {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe { sl_mat_free(self.ptr, self.mem) };
        }
    }
}

fn main() -> ExitCode {
    let depth_mode = match env::args().nth(1) {
        Some(value) => match parse_depth_mode(&value) {
            Ok(mode) => mode,
            Err(err) => {
                eprintln!("{err}");
                return ExitCode::from(2);
            }
        },
        None => SL_DEPTH_MODE::SL_DEPTH_MODE_NEURAL_LIGHT,
    };

    let created = unsafe { sl_create_camera(CAMERA_ID) };
    if !created {
        eprintln!("sl_create_camera({CAMERA_ID}) returned false");
    }
    let _camera = CameraGuard { id: CAMERA_ID };

    let mut init_params = build_init_parameters(depth_mode);
    println!(
        "Running zed_lowlevel_demo with open_timeout_sec={}, resolution=HD720, fps={}, depth_mode={}",
        init_params.open_timeout_sec,
        init_params.camera_fps,
        depth_mode_name(init_params.depth_mode)
    );

    let start = Instant::now();
    let open_err = unsafe {
        sl_open_camera(
            CAMERA_ID,
            &mut init_params,
            0,
            EMPTY_CSTR.as_ptr().cast(),
            EMPTY_CSTR.as_ptr().cast(),
            0,
            -1,
            EMPTY_CSTR.as_ptr().cast(),
            EMPTY_CSTR.as_ptr().cast(),
            EMPTY_CSTR.as_ptr().cast(),
        )
    };
    let elapsed_ms = start.elapsed().as_millis();

    println!(
        "open() returned {} ({open_err}) after {elapsed_ms} ms",
        error_code_name(open_err)
    );
    if open_err != SL_ERROR_CODE::SL_ERROR_CODE_SUCCESS as i32 {
        return ExitCode::from(1);
    }

    let info = unsafe { sl_get_camera_information(CAMERA_ID, 0, 0) };
    if info.is_null() {
        println!("Opened camera, but sl_get_camera_information() returned null");
    } else {
        println!("Opened serial {}", unsafe { (*info).serial_number });
    }

    let width = unsafe { sl_get_width(CAMERA_ID) };
    let height = unsafe { sl_get_height(CAMERA_ID) };
    if width <= 0 || height <= 0 {
        eprintln!("invalid image size reported by SDK: {width}x{height}");
        return ExitCode::from(1);
    }

    let mut runtime = build_runtime_parameters(depth_mode);

    let mut grabbed = false;
    for index in 0..10 {
        let grab_err = unsafe { sl_grab(CAMERA_ID, &mut runtime) };
        println!(
            "grab[{index}] => {} ({grab_err})",
            error_code_name(grab_err)
        );
        if grab_err == SL_ERROR_CODE::SL_ERROR_CODE_SUCCESS as i32 {
            grabbed = true;
            break;
        }
        thread::sleep(Duration::from_millis(100));
    }

    if grabbed {
        let left_image = match MatHandle::new(
            width,
            height,
            SL_MAT_TYPE::SL_MAT_TYPE_U8_C4,
            SL_MEM::SL_MEM_CPU,
        ) {
            Ok(mat) => mat,
            Err(err) => {
                eprintln!("{err}");
                return ExitCode::from(1);
            }
        };

        let image_err = unsafe {
            sl_retrieve_image(
                CAMERA_ID,
                left_image.as_mut_ptr(),
                SL_VIEW::SL_VIEW_LEFT,
                SL_MEM::SL_MEM_CPU,
                width,
                height,
                ptr::null_mut(),
            )
        };
        println!(
            "retrieveImage(LEFT) => {} ({image_err})",
            error_code_name(image_err)
        );
        if image_err == SL_ERROR_CODE::SL_ERROR_CODE_SUCCESS as i32 {
            println!("left image: {}x{}", left_image.width(), left_image.height());
        }

        if depth_mode != SL_DEPTH_MODE::SL_DEPTH_MODE_NONE {
            let depth_map = match MatHandle::new(
                width,
                height,
                SL_MAT_TYPE::SL_MAT_TYPE_F32_C1,
                SL_MEM::SL_MEM_CPU,
            ) {
                Ok(mat) => mat,
                Err(err) => {
                    eprintln!("{err}");
                    return ExitCode::from(1);
                }
            };

            let depth_err = unsafe {
                sl_retrieve_measure(
                    CAMERA_ID,
                    depth_map.as_mut_ptr(),
                    SL_MEASURE::SL_MEASURE_DEPTH,
                    SL_MEM::SL_MEM_CPU,
                    width,
                    height,
                    ptr::null_mut(),
                )
            };
            println!(
                "retrieveMeasure(DEPTH) => {} ({depth_err})",
                error_code_name(depth_err)
            );

            if depth_err == SL_ERROR_CODE::SL_ERROR_CODE_SUCCESS as i32 {
                let cx = depth_map.width() / 2;
                let cy = depth_map.height() / 2;
                let mut depth_m = 0.0f32;
                let value_err = unsafe {
                    sl_mat_get_value_float(
                        depth_map.as_mut_ptr(),
                        cx,
                        cy,
                        &mut depth_m,
                        SL_MEM::SL_MEM_CPU,
                    )
                };

                print!(
                    "depth[{cx},{cy}] => {} ({value_err})",
                    error_code_name(value_err)
                );
                if value_err == SL_ERROR_CODE::SL_ERROR_CODE_SUCCESS as i32 {
                    if depth_m.is_finite() {
                        print!(" ({depth_m} m)");
                    } else {
                        print!(" (non-finite)");
                    }
                }
                println!();
            }
        }
    }

    ExitCode::SUCCESS
}

fn build_init_parameters(depth_mode: SL_DEPTH_MODE) -> SL_InitParameters {
    SL_InitParameters {
        input_type: zed_sdk_sys::SL_INPUT_TYPE::SL_INPUT_TYPE_USB,
        resolution: SL_RESOLUTION::SL_RESOLUTION_HD720,
        camera_fps: 30,
        camera_device_id: CAMERA_ID,
        camera_image_flip: SL_FLIP_MODE::SL_FLIP_MODE_AUTO,
        camera_disable_self_calib: false,
        enable_right_side_measure: false,
        svo_real_time_mode: false,
        depth_mode,
        depth_stabilization: 30,
        depth_minimum_distance: -1.0,
        depth_maximum_distance: 40.0,
        coordinate_unit: SL_UNIT::SL_UNIT_METER,
        coordinate_system: SL_COORDINATE_SYSTEM::SL_COORDINATE_SYSTEM_LEFT_HANDED_Y_UP,
        sdk_gpu_id: -1,
        sdk_verbose: 0,
        sensors_required: false,
        enable_image_enhancement: true,
        open_timeout_sec: 20.0,
        async_grab_camera_recovery: false,
        grab_compute_capping_fps: 0.0,
        enable_image_validity_check: false,
        maximum_working_resolution: Default::default(),
    }
}

fn build_runtime_parameters(depth_mode: SL_DEPTH_MODE) -> SL_RuntimeParameters {
    SL_RuntimeParameters {
        reference_frame: SL_REFERENCE_FRAME::SL_REFERENCE_FRAME_CAMERA,
        enable_depth: depth_mode != SL_DEPTH_MODE::SL_DEPTH_MODE_NONE,
        enable_fill_mode: false,
        confidence_threshold: 95,
        texture_confidence_threshold: 100,
        remove_saturated_areas: true,
    }
}

fn parse_depth_mode(value: &str) -> Result<SL_DEPTH_MODE, String> {
    match value {
        "NONE" => Ok(SL_DEPTH_MODE::SL_DEPTH_MODE_NONE),
        "PERFORMANCE" => Ok(SL_DEPTH_MODE::SL_DEPTH_MODE_PERFORMANCE),
        "QUALITY" => Ok(SL_DEPTH_MODE::SL_DEPTH_MODE_QUALITY),
        "ULTRA" => Ok(SL_DEPTH_MODE::SL_DEPTH_MODE_ULTRA),
        "NEURAL_LIGHT" => Ok(SL_DEPTH_MODE::SL_DEPTH_MODE_NEURAL_LIGHT),
        "NEURAL" => Ok(SL_DEPTH_MODE::SL_DEPTH_MODE_NEURAL),
        "NEURAL_PLUS" => Ok(SL_DEPTH_MODE::SL_DEPTH_MODE_NEURAL_PLUS),
        _ => Err(format!("unsupported depth mode: {value}")),
    }
}

fn depth_mode_name(mode: SL_DEPTH_MODE) -> &'static str {
    match mode {
        SL_DEPTH_MODE::SL_DEPTH_MODE_NONE => "NONE",
        SL_DEPTH_MODE::SL_DEPTH_MODE_PERFORMANCE => "PERFORMANCE",
        SL_DEPTH_MODE::SL_DEPTH_MODE_QUALITY => "QUALITY",
        SL_DEPTH_MODE::SL_DEPTH_MODE_ULTRA => "ULTRA",
        SL_DEPTH_MODE::SL_DEPTH_MODE_NEURAL_LIGHT => "NEURAL_LIGHT",
        SL_DEPTH_MODE::SL_DEPTH_MODE_NEURAL => "NEURAL",
        SL_DEPTH_MODE::SL_DEPTH_MODE_NEURAL_PLUS => "NEURAL_PLUS",
    }
}
