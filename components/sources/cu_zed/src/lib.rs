mod payloads;

pub use payloads::*;

use cu_sensor_payloads::{
    BarometerPayload, CuImage, CuImageBufferFormat, ImuPayload, MagnetometerPayload,
};
use cu_transform::FrameTransform;
use cu29::prelude::*;

pub type ZedStereoImages = (CuImage<Vec<u8>>, CuImage<Vec<u8>>);

pub type ZedSourceOutputs = (
    CuMsg<ZedStereoImages>,
    CuMsg<ZedDepthMap<Vec<f32>>>,
    CuMsg<ZedConfidenceMap<Vec<f32>>>,
    CuMsg<CuLatchedStateUpdate<ZedCalibrationBundle>>,
    CuMsg<CuLatchedStateUpdate<ZedRigTransforms>>,
    CuMsg<ImuPayload>,
    CuMsg<MagnetometerPayload>,
    CuMsg<BarometerPayload>,
    CuMsg<ZedFrameMeta>,
);

#[cfg(not(target_os = "linux"))]
mod empty_impl {
    use super::*;

    #[derive(Reflect)]
    #[reflect(from_reflect = false)]
    pub struct Zed;

    impl Freezable for Zed {}

    impl CuSrcTask for Zed {
        type Resources<'r> = ();
        type Output<'m> = ZedSourceOutputs;

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            let (stereo, depth, confidence, calib, transforms, imu, mag, baro, meta) = output;
            stereo.clear_payload();
            depth.clear_payload();
            confidence.clear_payload();
            calib.set_payload(CuLatchedStateUpdate::NoChange);
            transforms.set_payload(CuLatchedStateUpdate::NoChange);
            imu.clear_payload();
            mag.clear_payload();
            baro.clear_payload();
            meta.clear_payload();
            Ok(())
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub use empty_impl::Zed;

#[cfg(target_os = "linux")]
mod linux_impl {
    use super::*;
    use core::mem::size_of;
    use std::path::PathBuf;
    use std::sync::Arc;
    use zed_sdk::{
        CalibrationParameters, Camera, CameraImuTransform, CameraInformation, CameraParameters,
        CoordinateSystem, DepthMode, ErrorCode, InputSource, OpenOptions, ReferenceFrame,
        ResolutionPreset, RuntimeParameters, SensorsConfiguration, SensorsData, Unit,
    };
    use zed_sdk::{Mat, Rgba8};

    #[derive(Reflect)]
    #[reflect(from_reflect = false)]
    pub struct Zed {
        #[reflect(ignore)]
        camera: Camera,
        #[reflect(ignore)]
        runtime: RuntimeParameters,
        #[reflect(ignore)]
        left_mat: Mat<Rgba8>,
        #[reflect(ignore)]
        right_mat: Mat<Rgba8>,
        #[reflect(ignore)]
        depth_mat: Mat<f32>,
        #[reflect(ignore)]
        confidence_mat: Option<Mat<f32>>,
        #[reflect(ignore)]
        left_pool: Arc<CuHostMemoryPool<Vec<u8>>>,
        #[reflect(ignore)]
        right_pool: Arc<CuHostMemoryPool<Vec<u8>>>,
        #[reflect(ignore)]
        depth_pool: Arc<CuHostMemoryPool<Vec<f32>>>,
        #[reflect(ignore)]
        confidence_pool: Option<Arc<CuHostMemoryPool<Vec<f32>>>>,
        #[reflect(ignore)]
        left_format: CuImageBufferFormat,
        #[reflect(ignore)]
        right_format: CuImageBufferFormat,
        #[reflect(ignore)]
        depth_format: ZedRasterFormat,
        #[reflect(ignore)]
        confidence_format: Option<ZedRasterFormat>,
        emit_confidence: bool,
        emit_imu: bool,
        emit_mag: bool,
        emit_baro: bool,
        seq: u64,
        #[reflect(ignore)]
        pending_calibration: Option<ZedCalibrationBundle>,
        #[reflect(ignore)]
        pending_transforms: Option<ZedRigTransforms>,
    }

    impl Freezable for Zed {}

    // SAFETY: `Zed` owns opaque SDK handles and Copper drives each task instance through
    // exclusive `&mut self` process calls. We never expose aliased mutable access to the
    // underlying SDK resources through this wrapper.
    unsafe impl Send for Zed {}
    // SAFETY: shared references do not permit mutation of the owned SDK handles through this
    // wrapper. Copper requires `Reflect` tasks to be `Sync`, so the task advertises that bound.
    unsafe impl Sync for Zed {}

    impl CuSrcTask for Zed {
        type Resources<'r> = ();
        type Output<'m> = ZedSourceOutputs;

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let open = build_open_options(config)?;
            let runtime = build_runtime_parameters(config)?;
            let emit_confidence = config_bool(config, "emit_confidence", false)?;
            let emit_imu = config_bool(config, "emit_imu", true)?;
            let emit_mag = config_bool(config, "emit_mag", true)?;
            let emit_baro = config_bool(config, "emit_baro", true)?;
            let pool_slots = config_u32(config, "pool_slots", 4)?.max(1) as usize;
            let frame_prefix =
                config_string_opt(config, "frame_id_prefix")?.unwrap_or_else(|| "zed".to_string());

            let camera = Camera::open(open)
                .map_err(|e| CuError::new_with_cause("Could not open ZED camera", e))?;
            let info = camera.info().map_err(|e| {
                CuError::new_with_cause("Could not fetch ZED camera information", e)
            })?;
            let calibration = camera.calibration_parameters(false).map_err(|e| {
                CuError::new_with_cause("Could not fetch ZED calibration parameters", e)
            })?;
            let sensors_configuration = camera.sensors_configuration().ok();
            let camera_imu = sensors_configuration
                .as_ref()
                .filter(|config| has_motion_sensors(config))
                .map(|_| camera.camera_imu_transform())
                .transpose()
                .map_err(|e| {
                    CuError::new_with_cause("Could not fetch ZED camera->IMU transform", e)
                })?;

            let resolution = camera
                .resolution()
                .map_err(|e| CuError::new_with_cause("Could not read ZED camera resolution", e))?;

            let left_mat = Mat::new_cpu(resolution)
                .map_err(|e| CuError::new_with_cause("Could not allocate ZED left frame mat", e))?;
            let right_mat = Mat::new_cpu(resolution).map_err(|e| {
                CuError::new_with_cause("Could not allocate ZED right frame mat", e)
            })?;
            let depth_mat = Mat::new_cpu(resolution)
                .map_err(|e| CuError::new_with_cause("Could not allocate ZED depth mat", e))?;
            let confidence_mat = if emit_confidence {
                Some(Mat::new_cpu(resolution).map_err(|e| {
                    CuError::new_with_cause("Could not allocate ZED confidence mat", e)
                })?)
            } else {
                None
            };

            let left_format = rgba_format(&left_mat)
                .map_err(|e| CuError::new_with_cause("Could not inspect ZED left mat", e))?;
            let right_format = rgba_format(&right_mat)
                .map_err(|e| CuError::new_with_cause("Could not inspect ZED right mat", e))?;
            let depth_format = raster_format(&depth_mat)
                .map_err(|e| CuError::new_with_cause("Could not inspect ZED depth mat", e))?;
            let confidence_format = confidence_mat
                .as_ref()
                .map(raster_format)
                .transpose()
                .map_err(|e| CuError::new_with_cause("Could not inspect ZED confidence mat", e))?;

            let left_pool = CuHostMemoryPool::new("zed-left", pool_slots, || {
                vec![0u8; left_format.byte_size()]
            })?;
            let right_pool = CuHostMemoryPool::new("zed-right", pool_slots, || {
                vec![0u8; right_format.byte_size()]
            })?;
            let depth_pool = CuHostMemoryPool::new("zed-depth", pool_slots, || {
                vec![0f32; depth_format.len_elements()]
            })?;
            let confidence_pool = confidence_format
                .map(|format| {
                    CuHostMemoryPool::new("zed-confidence", pool_slots, || {
                        vec![0f32; format.len_elements()]
                    })
                })
                .transpose()?;

            let pending_calibration = Some(build_calibration_bundle(
                &info,
                &calibration,
                camera_imu.as_ref(),
            ));
            let pending_left_to_right = build_left_to_right_transform(&frame_prefix, &calibration);
            let pending_camera_to_imu = camera_imu
                .as_ref()
                .map(|transform| build_camera_to_imu_transform(&frame_prefix, transform));
            let pending_transforms = Some(ZedRigTransforms {
                left_to_right: ZedNamedTransform::from_frame_transform(&pending_left_to_right),
                camera_to_imu: pending_camera_to_imu
                    .as_ref()
                    .map(ZedNamedTransform::from_frame_transform)
                    .unwrap_or_default(),
                has_camera_to_imu: pending_camera_to_imu.is_some(),
            });

            Ok(Self {
                camera,
                runtime,
                left_mat,
                right_mat,
                depth_mat,
                confidence_mat,
                left_pool,
                right_pool,
                depth_pool,
                confidence_pool,
                left_format,
                right_format,
                depth_format,
                confidence_format,
                emit_confidence,
                emit_imu,
                emit_mag,
                emit_baro,
                seq: 0,
                pending_calibration,
                pending_transforms,
            })
        }

        fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            let (stereo, depth, confidence, calibration, transforms, imu, mag, baro, meta) = output;

            self.camera
                .grab(&self.runtime)
                .map_err(|e| CuError::new_with_cause("ZED grab failed", e))?;

            let frame_tov: CuTime = self.camera.image_timestamp().into();
            let seq = self.seq;
            self.seq = self.seq.wrapping_add(1);

            self.camera
                .retrieve_left(&mut self.left_mat)
                .map_err(|e| CuError::new_with_cause("Could not retrieve ZED left image", e))?;
            self.camera
                .retrieve_right(&mut self.right_mat)
                .map_err(|e| CuError::new_with_cause("Could not retrieve ZED right image", e))?;
            self.camera
                .retrieve_depth(&mut self.depth_mat)
                .map_err(|e| CuError::new_with_cause("Could not retrieve ZED depth map", e))?;

            stereo.set_payload((
                copy_image_payload(seq, &self.left_mat, &self.left_pool, self.left_format)?,
                copy_image_payload(seq, &self.right_mat, &self.right_pool, self.right_format)?,
            ));
            stereo.tov = Tov::Time(frame_tov);

            depth.set_payload(copy_raster_payload(
                seq,
                &self.depth_mat,
                &self.depth_pool,
                self.depth_format,
                ZedDepthMap::new,
            )?);
            depth.tov = Tov::Time(frame_tov);

            if self.emit_confidence {
                let confidence_mat = self.confidence_mat.as_mut().ok_or_else(|| {
                    CuError::from("confidence output requested without a backing mat")
                })?;
                self.camera
                    .retrieve_confidence(confidence_mat)
                    .map_err(|e| {
                        CuError::new_with_cause("Could not retrieve ZED confidence map", e)
                    })?;
                let confidence_pool = self.confidence_pool.as_ref().ok_or_else(|| {
                    CuError::from("confidence output requested without a backing pool")
                })?;
                let confidence_format = self
                    .confidence_format
                    .ok_or_else(|| CuError::from("confidence output requested without a format"))?;
                confidence.set_payload(copy_raster_payload(
                    seq,
                    confidence_mat,
                    confidence_pool,
                    confidence_format,
                    ZedConfidenceMap::new,
                )?);
                confidence.tov = Tov::Time(frame_tov);
            } else {
                confidence.clear_payload();
            }

            emit_latched(calibration, frame_tov, self.pending_calibration.take());
            emit_latched(transforms, frame_tov, self.pending_transforms.take());

            let sensor_result = self.camera.sensors_data();
            let sensors = match sensor_result {
                Ok(sensors) => Some(sensors),
                Err(err) if is_optional_sensor_error(&err) => None,
                Err(err) => {
                    return Err(CuError::new_with_cause(
                        "Could not retrieve ZED sensors data",
                        err,
                    ));
                }
            };

            if self.emit_imu {
                if let Some(imu_data) = sensors.as_ref().and_then(|data| data.imu) {
                    let temperature_c = sensors
                        .as_ref()
                        .map(|data| data.temperature.imu_temp_c)
                        .unwrap_or_default();
                    imu.set_payload(ImuPayload::from_raw(
                        [
                            imu_data.linear_acceleration.x,
                            imu_data.linear_acceleration.y,
                            imu_data.linear_acceleration.z,
                        ],
                        [
                            imu_data.angular_velocity.x,
                            imu_data.angular_velocity.y,
                            imu_data.angular_velocity.z,
                        ],
                        temperature_c,
                    ));
                    imu.tov = Tov::Time(imu_data.timestamp_ns.into());
                } else {
                    imu.clear_payload();
                }
            } else {
                imu.clear_payload();
            }

            if self.emit_mag {
                if let Some(mag_data) = sensors.as_ref().and_then(|data| data.magnetometer) {
                    mag.set_payload(MagnetometerPayload::from_raw([
                        mag_data.magnetic_field_ut.x,
                        mag_data.magnetic_field_ut.y,
                        mag_data.magnetic_field_ut.z,
                    ]));
                    mag.tov = Tov::Time(mag_data.timestamp_ns.into());
                } else {
                    mag.clear_payload();
                }
            } else {
                mag.clear_payload();
            }

            if self.emit_baro {
                if let Some(baro_data) = sensors.as_ref().and_then(|data| data.barometer) {
                    let temperature_c = sensors
                        .as_ref()
                        .map(|data| data.temperature.barometer_temp_c)
                        .unwrap_or_default();
                    baro.set_payload(BarometerPayload::from_raw(
                        baro_data.pressure_pa,
                        temperature_c,
                    ));
                    baro.tov = Tov::Time(baro_data.timestamp_ns.into());
                } else {
                    baro.clear_payload();
                }
            } else {
                baro.clear_payload();
            }

            meta.set_payload(build_frame_meta(seq, &self.camera, sensors.as_ref()));
            meta.tov = Tov::Time(frame_tov);

            Ok(())
        }
    }

    fn config_bool(config: Option<&ComponentConfig>, key: &str, default: bool) -> CuResult<bool> {
        Ok(match config {
            Some(config) => config.get::<bool>(key)?.unwrap_or(default),
            None => default,
        })
    }

    fn config_u32(config: Option<&ComponentConfig>, key: &str, default: u32) -> CuResult<u32> {
        Ok(match config {
            Some(config) => config.get::<u32>(key)?.unwrap_or(default),
            None => default,
        })
    }

    fn config_string_opt(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<String>> {
        match config {
            Some(config) => config.get::<String>(key).map_err(Into::into),
            None => Ok(None),
        }
    }

    fn build_open_options(config: Option<&ComponentConfig>) -> CuResult<OpenOptions> {
        let mut options = OpenOptions::default();

        if let (Some(serial_number), Some(port)) = (
            config_u32_opt(config, "gmsl_serial_number")?,
            config_i32_opt(config, "gmsl_port")?,
        ) {
            options = options.source(InputSource::Gmsl {
                serial_number,
                port,
            });
        } else if let Some(serial_number) = config_u32_opt(config, "serial_number")? {
            options = options.source(InputSource::SerialNumber(serial_number));
        } else if let Some(svo_file) = config_string_opt(config, "svo_file")? {
            options = options.source(InputSource::SvoFile(PathBuf::from(svo_file)));
        } else if let Some(stream_ip) = config_string_opt(config, "stream_ip")? {
            let stream_port = config_i32_opt(config, "stream_port")?.unwrap_or(30000);
            options = options.source(InputSource::Stream {
                ip: stream_ip,
                port: stream_port,
            });
        } else {
            let device_id = config_i32_opt(config, "device_id")?.unwrap_or(0);
            options = options.camera_device_id(device_id);
        }

        if let Some(resolution) = config_string_opt(config, "resolution")? {
            let parsed = resolution
                .to_ascii_uppercase()
                .parse::<ResolutionPreset>()
                .map_err(|err| CuError::from(format!("Invalid ZED resolution preset: {err}")))?;
            options = options.resolution(parsed);
        }
        if let Some(fps) = config_i32_opt(config, "fps")? {
            options = options.fps(fps);
        }
        if let Some(depth_mode) = config_string_opt(config, "depth_mode")? {
            let parsed = depth_mode
                .to_ascii_uppercase()
                .parse::<DepthMode>()
                .map_err(|err| CuError::from(format!("Invalid ZED depth mode: {err}")))?;
            options = options.depth_mode(parsed);
        }
        if let Some(coordinate_system) = config_string_opt(config, "coordinate_system")? {
            let parsed = parse_coordinate_system(&coordinate_system)?;
            options = options.coordinate_system(parsed);
        }
        if let Some(coordinate_unit) = config_string_opt(config, "coordinate_unit")? {
            let parsed = parse_coordinate_unit(&coordinate_unit)?;
            options = options.coordinate_unit(parsed);
        }

        let depth_min = config_f32_opt(config, "depth_minimum_distance_m")?;
        let depth_max = config_f32_opt(config, "depth_maximum_distance_m")?.unwrap_or(40.0);
        options = options.depth_range_m(depth_min, depth_max);

        if let Some(timeout_ms) = config_u32_opt(config, "open_timeout_ms")? {
            options = options.open_timeout(core::time::Duration::from_millis(timeout_ms as u64));
        }
        if let Some(sensors_required) = config_bool_opt(config, "sensors_required")? {
            options = options.sensors_required(sensors_required);
        }
        if let Some(verbose) = config_i32_opt(config, "sdk_verbose")? {
            options = options.sdk_verbose(verbose);
        }
        if let Some(settings_path) = config_string_opt(config, "settings_path")? {
            options = options.settings_path(settings_path);
        }
        if let Some(calibration_path) = config_string_opt(config, "opencv_calibration_path")? {
            options = options.opencv_calibration_path(calibration_path);
        }

        Ok(options)
    }

    fn build_runtime_parameters(config: Option<&ComponentConfig>) -> CuResult<RuntimeParameters> {
        let mut runtime = RuntimeParameters::default();
        if let Some(reference_frame) = config_string_opt(config, "reference_frame")? {
            let parsed = parse_reference_frame(&reference_frame)?;
            runtime = runtime.reference_frame(parsed);
        }
        if let Some(enable_fill_mode) = config_bool_opt(config, "enable_fill_mode")? {
            runtime = runtime.enable_fill_mode(enable_fill_mode);
        }
        if let Some(confidence_threshold) = config_i32_opt(config, "confidence_threshold")? {
            runtime = runtime.confidence_threshold(confidence_threshold);
        }
        if let Some(texture_confidence_threshold) =
            config_i32_opt(config, "texture_confidence_threshold")?
        {
            runtime = runtime.texture_confidence_threshold(texture_confidence_threshold);
        }
        Ok(runtime)
    }

    fn config_bool_opt(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<bool>> {
        match config {
            Some(config) => config.get::<bool>(key).map_err(Into::into),
            None => Ok(None),
        }
    }

    fn config_u32_opt(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<u32>> {
        match config {
            Some(config) => config.get::<u32>(key).map_err(Into::into),
            None => Ok(None),
        }
    }

    fn config_i32_opt(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<i32>> {
        match config {
            Some(config) => config.get::<i32>(key).map_err(Into::into),
            None => Ok(None),
        }
    }

    fn config_f32_opt(config: Option<&ComponentConfig>, key: &str) -> CuResult<Option<f32>> {
        match config {
            Some(config) => config
                .get::<f64>(key)
                .map(|value| value.map(|value| value as f32))
                .map_err(Into::into),
            None => Ok(None),
        }
    }

    fn parse_coordinate_system(value: &str) -> CuResult<CoordinateSystem> {
        match value.to_ascii_uppercase().as_str() {
            "IMAGE" => Ok(CoordinateSystem::Image),
            "LEFT_HANDED_Y_UP" => Ok(CoordinateSystem::LeftHandedYUp),
            "RIGHT_HANDED_Y_UP" => Ok(CoordinateSystem::RightHandedYUp),
            "RIGHT_HANDED_Z_UP" => Ok(CoordinateSystem::RightHandedZUp),
            "LEFT_HANDED_Z_UP" => Ok(CoordinateSystem::LeftHandedZUp),
            "RIGHT_HANDED_Z_UP_X_FWD" | "RIGHT_HANDED_Z_UP_X_FORWARD" => {
                Ok(CoordinateSystem::RightHandedZUpXForward)
            }
            _ => Err(CuError::from(format!(
                "Invalid ZED coordinate_system: {value}"
            ))),
        }
    }

    fn parse_coordinate_unit(value: &str) -> CuResult<Unit> {
        match value.to_ascii_uppercase().as_str() {
            "MILLIMETER" | "MILLIMETERS" => Ok(Unit::Millimeter),
            "CENTIMETER" | "CENTIMETERS" => Ok(Unit::Centimeter),
            "METER" | "METERS" => Ok(Unit::Meter),
            "INCH" | "INCHES" => Ok(Unit::Inch),
            "FOOT" | "FEET" => Ok(Unit::Foot),
            _ => Err(CuError::from(format!(
                "Invalid ZED coordinate_unit: {value}"
            ))),
        }
    }

    fn parse_reference_frame(value: &str) -> CuResult<ReferenceFrame> {
        match value.to_ascii_uppercase().as_str() {
            "WORLD" => Ok(ReferenceFrame::World),
            "CAMERA" => Ok(ReferenceFrame::Camera),
            _ => Err(CuError::from(format!(
                "Invalid ZED reference_frame: {value}"
            ))),
        }
    }

    fn has_motion_sensors(config: &SensorsConfiguration) -> bool {
        config.accelerometer.is_available || config.gyroscope.is_available
    }

    fn rgba_format(mat: &Mat<Rgba8>) -> zed_sdk::Result<CuImageBufferFormat> {
        let view = mat.view()?;
        Ok(CuImageBufferFormat {
            width: view.width() as u32,
            height: view.height() as u32,
            stride: (view.stride_elems() * size_of::<Rgba8>()) as u32,
            pixel_format: *b"RGBA",
        })
    }

    fn raster_format(mat: &Mat<f32>) -> zed_sdk::Result<ZedRasterFormat> {
        let view = mat.view()?;
        Ok(ZedRasterFormat {
            width: view.width() as u32,
            height: view.height() as u32,
            stride: view.stride_elems() as u32,
        })
    }

    fn copy_image_payload(
        seq: u64,
        mat: &Mat<Rgba8>,
        pool: &Arc<CuHostMemoryPool<Vec<u8>>>,
        format: CuImageBufferFormat,
    ) -> CuResult<CuImage<Vec<u8>>> {
        let view = mat
            .view()
            .map_err(|e| CuError::new_with_cause("Could not map ZED image mat", e))?;
        let handle = pool
            .acquire()
            .ok_or_else(|| CuError::from("No free pooled buffer available for ZED image"))?;
        handle.with_inner_mut(|inner| inner.copy_from_slice(view.as_padded_bytes()));
        let mut image = CuImage::new(format, handle);
        image.seq = seq;
        Ok(image)
    }

    fn copy_raster_payload<P>(
        seq: u64,
        mat: &Mat<f32>,
        pool: &Arc<CuHostMemoryPool<Vec<f32>>>,
        format: ZedRasterFormat,
        ctor: impl FnOnce(ZedRasterFormat, CuHandle<Vec<f32>>) -> P,
    ) -> CuResult<P>
    where
        P: RasterSeq,
    {
        let view = mat
            .view()
            .map_err(|e| CuError::new_with_cause("Could not map ZED raster mat", e))?;
        let handle = pool
            .acquire()
            .ok_or_else(|| CuError::from("No free pooled buffer available for ZED raster"))?;
        handle.with_inner_mut(|inner| inner.copy_from_slice(view.as_padded_slice()));
        let mut payload = ctor(format, handle);
        payload.set_seq(seq);
        Ok(payload)
    }

    trait RasterSeq {
        fn set_seq(&mut self, seq: u64);
    }

    impl RasterSeq for ZedDepthMap<Vec<f32>> {
        fn set_seq(&mut self, seq: u64) {
            self.seq = seq;
        }
    }

    impl RasterSeq for ZedConfidenceMap<Vec<f32>> {
        fn set_seq(&mut self, seq: u64) {
            self.seq = seq;
        }
    }

    fn emit_latched<T: CuMsgPayload>(
        msg: &mut CuMsg<CuLatchedStateUpdate<T>>,
        tov: CuTime,
        pending: Option<T>,
    ) {
        msg.tov = Tov::Time(tov);
        match pending {
            Some(value) => msg.set_payload(CuLatchedStateUpdate::Set(value)),
            None => msg.set_payload(CuLatchedStateUpdate::NoChange),
        }
    }

    fn build_calibration_bundle(
        info: &CameraInformation,
        calibration: &CalibrationParameters,
        camera_imu: Option<&CameraImuTransform>,
    ) -> ZedCalibrationBundle {
        ZedCalibrationBundle {
            serial_number: info.serial_number,
            width: info.resolution.width(),
            height: info.resolution.height(),
            fps: info.fps,
            left: intrinsics_from_sdk(&calibration.left_cam),
            right: intrinsics_from_sdk(&calibration.right_cam),
            stereo_rotation_rodrigues: calibration.rotation_rodrigues,
            stereo_translation_m: [
                calibration.translation.x,
                calibration.translation.y,
                calibration.translation.z,
            ],
            camera_to_imu_translation_m: camera_imu.map(|transform| {
                [
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ]
            }),
            camera_to_imu_quaternion_xyzw: camera_imu.map(|transform| transform.rotation_xyzw),
        }
    }

    fn intrinsics_from_sdk(parameters: &CameraParameters) -> ZedCameraIntrinsics {
        ZedCameraIntrinsics {
            fx: parameters.fx,
            fy: parameters.fy,
            cx: parameters.cx,
            cy: parameters.cy,
            disto: parameters.disto,
            v_fov: parameters.v_fov,
            h_fov: parameters.h_fov,
            d_fov: parameters.d_fov,
            width: parameters.image_size.width(),
            height: parameters.image_size.height(),
            focal_length_metric: parameters.focal_length_metric,
        }
    }

    fn build_left_to_right_transform(
        frame_prefix: &str,
        calibration: &CalibrationParameters,
    ) -> FrameTransform<f32> {
        let transform = transform_from_rodrigues_translation(
            calibration.rotation_rodrigues,
            [
                calibration.translation.x,
                calibration.translation.y,
                calibration.translation.z,
            ],
        );
        FrameTransform::new(
            transform,
            left_frame_id(frame_prefix),
            right_frame_id(frame_prefix),
        )
    }

    fn build_camera_to_imu_transform(
        frame_prefix: &str,
        camera_imu: &CameraImuTransform,
    ) -> FrameTransform<f32> {
        let transform = transform_from_quaternion_translation(
            camera_imu.rotation_xyzw,
            [
                camera_imu.translation.x,
                camera_imu.translation.y,
                camera_imu.translation.z,
            ],
        );
        FrameTransform::new(
            transform,
            left_frame_id(frame_prefix),
            imu_frame_id(frame_prefix),
        )
    }

    fn left_frame_id(frame_prefix: &str) -> String {
        format!("{frame_prefix}_left")
    }

    fn right_frame_id(frame_prefix: &str) -> String {
        format!("{frame_prefix}_right")
    }

    fn imu_frame_id(frame_prefix: &str) -> String {
        format!("{frame_prefix}_imu")
    }

    fn build_frame_meta(seq: u64, camera: &Camera, sensors: Option<&SensorsData>) -> ZedFrameMeta {
        let temps = sensors.map(|data| data.temperature);
        ZedFrameMeta {
            seq,
            image_timestamp_ns: camera.image_timestamp(),
            current_timestamp_ns: camera.current_timestamp(),
            current_fps: camera.current_fps(),
            camera_moving_state: sensors.map(|data| data.camera_moving_state),
            image_sync_trigger: sensors.map(|data| data.image_sync_trigger),
            imu_temp_c: temps.map(|data| data.imu_temp_c),
            barometer_temp_c: temps.map(|data| data.barometer_temp_c),
            onboard_left_temp_c: temps.map(|data| data.onboard_left_temp_c),
            onboard_right_temp_c: temps.map(|data| data.onboard_right_temp_c),
        }
    }

    fn is_optional_sensor_error(err: &zed_sdk::Error) -> bool {
        matches!(
            err.sdk_code(),
            Some(
                ErrorCode::SensorsDataRequired
                    | ErrorCode::SensorsNotAvailable
                    | ErrorCode::SensorsNotInitialized
                    | ErrorCode::MotionSensorsRequired
            )
        )
    }

    fn transform_from_rodrigues_translation(
        rotation_rodrigues: [f32; 3],
        translation: [f32; 3],
    ) -> cu_transform::Transform3D<f32> {
        let theta = (rotation_rodrigues[0] * rotation_rodrigues[0]
            + rotation_rodrigues[1] * rotation_rodrigues[1]
            + rotation_rodrigues[2] * rotation_rodrigues[2])
            .sqrt();

        let mut matrix = [
            [1.0f32, 0.0, 0.0, translation[0]],
            [0.0, 1.0, 0.0, translation[1]],
            [0.0, 0.0, 1.0, translation[2]],
            [0.0, 0.0, 0.0, 1.0],
        ];

        if theta > 1.0e-6 {
            let x = rotation_rodrigues[0] / theta;
            let y = rotation_rodrigues[1] / theta;
            let z = rotation_rodrigues[2] / theta;
            let cos_theta = theta.cos();
            let sin_theta = theta.sin();
            let one_minus_cos = 1.0 - cos_theta;

            matrix[0][0] = cos_theta + x * x * one_minus_cos;
            matrix[0][1] = x * y * one_minus_cos - z * sin_theta;
            matrix[0][2] = x * z * one_minus_cos + y * sin_theta;

            matrix[1][0] = y * x * one_minus_cos + z * sin_theta;
            matrix[1][1] = cos_theta + y * y * one_minus_cos;
            matrix[1][2] = y * z * one_minus_cos - x * sin_theta;

            matrix[2][0] = z * x * one_minus_cos - y * sin_theta;
            matrix[2][1] = z * y * one_minus_cos + x * sin_theta;
            matrix[2][2] = cos_theta + z * z * one_minus_cos;
        }

        cu_transform::Transform3D::from_matrix(matrix)
    }

    fn transform_from_quaternion_translation(
        rotation_xyzw: [f32; 4],
        translation: [f32; 3],
    ) -> cu_transform::Transform3D<f32> {
        let [x, y, z, w] = rotation_xyzw;
        let norm = (x * x + y * y + z * z + w * w).sqrt();
        let (x, y, z, w) = if norm > 1.0e-6 {
            (x / norm, y / norm, z / norm, w / norm)
        } else {
            (0.0, 0.0, 0.0, 1.0)
        };

        cu_transform::Transform3D::from_matrix([
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
                translation[0],
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
                translation[1],
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
                translation[2],
            ],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }
}

#[cfg(target_os = "linux")]
pub use linux_impl::Zed;
