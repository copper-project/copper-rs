mod payloads;

pub use payloads::*;

use cu_sensor_payloads::{
    BarometerPayload, CuImage, Distance, ImuPayload, MagnetometerPayload, PointCloudSoa,
    Reflectivity,
};
use cu29::prelude::*;
use cu29::units::si::length::meter;
use cu29::units::si::ratio::percent;

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

/// Standard dense point cloud capacity for the default ZED `HD720` resolution.
pub type ZedPointCloudHd720 = PointCloudSoa<{ 1280 * 720 }>;

/// Pure projection task sized for the default ZED `HD720` resolution.
pub type ZedDepthToPointCloudHd720 = ZedDepthToPointCloud<{ 1280 * 720 }>;

/// Projects a `ZedDepthMap` into a standard `PointCloudSoa`.
///
/// The task consumes the latest latched calibration bundle and currently supports
/// `IMAGE` and `LEFT_HANDED_Y_UP` ZED coordinate systems. Use a capacity large
/// enough for the expected number of valid depth samples. The `Hd720` alias
/// matches the source crate's default resolution.
#[derive(Default, Reflect)]
#[reflect(from_reflect = false)]
pub struct ZedDepthToPointCloud<const MAX_POINTS: usize> {
    calibration: CuLatchedState<ZedCalibrationBundle>,
}

impl<const MAX_POINTS: usize> Freezable for ZedDepthToPointCloud<MAX_POINTS> {}

impl<const MAX_POINTS: usize> CuTask for ZedDepthToPointCloud<MAX_POINTS> {
    type Resources<'r> = ();
    type Input<'m> =
        input_msg!('m, ZedDepthMap<Vec<f32>>, CuLatchedStateUpdate<ZedCalibrationBundle>);
    type Output<'m> = output_msg!(PointCloudSoa<MAX_POINTS>);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self::default())
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let (depth_msg, calibration_msg) = *input;
        self.apply_calibration_update(calibration_msg.payload());

        let Some(depth) = depth_msg.payload() else {
            output.clear_payload();
            output.tov = depth_msg.tov;
            output.metadata.set_status("no depth");
            return Ok(());
        };

        let Some(calibration) = self.calibration.as_ref() else {
            output.clear_payload();
            output.tov = depth_msg.tov;
            output.metadata.set_status("no calib");
            return Ok(());
        };

        let projection = ProjectionIntrinsics::from_bundle(depth.format, calibration)?;
        let point_tov = representative_tov(depth_msg.tov);
        let projected_points = {
            let pointcloud = output.payload_mut().get_or_insert_with(Default::default);
            pointcloud.len = 0;

            depth.buffer_handle.with_inner(|samples| {
                for row in 0..depth.format.height as usize {
                    let row_offset = row * depth.format.stride as usize;
                    for col in 0..depth.format.width as usize {
                        let depth_value = samples[row_offset + col];
                        if !depth_value.is_finite() || depth_value <= 0.0 {
                            continue;
                        }

                        if pointcloud.len == MAX_POINTS {
                            return Err(CuError::from(format!(
                                "ZED point cloud capacity {MAX_POINTS} exceeded while projecting {}x{} depth map",
                                depth.format.width, depth.format.height
                            )));
                        }

                        let (x, y, z) = projection.project(col as f32, row as f32, depth_value)?;
                        let idx = pointcloud.len;
                        pointcloud.tov[idx] = point_tov;
                        pointcloud.x[idx] = Distance::new::<meter>(x);
                        pointcloud.y[idx] = Distance::new::<meter>(y);
                        pointcloud.z[idx] = Distance::new::<meter>(z);
                        pointcloud.i[idx] = Reflectivity::new::<percent>(0.0);
                        pointcloud.return_order[idx] = 0;
                        pointcloud.len += 1;
                    }
                }

                Ok(pointcloud.len)
            })?
        };

        output.tov = depth_msg.tov;
        output.metadata.set_status(projected_points);
        Ok(())
    }
}

impl<const MAX_POINTS: usize> ZedDepthToPointCloud<MAX_POINTS> {
    fn apply_calibration_update(
        &mut self,
        update: Option<&CuLatchedStateUpdate<ZedCalibrationBundle>>,
    ) {
        match update {
            Some(CuLatchedStateUpdate::Set(bundle)) => {
                self.calibration = CuLatchedState::Set(bundle.clone());
            }
            Some(CuLatchedStateUpdate::Clear) => {
                self.calibration = CuLatchedState::Unset;
            }
            Some(CuLatchedStateUpdate::NoChange) | None => {}
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct ProjectionIntrinsics {
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
    y_sign: f32,
    depth_scale_m: f32,
}

impl ProjectionIntrinsics {
    fn from_bundle(format: ZedRasterFormat, calibration: &ZedCalibrationBundle) -> CuResult<Self> {
        if calibration.left.width == 0 || calibration.left.height == 0 {
            return Err(CuError::from("ZED calibration reported a zero-sized image"));
        }
        if calibration.left.fx.abs() <= f32::EPSILON || calibration.left.fy.abs() <= f32::EPSILON {
            return Err(CuError::from("ZED calibration reported zero focal length"));
        }

        let scale_x = format.width as f32 / calibration.left.width as f32;
        let scale_y = format.height as f32 / calibration.left.height as f32;
        let y_sign = match calibration.coordinate_system {
            ZedCoordinateSystem::Image => 1.0,
            ZedCoordinateSystem::LeftHandedYUp => -1.0,
            other => {
                return Err(CuError::from(format!(
                    "ZedDepthToPointCloud only supports IMAGE and LEFT_HANDED_Y_UP depth projections, got {other:?}"
                )));
            }
        };

        Ok(Self {
            fx: calibration.left.fx * scale_x,
            fy: calibration.left.fy * scale_y,
            cx: calibration.left.cx * scale_x,
            cy: calibration.left.cy * scale_y,
            y_sign,
            depth_scale_m: depth_unit_scale_m(calibration.coordinate_unit),
        })
    }

    fn project(&self, pixel_x: f32, pixel_y: f32, raw_depth: f32) -> CuResult<(f32, f32, f32)> {
        let z = raw_depth * self.depth_scale_m;
        let x = (pixel_x - self.cx) * z / self.fx;
        let y = self.y_sign * (pixel_y - self.cy) * z / self.fy;
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return Err(CuError::from(
                "ZED depth projection produced a non-finite point",
            ));
        }
        Ok((x, y, z))
    }
}

fn depth_unit_scale_m(unit: ZedCoordinateUnit) -> f32 {
    match unit {
        ZedCoordinateUnit::Millimeter => 1.0e-3,
        ZedCoordinateUnit::Centimeter => 1.0e-2,
        ZedCoordinateUnit::Meter => 1.0,
        ZedCoordinateUnit::Inch => 0.0254,
        ZedCoordinateUnit::Foot => 0.3048,
    }
}

fn representative_tov(tov: Tov) -> CuTime {
    match tov {
        Tov::Time(time) => time,
        Tov::Range(range) => range.start,
        Tov::None => CuTime::default(),
    }
}

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
    use cu_sensor_payloads::CuImageBufferFormat;
    use cu_transform::FrameTransform;
    use std::path::PathBuf;
    use std::sync::Arc;
    use zed_sdk::{
        CalibrationParameters, Camera, CameraImuTransform, CameraInformation, CameraParameters,
        CoordinateSystem, DepthMode, ErrorCode, InputSource, OpenOptions, ReferenceFrame,
        Resolution, ResolutionPreset, RuntimeParameters, SensorsConfiguration, SensorsData, Unit,
    };
    use zed_sdk::{Mat, Rgba8};

    struct ImageSlot {
        handle: CuHandle<Vec<u8>>,
        mat: Mat<Rgba8>,
    }

    struct RasterSlot {
        handle: CuHandle<Vec<f32>>,
        mat: Mat<f32>,
    }

    struct OutputSlot {
        left: ImageSlot,
        right: ImageSlot,
        depth: RasterSlot,
        confidence: Option<RasterSlot>,
    }

    impl OutputSlot {
        fn is_available(&self) -> bool {
            handle_is_available(&self.left.handle)
                && handle_is_available(&self.right.handle)
                && handle_is_available(&self.depth.handle)
                && self
                    .confidence
                    .as_ref()
                    .is_none_or(|slot| handle_is_available(&slot.handle))
        }
    }

    #[derive(Reflect)]
    #[reflect(from_reflect = false)]
    pub struct Zed {
        #[reflect(ignore)]
        camera: Camera,
        #[reflect(ignore)]
        runtime: RuntimeParameters,
        #[reflect(ignore)]
        slots: Vec<OutputSlot>,
        next_slot: usize,
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
            let coordinate_system = config_zed_coordinate_system(config)?;
            let coordinate_unit = config_zed_coordinate_unit(config)?;
            let open = build_open_options(config, coordinate_system, coordinate_unit)?;
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

            let left_format = rgba_format(resolution);
            let right_format = rgba_format(resolution);
            let depth_format = raster_format(resolution);
            let confidence_format = emit_confidence.then_some(depth_format);
            let slots = build_output_slots(
                pool_slots,
                resolution,
                left_format,
                right_format,
                depth_format,
                confidence_format,
            )?;

            let pending_calibration = Some(build_calibration_bundle(
                &info,
                &calibration,
                camera_imu.as_ref(),
                coordinate_system,
                coordinate_unit,
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
                slots,
                next_slot: 0,
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
            let slot_index = acquire_output_slot_index(&self.slots, &mut self.next_slot)
                .ok_or_else(|| {
                    CuError::from(
                        "No reusable ZED output slot available; increase pool_slots or release downstream handles sooner",
                    )
                })?;
            let slot = &mut self.slots[slot_index];

            self.camera
                .retrieve_left(&mut slot.left.mat)
                .map_err(|e| CuError::new_with_cause("Could not retrieve ZED left image", e))?;
            self.camera
                .retrieve_right(&mut slot.right.mat)
                .map_err(|e| CuError::new_with_cause("Could not retrieve ZED right image", e))?;
            self.camera
                .retrieve_depth(&mut slot.depth.mat)
                .map_err(|e| CuError::new_with_cause("Could not retrieve ZED depth map", e))?;

            stereo.set_payload((
                image_payload_from_handle(seq, &slot.left.handle, self.left_format),
                image_payload_from_handle(seq, &slot.right.handle, self.right_format),
            ));
            stereo.tov = Tov::Time(frame_tov);

            depth.set_payload(raster_payload_from_handle(
                seq,
                &slot.depth.handle,
                self.depth_format,
                ZedDepthMap::new,
            ));
            depth.tov = Tov::Time(frame_tov);

            if self.emit_confidence {
                let confidence_slot = slot.confidence.as_mut().ok_or_else(|| {
                    CuError::from("confidence output requested without a backing mat")
                })?;
                self.camera
                    .retrieve_confidence(&mut confidence_slot.mat)
                    .map_err(|e| {
                        CuError::new_with_cause("Could not retrieve ZED confidence map", e)
                    })?;
                let confidence_format = self
                    .confidence_format
                    .ok_or_else(|| CuError::from("confidence output requested without a format"))?;
                confidence.set_payload(raster_payload_from_handle(
                    seq,
                    &confidence_slot.handle,
                    confidence_format,
                    ZedConfidenceMap::new,
                ));
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

    fn build_open_options(
        config: Option<&ComponentConfig>,
        coordinate_system: ZedCoordinateSystem,
        coordinate_unit: ZedCoordinateUnit,
    ) -> CuResult<OpenOptions> {
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
        options = options.coordinate_system(to_sdk_coordinate_system(coordinate_system));
        options = options.coordinate_unit(to_sdk_coordinate_unit(coordinate_unit));

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

    fn config_zed_coordinate_system(
        config: Option<&ComponentConfig>,
    ) -> CuResult<ZedCoordinateSystem> {
        match config_string_opt(config, "coordinate_system")? {
            Some(value) => parse_zed_coordinate_system(&value),
            None => Ok(ZedCoordinateSystem::default()),
        }
    }

    fn config_zed_coordinate_unit(config: Option<&ComponentConfig>) -> CuResult<ZedCoordinateUnit> {
        match config_string_opt(config, "coordinate_unit")? {
            Some(value) => parse_zed_coordinate_unit(&value),
            None => Ok(ZedCoordinateUnit::default()),
        }
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

    fn parse_zed_coordinate_system(value: &str) -> CuResult<ZedCoordinateSystem> {
        match value.to_ascii_uppercase().as_str() {
            "IMAGE" => Ok(ZedCoordinateSystem::Image),
            "LEFT_HANDED_Y_UP" => Ok(ZedCoordinateSystem::LeftHandedYUp),
            "RIGHT_HANDED_Y_UP" => Ok(ZedCoordinateSystem::RightHandedYUp),
            "RIGHT_HANDED_Z_UP" => Ok(ZedCoordinateSystem::RightHandedZUp),
            "LEFT_HANDED_Z_UP" => Ok(ZedCoordinateSystem::LeftHandedZUp),
            "RIGHT_HANDED_Z_UP_X_FWD" | "RIGHT_HANDED_Z_UP_X_FORWARD" => {
                Ok(ZedCoordinateSystem::RightHandedZUpXForward)
            }
            _ => Err(CuError::from(format!(
                "Invalid ZED coordinate_system: {value}"
            ))),
        }
    }

    fn parse_zed_coordinate_unit(value: &str) -> CuResult<ZedCoordinateUnit> {
        match value.to_ascii_uppercase().as_str() {
            "MILLIMETER" | "MILLIMETERS" => Ok(ZedCoordinateUnit::Millimeter),
            "CENTIMETER" | "CENTIMETERS" => Ok(ZedCoordinateUnit::Centimeter),
            "METER" | "METERS" => Ok(ZedCoordinateUnit::Meter),
            "INCH" | "INCHES" => Ok(ZedCoordinateUnit::Inch),
            "FOOT" | "FEET" => Ok(ZedCoordinateUnit::Foot),
            _ => Err(CuError::from(format!(
                "Invalid ZED coordinate_unit: {value}"
            ))),
        }
    }

    fn to_sdk_coordinate_system(value: ZedCoordinateSystem) -> CoordinateSystem {
        match value {
            ZedCoordinateSystem::Image => CoordinateSystem::Image,
            ZedCoordinateSystem::LeftHandedYUp => CoordinateSystem::LeftHandedYUp,
            ZedCoordinateSystem::RightHandedYUp => CoordinateSystem::RightHandedYUp,
            ZedCoordinateSystem::RightHandedZUp => CoordinateSystem::RightHandedZUp,
            ZedCoordinateSystem::LeftHandedZUp => CoordinateSystem::LeftHandedZUp,
            ZedCoordinateSystem::RightHandedZUpXForward => CoordinateSystem::RightHandedZUpXForward,
        }
    }

    fn to_sdk_coordinate_unit(value: ZedCoordinateUnit) -> Unit {
        match value {
            ZedCoordinateUnit::Millimeter => Unit::Millimeter,
            ZedCoordinateUnit::Centimeter => Unit::Centimeter,
            ZedCoordinateUnit::Meter => Unit::Meter,
            ZedCoordinateUnit::Inch => Unit::Inch,
            ZedCoordinateUnit::Foot => Unit::Foot,
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

    fn rgba_format(resolution: Resolution) -> CuImageBufferFormat {
        CuImageBufferFormat {
            width: resolution.width(),
            height: resolution.height(),
            stride: (resolution.width() as usize * size_of::<Rgba8>()) as u32,
            pixel_format: *b"RGBA",
        }
    }

    fn raster_format(resolution: Resolution) -> ZedRasterFormat {
        ZedRasterFormat {
            width: resolution.width(),
            height: resolution.height(),
            stride: resolution.width(),
        }
    }

    fn build_output_slots(
        slot_count: usize,
        resolution: Resolution,
        left_format: CuImageBufferFormat,
        right_format: CuImageBufferFormat,
        depth_format: ZedRasterFormat,
        confidence_format: Option<ZedRasterFormat>,
    ) -> CuResult<Vec<OutputSlot>> {
        (0..slot_count)
            .map(|_| {
                Ok(OutputSlot {
                    left: build_image_slot(resolution, left_format)?,
                    right: build_image_slot(resolution, right_format)?,
                    depth: build_raster_slot(resolution, depth_format)?,
                    confidence: confidence_format
                        .map(|format| build_raster_slot(resolution, format))
                        .transpose()?,
                })
            })
            .collect()
    }

    fn build_image_slot(
        resolution: Resolution,
        format: CuImageBufferFormat,
    ) -> CuResult<ImageSlot> {
        let handle = CuHandle::new_detached(vec![0u8; format.byte_size()]);
        let stride_bytes = format.stride as usize;
        let stride_elems = stride_bytes / size_of::<Rgba8>();
        if stride_elems * size_of::<Rgba8>() != stride_bytes {
            return Err(CuError::from(
                "ZED image stride is not aligned to the RGBA pixel size",
            ));
        }
        let (ptr, len_bytes) = handle.with_inner_mut(|inner| (inner.as_mut_ptr(), inner.len()));
        if len_bytes % size_of::<Rgba8>() != 0 {
            return Err(CuError::from(
                "ZED image buffer length is not aligned to the RGBA pixel size",
            ));
        }

        let mat = unsafe {
            Mat::from_external_cpu_buffer(
                resolution,
                stride_elems,
                len_bytes / size_of::<Rgba8>(),
                ptr.cast::<Rgba8>(),
            )
        }
        .map_err(|e| CuError::new_with_cause("Could not alias ZED image buffer as sl::Mat", e))?;

        Ok(ImageSlot { handle, mat })
    }

    fn build_raster_slot(resolution: Resolution, format: ZedRasterFormat) -> CuResult<RasterSlot> {
        let handle = CuHandle::new_detached(vec![0f32; format.len_elements()]);
        let (ptr, len_elements) = handle.with_inner_mut(|inner| (inner.as_mut_ptr(), inner.len()));
        let mat = unsafe {
            Mat::from_external_cpu_buffer(resolution, format.stride as usize, len_elements, ptr)
        }
        .map_err(|e| CuError::new_with_cause("Could not alias ZED raster buffer as sl::Mat", e))?;

        Ok(RasterSlot { handle, mat })
    }

    fn handle_is_available<T>(handle: &CuHandle<T>) -> bool
    where
        T: ArrayLike,
    {
        Arc::strong_count(handle) == 1
    }

    fn acquire_output_slot_index(slots: &[OutputSlot], next_slot: &mut usize) -> Option<usize> {
        if slots.is_empty() {
            return None;
        }

        for offset in 0..slots.len() {
            let index = (*next_slot + offset) % slots.len();
            if slots[index].is_available() {
                *next_slot = (index + 1) % slots.len();
                return Some(index);
            }
        }

        None
    }

    fn image_payload_from_handle(
        seq: u64,
        handle: &CuHandle<Vec<u8>>,
        format: CuImageBufferFormat,
    ) -> CuImage<Vec<u8>> {
        let mut image = CuImage::new(format, handle.clone());
        image.seq = seq;
        image
    }

    fn raster_payload_from_handle<P>(
        seq: u64,
        handle: &CuHandle<Vec<f32>>,
        format: ZedRasterFormat,
        ctor: impl FnOnce(ZedRasterFormat, CuHandle<Vec<f32>>) -> P,
    ) -> P
    where
        P: RasterSeq,
    {
        let mut payload = ctor(format, handle.clone());
        payload.set_seq(seq);
        payload
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
        coordinate_system: ZedCoordinateSystem,
        coordinate_unit: ZedCoordinateUnit,
    ) -> ZedCalibrationBundle {
        ZedCalibrationBundle {
            serial_number: info.serial_number,
            width: info.resolution.width(),
            height: info.resolution.height(),
            fps: info.fps,
            coordinate_system,
            coordinate_unit,
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

#[cfg(test)]
mod tests {
    use super::*;

    fn calibration_bundle(
        coordinate_system: ZedCoordinateSystem,
        width: u32,
        height: u32,
    ) -> ZedCalibrationBundle {
        ZedCalibrationBundle {
            serial_number: 1,
            width,
            height,
            fps: 30.0,
            coordinate_system,
            coordinate_unit: ZedCoordinateUnit::Meter,
            left: ZedCameraIntrinsics {
                fx: width as f32,
                fy: height as f32,
                cx: width as f32 / 2.0,
                cy: height as f32 / 2.0,
                width,
                height,
                ..Default::default()
            },
            right: ZedCameraIntrinsics {
                width,
                height,
                ..Default::default()
            },
            ..Default::default()
        }
    }

    fn depth_msg(width: u32, height: u32, values: Vec<f32>) -> CuMsg<ZedDepthMap<Vec<f32>>> {
        let mut msg = CuMsg::new(Some(ZedDepthMap::new(
            ZedRasterFormat {
                width,
                height,
                stride: width,
            },
            CuHandle::new_detached(values),
        )));
        msg.tov = Tov::from(CuDuration(42));
        msg
    }

    #[test]
    fn depth_to_pointcloud_projects_image_coordinates() {
        let ctx = CuContext::new_with_clock();
        let mut task = ZedDepthToPointCloud::<4>::new(None, ()).expect("task");
        let depth = depth_msg(2, 2, vec![2.0, 2.0, 2.0, 2.0]);
        let calibration = CuMsg::new(Some(CuLatchedStateUpdate::Set(calibration_bundle(
            ZedCoordinateSystem::Image,
            2,
            2,
        ))));
        let input = (&depth, &calibration);
        let mut output: <ZedDepthToPointCloud<4> as CuTask>::Output<'_> = Default::default();

        task.process(&ctx, &input, &mut output).expect("process");

        let payload = output.payload().expect("payload");
        assert_eq!(payload.len, 4);
        assert_eq!(payload.x[0].value, -1.0);
        assert_eq!(payload.y[0].value, -1.0);
        assert_eq!(payload.z[0].value, 2.0);
        assert_eq!(payload.x[3].value, 0.0);
        assert_eq!(payload.y[3].value, 0.0);
        assert_eq!(payload.z[3].value, 2.0);
    }

    #[test]
    fn depth_to_pointcloud_flips_y_for_left_handed_y_up() {
        let ctx = CuContext::new_with_clock();
        let mut task = ZedDepthToPointCloud::<4>::new(None, ()).expect("task");
        let depth = depth_msg(2, 2, vec![1.0, 1.0, 1.0, 1.0]);
        let calibration = CuMsg::new(Some(CuLatchedStateUpdate::Set(calibration_bundle(
            ZedCoordinateSystem::LeftHandedYUp,
            2,
            2,
        ))));
        let input = (&depth, &calibration);
        let mut output: <ZedDepthToPointCloud<4> as CuTask>::Output<'_> = Default::default();

        task.process(&ctx, &input, &mut output).expect("process");

        let payload = output.payload().expect("payload");
        assert_eq!(payload.len, 4);
        assert_eq!(payload.y[0].value, 0.5);
        assert_eq!(payload.y[2].value, -0.0);
    }

    #[test]
    fn depth_to_pointcloud_scales_intrinsics_to_raster_size() {
        let ctx = CuContext::new_with_clock();
        let mut task = ZedDepthToPointCloud::<4>::new(None, ()).expect("task");
        let depth = depth_msg(2, 2, vec![4.0, 4.0, 4.0, 4.0]);
        let calibration = CuMsg::new(Some(CuLatchedStateUpdate::Set(calibration_bundle(
            ZedCoordinateSystem::Image,
            4,
            4,
        ))));
        let input = (&depth, &calibration);
        let mut output: <ZedDepthToPointCloud<4> as CuTask>::Output<'_> = Default::default();

        task.process(&ctx, &input, &mut output).expect("process");

        let payload = output.payload().expect("payload");
        assert_eq!(payload.len, 4);
        assert_eq!(payload.x[3].value, 0.0);
        assert_eq!(payload.y[3].value, 0.0);
    }

    #[test]
    fn depth_to_pointcloud_skips_invalid_depth_samples() {
        let ctx = CuContext::new_with_clock();
        let mut task = ZedDepthToPointCloud::<4>::new(None, ()).expect("task");
        let depth = depth_msg(2, 2, vec![1.0, 0.0, f32::NAN, -1.0]);
        let calibration = CuMsg::new(Some(CuLatchedStateUpdate::Set(calibration_bundle(
            ZedCoordinateSystem::Image,
            2,
            2,
        ))));
        let input = (&depth, &calibration);
        let mut output: <ZedDepthToPointCloud<4> as CuTask>::Output<'_> = Default::default();

        task.process(&ctx, &input, &mut output).expect("process");

        let payload = output.payload().expect("payload");
        assert_eq!(payload.len, 1);
        assert_eq!(payload.z[0].value, 1.0);
    }
}
