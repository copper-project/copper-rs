use cu_sensor_payloads::{BarometerPayload, ImuPayload, MagnetometerPayload};
use cu_zed::{
    ZedCalibrationBundle, ZedConfidenceMap, ZedDepthMap, ZedFrameMeta, ZedPointCloudHd720,
    ZedRigTransforms, ZedStereoImages,
};
use cu29::clock::Tov;
use cu29::prelude::*;
use cu29_logviz::{apply_tov, log_as_components, log_fallback_payload, log_imu, log_pointcloud};
use rerun::{Arrows3D, DepthImage, Image, RecordingStream, RecordingStreamBuilder};
use std::path::PathBuf;

#[copper_runtime(config = "examples/zed_rerun_demo.ron")]
struct ZedRerunDemoApp {}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
struct ZedRerunSink {
    #[reflect(ignore)]
    rec: RecordingStream,
}

impl Freezable for ZedRerunSink {}

impl CuSinkTask for ZedRerunSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(
        'm,
        ZedStereoImages,
        ZedDepthMap<Vec<f32>>,
        ZedConfidenceMap<Vec<f32>>,
        CuLatchedStateUpdate<ZedCalibrationBundle>,
        CuLatchedStateUpdate<ZedRigTransforms>,
        ZedPointCloudHd720,
        ImuPayload,
        MagnetometerPayload,
        BarometerPayload,
        ZedFrameMeta
    );

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let rec = RecordingStreamBuilder::new("cu-zed-rerun-demo")
            .spawn()
            .map_err(|e| CuError::new_with_cause("Failed to spawn Rerun stream", e))?;

        rec.log_static("zed", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP())
            .map_err(|e| CuError::new_with_cause("Failed to log world coordinates", e))?;
        log_axes(&rec, "zed", 0.15)?;

        Ok(Self { rec })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let (
            stereo_msg,
            depth_msg,
            confidence_msg,
            calibration_msg,
            transforms_msg,
            pointcloud_msg,
            imu_msg,
            mag_msg,
            baro_msg,
            meta_msg,
        ) = *input;

        self.update_calibration(calibration_msg)?;
        self.update_transforms(transforms_msg)?;

        if let Some((left, right)) = stereo_msg.payload() {
            apply_tov(&self.rec, &stereo_msg.tov);
            log_rgba_image(&self.rec, "zed/left_camera/image", left)?;
            log_rgba_image(&self.rec, "zed/right_camera/image", right)?;
        }

        if let Some(depth) = depth_msg.payload() {
            apply_tov(&self.rec, &depth_msg.tov);
            log_depth_map(&self.rec, "zed/left_camera/depth", depth)?;
        }

        if let Some(confidence) = confidence_msg.payload() {
            apply_tov(&self.rec, &confidence_msg.tov);
            log_depth_map(&self.rec, "zed/left_camera/confidence", confidence)?;
        }

        if let Some(pointcloud) = pointcloud_msg.payload() {
            apply_tov(&self.rec, &pointcloud_msg.tov);
            log_pointcloud(&self.rec, "zed/pointcloud", pointcloud)?;
        }

        if let Some(imu) = imu_msg.payload() {
            apply_tov(&self.rec, &imu_msg.tov);
            log_imu(&self.rec, "zed/imu", imu)?;
        }

        if let Some(mag) = mag_msg.payload() {
            apply_tov(&self.rec, &mag_msg.tov);
            log_fallback_payload(&self.rec, "zed/imu/magnetometer", mag)?;
        }

        if let Some(baro) = baro_msg.payload() {
            apply_tov(&self.rec, &baro_msg.tov);
            log_fallback_payload(&self.rec, "zed/barometer", baro)?;
        }

        if let Some(meta) = meta_msg.payload() {
            apply_tov(&self.rec, &meta_msg.tov);
            log_fallback_payload(&self.rec, "zed/meta", meta)?;
        }

        Ok(())
    }
}

impl ZedRerunSink {
    fn update_calibration(
        &mut self,
        calibration_msg: &CuMsg<CuLatchedStateUpdate<ZedCalibrationBundle>>,
    ) -> CuResult<()> {
        match calibration_msg.payload() {
            Some(CuLatchedStateUpdate::Set(bundle)) => {
                self.log_calibration(bundle)?;
            }
            Some(CuLatchedStateUpdate::Clear) => {}
            Some(CuLatchedStateUpdate::NoChange) | None => {}
        }
        Ok(())
    }

    fn update_transforms(
        &mut self,
        transforms_msg: &CuMsg<CuLatchedStateUpdate<ZedRigTransforms>>,
    ) -> CuResult<()> {
        match transforms_msg.payload() {
            Some(CuLatchedStateUpdate::Set(transforms)) => {
                self.log_rig_transforms(transforms, &transforms_msg.tov)?;
            }
            Some(CuLatchedStateUpdate::Clear) => {}
            Some(CuLatchedStateUpdate::NoChange) | None => {}
        }
        Ok(())
    }

    fn log_calibration(&self, calibration: &ZedCalibrationBundle) -> CuResult<()> {
        log_fallback_payload(&self.rec, "zed/calibration", calibration)?;

        let left_pinhole = pinhole_from_intrinsics(&calibration.left)
            .with_camera_xyz(rerun::components::ViewCoordinates::RDF);
        self.rec
            .log_static("zed/left_camera", &left_pinhole)
            .map_err(|e| CuError::new_with_cause("Failed to log left pinhole", e))?;

        let right_pinhole = pinhole_from_intrinsics(&calibration.right)
            .with_camera_xyz(rerun::components::ViewCoordinates::RDF);
        self.rec
            .log_static("zed/right_camera", &right_pinhole)
            .map_err(|e| CuError::new_with_cause("Failed to log right pinhole", e))?;

        Ok(())
    }

    fn log_rig_transforms(&self, transforms: &ZedRigTransforms, tov: &Tov) -> CuResult<()> {
        apply_tov(&self.rec, tov);
        log_as_components(
            &self.rec,
            "zed/right_camera",
            &transforms.left_to_right.to_transform3d(),
        )?;
        log_fallback_payload(
            &self.rec,
            "zed/transforms/left_to_right",
            &transforms.left_to_right,
        )?;
        log_axes(&self.rec, "zed/right_camera", 0.08)?;

        if transforms.has_camera_to_imu {
            log_as_components(
                &self.rec,
                "zed/imu",
                &transforms.camera_to_imu.to_transform3d(),
            )?;
            log_fallback_payload(
                &self.rec,
                "zed/transforms/camera_to_imu",
                &transforms.camera_to_imu,
            )?;
            log_axes(&self.rec, "zed/imu", 0.05)?;
        }

        Ok(())
    }
}

fn pinhole_from_intrinsics(intrinsics: &cu_zed::ZedCameraIntrinsics) -> rerun::Pinhole {
    let projection = rerun::components::PinholeProjection::from_focal_length_and_principal_point(
        [intrinsics.fx, intrinsics.fy],
        [intrinsics.cx, intrinsics.cy],
    );
    rerun::Pinhole::new(projection)
        .with_resolution([intrinsics.width as f32, intrinsics.height as f32])
}

fn log_axes(rec: &RecordingStream, path: &str, axis_len: f32) -> CuResult<()> {
    let axes = Arrows3D::from_vectors([
        [axis_len, 0.0, 0.0],
        [0.0, axis_len, 0.0],
        [0.0, 0.0, axis_len],
    ])
    .with_colors([
        rerun::Color::from([255, 0, 0]),
        rerun::Color::from([0, 255, 0]),
        rerun::Color::from([0, 0, 255]),
    ]);

    rec.log_static(format!("{path}/axes"), &axes)
        .map_err(|e| CuError::new_with_cause("Failed to log axes", e))
}

fn log_depth_map<T>(rec: &RecordingStream, path: &str, payload: &T) -> CuResult<()>
where
    T: ZedFloatRaster,
{
    let bytes = payload.packed_bytes();
    let image = DepthImage::from_data_type_and_bytes(
        bytes,
        [payload.width(), payload.height()],
        rerun::ChannelDatatype::F32,
    )
    .with_meter(1.0)
    .with_colormap(rerun::components::Colormap::Turbo);

    rec.log(path, &image)
        .map_err(|e| CuError::new_with_cause("Failed to log depth image", e))
}

trait ZedFloatRaster {
    fn width(&self) -> u32;
    fn height(&self) -> u32;
    fn packed_bytes(&self) -> Vec<u8>;
}

impl ZedFloatRaster for ZedDepthMap<Vec<f32>> {
    fn width(&self) -> u32 {
        self.format.width
    }

    fn height(&self) -> u32 {
        self.format.height
    }

    fn packed_bytes(&self) -> Vec<u8> {
        self.buffer_handle.with_inner(|inner| {
            pack_f32_raster_bytes(
                &inner[..],
                self.format.width,
                self.format.height,
                self.format.stride,
            )
        })
    }
}

impl ZedFloatRaster for ZedConfidenceMap<Vec<f32>> {
    fn width(&self) -> u32 {
        self.format.width
    }

    fn height(&self) -> u32 {
        self.format.height
    }

    fn packed_bytes(&self) -> Vec<u8> {
        self.buffer_handle.with_inner(|inner| {
            pack_f32_raster_bytes(
                &inner[..],
                self.format.width,
                self.format.height,
                self.format.stride,
            )
        })
    }
}

fn log_rgba_image(
    rec: &RecordingStream,
    path: &str,
    image: &cu_sensor_payloads::CuImage<Vec<u8>>,
) -> CuResult<()> {
    let bytes = image.buffer_handle.with_inner(|inner| {
        if image.format.stride as usize == image.format.width as usize * 4 {
            inner.to_vec()
        } else {
            let row_bytes = image.format.width as usize * 4;
            let mut packed = Vec::with_capacity(row_bytes * image.format.height as usize);
            for row in 0..image.format.height as usize {
                let start = row * image.format.stride as usize;
                let end = start + row_bytes;
                packed.extend_from_slice(&inner[start..end]);
            }
            packed
        }
    });

    let image = Image::from_rgba32(bytes, [image.format.width, image.format.height]);
    rec.log(path, &image)
        .map_err(|e| CuError::new_with_cause("Failed to log RGBA image", e))
}

fn pack_f32_raster_bytes(values: &[f32], width: u32, height: u32, stride: u32) -> Vec<u8> {
    let width = width as usize;
    let height = height as usize;
    let stride = stride as usize;

    if stride == width {
        return bytemuck::cast_slice(&values[..width * height]).to_vec();
    }

    let mut packed_values = Vec::with_capacity(width * height);
    for row in 0..height {
        let start = row * stride;
        let end = start + width;
        let slice = &values[start..end];
        debug_assert_eq!(slice.len(), width);
        packed_values.extend_from_slice(slice);
    }
    bytemuck::cast_slice(packed_values.as_slice()).to_vec()
}

fn main() {
    let log_path = PathBuf::from("logs/zed_rerun_demo.copper");
    if let Some(parent) = log_path.parent() {
        std::fs::create_dir_all(parent).expect("Failed to create log directory");
    }

    let mut application = ZedRerunDemoApp::builder()
        .with_log_path(&log_path, Some(64 * 1024 * 1024))
        .expect("Failed to setup copper")
        .build()
        .expect("Failed to create application");
    application
        .start_all_tasks()
        .expect("Failed to start tasks");

    println!("Streaming cu_zed outputs to Rerun.");
    println!("Copper log: {}", log_path.display());

    loop {
        application
            .run_one_iteration()
            .expect("Failed to run ZED rerun demo iteration");
    }
}
