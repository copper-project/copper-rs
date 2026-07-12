//! Simulation-only ZED2i-compatible Copper source and frame exchange.

use bevy::prelude::Resource;
use cu_sensor_payloads::{BarometerPayload, ImuPayload, MagnetometerPayload};
use cu_zed::{
    ZedCalibrationBundle, ZedCameraIntrinsics, ZedConfidenceMap, ZedCoordinateSystem,
    ZedCoordinateUnit, ZedDepthMap, ZedFrameMeta, ZedNamedTransform, ZedRasterFormat,
    ZedRigTransforms, ZedSourceOutputs,
};
use cu29::prelude::*;
use std::sync::{Arc, Mutex, OnceLock};

pub(crate) const ZED_SIM_WIDTH: u32 = 320;
pub(crate) const ZED_SIM_HEIGHT: u32 = 180;
pub(crate) const ZED_SIM_VERTICAL_FOV_DEG: f32 = 70.0;
pub(crate) const ZED_SIM_BASELINE_M: f32 = 0.12;
pub(crate) const ZED_SIM_MAX_DEPTH_M: f32 = 20.0;
pub(crate) const ZED_SIM_DEPTH_FPS: u64 = 30;
pub(crate) const ZED_SIM_NEAR_M: f32 = 0.1;

#[derive(Clone, Copy, Default)]
pub(crate) struct SimZedSensors {
    pub(crate) imu: ImuPayload,
    pub(crate) magnetometer: MagnetometerPayload,
    pub(crate) barometer: BarometerPayload,
}

#[derive(Default)]
struct SimZedFrame {
    seq: u64,
    depth: Option<CuHandle<Vec<f32>>>,
    confidence: Option<CuHandle<Vec<f32>>>,
    sensors: Option<SimZedSensors>,
}

#[derive(Clone, Resource)]
pub(crate) struct SimZedFrameStore {
    inner: Arc<Mutex<SimZedFrame>>,
}

impl SimZedFrameStore {
    pub(crate) fn set_depth(&self, depth: Vec<f32>, confidence: Vec<f32>) {
        let mut frame = self.lock();
        frame.seq = frame.seq.wrapping_add(1);
        frame.depth = Some(CuHandle::new_detached(depth));
        frame.confidence = Some(CuHandle::new_detached(confidence));
    }

    pub(crate) fn set_sensors(&self, sensors: SimZedSensors) {
        self.lock().sensors = Some(sensors);
    }

    fn snapshot(&self) -> SimZedFrame {
        let frame = self.lock();
        SimZedFrame {
            seq: frame.seq,
            depth: frame.depth.clone(),
            confidence: frame.confidence.clone(),
            sensors: frame.sensors,
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, SimZedFrame> {
        self.inner
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

pub(crate) fn sim_zed_frame_store() -> SimZedFrameStore {
    static STORE: OnceLock<SimZedFrameStore> = OnceLock::new();
    STORE
        .get_or_init(|| SimZedFrameStore {
            inner: Arc::new(Mutex::new(SimZedFrame::default())),
        })
        .clone()
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct SimZed2iSource {
    #[reflect(ignore)]
    store: SimZedFrameStore,
    static_state_sent: bool,
}

impl Freezable for SimZed2iSource {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        cu29::bincode::Encode::encode(&self.static_state_sent, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        self.static_state_sent = cu29::bincode::Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSrcTask for SimZed2iSource {
    type Resources<'r> = ();
    type Output<'m> = ZedSourceOutputs;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            store: sim_zed_frame_store(),
            static_state_sent: false,
        })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let (stereo, depth, confidence, calibration, transforms, imu, mag, baro, meta) = output;
        let frame = self.store.snapshot();
        let tov = Tov::Time(ctx.now());

        stereo.clear_payload();
        stereo.tov = tov;

        if let Some(depth_handle) = frame.depth {
            let mut payload = ZedDepthMap::new(raster_format(), depth_handle);
            payload.seq = frame.seq;
            depth.set_payload(payload);
        } else {
            depth.clear_payload();
        }
        depth.tov = tov;

        if let Some(confidence_handle) = frame.confidence {
            let mut payload = ZedConfidenceMap::new(raster_format(), confidence_handle);
            payload.seq = frame.seq;
            confidence.set_payload(payload);
        } else {
            confidence.clear_payload();
        }
        confidence.tov = tov;

        if self.static_state_sent {
            calibration.set_payload(CuLatchedStateUpdate::NoChange);
            transforms.set_payload(CuLatchedStateUpdate::NoChange);
        } else {
            calibration.set_payload(CuLatchedStateUpdate::Set(calibration_bundle()));
            transforms.set_payload(CuLatchedStateUpdate::Set(rig_transforms()));
            self.static_state_sent = true;
        }
        calibration.tov = tov;
        transforms.tov = tov;

        if let Some(sensors) = frame.sensors {
            imu.set_payload(sensors.imu);
            mag.set_payload(sensors.magnetometer);
            baro.set_payload(sensors.barometer);
        } else {
            imu.clear_payload();
            mag.clear_payload();
            baro.clear_payload();
        }
        imu.tov = tov;
        mag.tov = tov;
        baro.tov = tov;

        meta.set_payload(ZedFrameMeta {
            seq: frame.seq,
            image_timestamp_ns: ctx.now().as_nanos(),
            current_timestamp_ns: ctx.now().as_nanos(),
            current_fps: ZED_SIM_DEPTH_FPS as f32,
            camera_moving_state: Some(1),
            image_sync_trigger: Some(0),
            imu_temp_c: Some(29.0),
            barometer_temp_c: Some(25.0),
            onboard_left_temp_c: Some(31.0),
            onboard_right_temp_c: Some(31.0),
        });
        meta.tov = tov;

        Ok(())
    }
}

fn raster_format() -> ZedRasterFormat {
    ZedRasterFormat {
        width: ZED_SIM_WIDTH,
        height: ZED_SIM_HEIGHT,
        stride: ZED_SIM_WIDTH,
    }
}

fn calibration_bundle() -> ZedCalibrationBundle {
    let width = ZED_SIM_WIDTH as f32;
    let height = ZED_SIM_HEIGHT as f32;
    let v_fov = ZED_SIM_VERTICAL_FOV_DEG;
    let fy = height / (2.0 * (0.5 * v_fov.to_radians()).tan());
    let fx = fy;
    let h_fov = 2.0 * (width / (2.0 * fx)).atan().to_degrees();
    let d_fov = 2.0
        * ((width * width + height * height).sqrt() / (2.0 * fx))
            .atan()
            .to_degrees();
    let intrinsics = ZedCameraIntrinsics {
        fx,
        fy,
        cx: (width - 1.0) * 0.5,
        cy: (height - 1.0) * 0.5,
        disto: [0.0; 12],
        v_fov,
        h_fov,
        d_fov,
        width: ZED_SIM_WIDTH,
        height: ZED_SIM_HEIGHT,
        focal_length_metric: 2.12,
    };

    ZedCalibrationBundle {
        serial_number: 2,
        width: ZED_SIM_WIDTH,
        height: ZED_SIM_HEIGHT,
        fps: ZED_SIM_DEPTH_FPS as f32,
        coordinate_system: ZedCoordinateSystem::LeftHandedYUp,
        coordinate_unit: ZedCoordinateUnit::Meter,
        left: intrinsics.clone(),
        right: intrinsics,
        stereo_rotation_rodrigues: [0.0; 3],
        stereo_translation_m: [ZED_SIM_BASELINE_M, 0.0, 0.0],
        camera_to_imu_translation_m: Some([0.0; 3]),
        camera_to_imu_quaternion_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
    }
}

fn rig_transforms() -> ZedRigTransforms {
    ZedRigTransforms {
        left_to_right: ZedNamedTransform {
            matrix: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [ZED_SIM_BASELINE_M, 0.0, 0.0, 1.0],
            ],
            parent_frame: "zed/left_camera".to_string(),
            child_frame: "zed/right_camera".to_string(),
        },
        camera_to_imu: ZedNamedTransform {
            matrix: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            parent_frame: "zed/left_camera".to_string(),
            child_frame: "zed/imu".to_string(),
        },
        has_camera_to_imu: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zed2i_sim_calibration_matches_render_target() {
        let calibration = calibration_bundle();
        assert_eq!(calibration.width, ZED_SIM_WIDTH);
        assert_eq!(calibration.height, ZED_SIM_HEIGHT);
        assert_eq!(calibration.stereo_translation_m[0], ZED_SIM_BASELINE_M);
        assert!(calibration.left.fx > 0.0);
        assert!(calibration.left.fy > 0.0);
    }
}
