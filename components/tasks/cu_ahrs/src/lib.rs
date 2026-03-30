#![cfg_attr(not(feature = "std"), no_std)]
#![doc = include_str!("../README.md")]

extern crate alloc;
use bincode::{Decode, Encode};
use core::time::Duration;
use cu_sensor_payloads::{ImuPayload, MagnetometerPayload};
use cu29::prelude::*;
use cu29::units::si::acceleration::meter_per_second_squared;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f32::Angle;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use uf_ahrs::{Ahrs, Mahony, MahonyParams};

/// Pose expressed as Euler angles (radians) using the aerospace body frame.
#[derive(
    Debug, Clone, Copy, Default, Encode, Decode, Serialize, Deserialize, PartialEq, Reflect,
)]
pub struct AhrsPose {
    pub roll: Angle,
    pub pitch: Angle,
    pub yaw: Angle,
}

/// Copper AHRS task that fuses IMU (and optional magnetometer) payloads into roll/pitch/yaw.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct CuAhrs {
    #[reflect(ignore)]
    filter: Mahony,
    last_tov: Option<CuTime>,
    sample_period_s: f32,
    auto_sample_period: bool,
    mahony_kp: f32,
    mahony_ki: f32,
}

impl CuAhrs {
    const DEFAULT_SAMPLE_PERIOD_S: f32 = 1.0 / 200.0;

    fn build_filter(
        sample_period_s: f32,
        kp: f32,
        ki: f32,
        orientation: UnitQuaternion<f32>,
    ) -> Mahony {
        let params = MahonyParams { kp, ki };
        Mahony::new_with_orientation(
            Duration::from_secs_f32(sample_period_s.max(1.0e-5)),
            params,
            orientation,
        )
    }

    pub fn new_filter() -> Self {
        let kp = MahonyParams::default().kp;
        let ki = MahonyParams::default().ki;
        let filter = Self::build_filter(
            Self::DEFAULT_SAMPLE_PERIOD_S,
            kp,
            ki,
            UnitQuaternion::identity(),
        );
        Self {
            filter,
            last_tov: None,
            sample_period_s: Self::DEFAULT_SAMPLE_PERIOD_S,
            auto_sample_period: true,
            mahony_kp: kp,
            mahony_ki: ki,
        }
    }

    fn from_config(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let mut filter = Self::new_filter();
        filter.mahony_kp =
            cfg_f32(config, "mahony_kp", MahonyParams::default().kp)?.clamp(0.0, 10.0);
        filter.mahony_ki =
            cfg_f32(config, "mahony_ki", MahonyParams::default().ki)?.clamp(0.0, 10.0);

        let sample_hz = cfg_f32(config, "sample_hz", 0.0)?;
        if sample_hz.is_finite() && sample_hz > 0.0 {
            filter.sample_period_s = (1.0 / sample_hz).clamp(1.0e-5, 1.0);
            filter.auto_sample_period = false;
        }

        filter.filter = Self::build_filter(
            filter.sample_period_s,
            filter.mahony_kp,
            filter.mahony_ki,
            UnitQuaternion::identity(),
        );
        Ok(filter)
    }

    fn dt_seconds(&mut self, tov: &Tov) -> Option<f32> {
        let current = match tov {
            Tov::Time(t) => Some(*t),
            Tov::Range(r) => Some(r.end),
            Tov::None => None,
        }?;

        let dt = self.last_tov.map(|previous| current - previous);
        self.last_tov = Some(current);

        dt.map(|duration| duration.as_nanos() as f32 * 1e-9)
    }

    fn maybe_lock_sample_period(&mut self, dt_s: Option<f32>) {
        if !self.auto_sample_period {
            return;
        }

        let Some(dt) = dt_s else {
            return;
        };
        if !dt.is_finite() || dt <= 1.0e-5 {
            return;
        }

        let orientation = self.filter.orientation();
        let bias = self.filter.bias;
        self.sample_period_s = dt.clamp(1.0e-5, 1.0);
        self.filter = Self::build_filter(
            self.sample_period_s,
            self.mahony_kp,
            self.mahony_ki,
            orientation,
        );
        self.filter.bias = bias;
        self.auto_sample_period = false;
    }

    fn update_pose(
        &mut self,
        imu: &ImuPayload,
        mag: Option<&MagnetometerPayload>,
        dt_s: Option<f32>,
    ) -> AhrsPose {
        self.maybe_lock_sample_period(dt_s);

        let gyro = Vector3::new(
            imu.gyro_x.get::<radian_per_second>(),
            imu.gyro_y.get::<radian_per_second>(),
            imu.gyro_z.get::<radian_per_second>(),
        );
        let accel = Vector3::new(
            imu.accel_x.get::<meter_per_second_squared>(),
            imu.accel_y.get::<meter_per_second_squared>(),
            imu.accel_z.get::<meter_per_second_squared>(),
        );

        let q = if let Some(mag) = mag {
            // Copper magnetometer payload follows the historical FC convention where +Y is mirrored
            // compared to the NED convention used by uf-ahrs. Convert at the AHRS boundary.
            let mag_vec = Vector3::new(mag.mag_x.value, -mag.mag_y.value, mag.mag_z.value);
            if mag_vec.x.is_finite()
                && mag_vec.y.is_finite()
                && mag_vec.z.is_finite()
                && mag_vec.norm_squared() > 1.0e-12
            {
                self.filter.update(gyro, accel, mag_vec)
            } else {
                self.filter.update_imu(gyro, accel)
            }
        } else {
            self.filter.update_imu(gyro, accel)
        };

        let (roll, pitch, yaw) = q.euler_angles();
        AhrsPose {
            roll: Angle::new::<radian>(roll),
            pitch: Angle::new::<radian>(pitch),
            yaw: Angle::new::<radian>(yaw),
        }
    }
}

pub mod sinks {
    //! Simple sinks for AHRS outputs.
    use super::*;

    /// Logs incoming RPY to stdout/log and forwards the payload.
    #[derive(Reflect)]
    pub struct RpyPrinter;

    impl Freezable for RpyPrinter {}

    impl CuTask for RpyPrinter {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(AhrsPose);
        type Output<'m> = output_msg!(AhrsPose);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(
            &mut self,
            _ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            if let Some(pose) = input.payload() {
                info!(
                    "AHRS RPY [rad]: roll={} pitch={} yaw={}",
                    pose.roll.get::<radian>(),
                    pose.pitch.get::<radian>(),
                    pose.yaw.get::<radian>()
                );
                output.set_payload(*pose);
            } else {
                output.clear_payload();
            }
            Ok(())
        }
    }
}

impl Freezable for CuAhrs {
    fn freeze<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        let q = self.filter.orientation();
        let qq = q.quaternion();
        let quat = [qq.w, qq.i, qq.j, qq.k];
        let bias = [self.filter.bias.x, self.filter.bias.y, self.filter.bias.z];
        Encode::encode(&quat, encoder)?;
        Encode::encode(&bias, encoder)?;
        Encode::encode(&self.sample_period_s, encoder)?;
        Encode::encode(&self.auto_sample_period, encoder)?;
        Encode::encode(&self.mahony_kp, encoder)?;
        Encode::encode(&self.mahony_ki, encoder)?;
        Encode::encode(&self.last_tov, encoder)?;
        Ok(())
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        let quat: [f32; 4] = Decode::decode(decoder)?;
        let bias: [f32; 3] = Decode::decode(decoder)?;
        self.sample_period_s = Decode::decode(decoder)?;
        self.auto_sample_period = Decode::decode(decoder)?;
        self.mahony_kp = Decode::decode(decoder)?;
        self.mahony_ki = Decode::decode(decoder)?;
        self.last_tov = Decode::decode(decoder)?;

        let valid_quat = quat.iter().all(|v| v.is_finite())
            && !(quat[0].abs() < 1.0e-12
                && quat[1].abs() < 1.0e-12
                && quat[2].abs() < 1.0e-12
                && quat[3].abs() < 1.0e-12);
        let orientation = if valid_quat {
            UnitQuaternion::new_normalize(Quaternion::new(quat[0], quat[1], quat[2], quat[3]))
        } else {
            UnitQuaternion::identity()
        };

        self.sample_period_s = if self.sample_period_s.is_finite() {
            self.sample_period_s.clamp(1.0e-5, 1.0)
        } else {
            Self::DEFAULT_SAMPLE_PERIOD_S
        };
        self.mahony_kp = if self.mahony_kp.is_finite() {
            self.mahony_kp.clamp(0.0, 10.0)
        } else {
            MahonyParams::default().kp
        };
        self.mahony_ki = if self.mahony_ki.is_finite() {
            self.mahony_ki.clamp(0.0, 10.0)
        } else {
            MahonyParams::default().ki
        };

        self.filter = Self::build_filter(
            self.sample_period_s,
            self.mahony_kp,
            self.mahony_ki,
            orientation,
        );
        if bias.iter().all(|v| v.is_finite()) {
            self.filter.bias = Vector3::new(bias[0], bias[1], bias[2]);
        } else {
            self.filter.bias = Vector3::new(0.0, 0.0, 0.0);
        }
        Ok(())
    }
}

impl CuTask for CuAhrs {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, ImuPayload, MagnetometerPayload);
    type Output<'m> = output_msg!(AhrsPose);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Self::from_config(config)
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let (imu_msg, mag_msg) = *input;
        output.tov = imu_msg.tov;
        let Some(imu) = imu_msg.payload() else {
            #[cfg(not(feature = "firmware"))]
            output.metadata.set_status("imu none");
            output.clear_payload();
            return Ok(());
        };

        let dt_s = self.dt_seconds(&imu_msg.tov);
        let pose = self.update_pose(imu, mag_msg.payload(), dt_s);

        #[cfg(not(feature = "firmware"))]
        output.metadata.set_status(alloc::format!(
            "r{} p{} y{}",
            round_degrees_to_i16(pose.roll.get::<radian>().to_degrees()),
            round_degrees_to_i16(pose.pitch.get::<radian>().to_degrees()),
            round_degrees_to_i16(pose.yaw.get::<radian>().to_degrees())
        ));
        output.set_payload(pose);

        Ok(())
    }
}

#[cfg(not(feature = "firmware"))]
fn round_degrees_to_i16(value: f32) -> i16 {
    if value >= 0.0 {
        (value + 0.5) as i16
    } else {
        (value - 0.5) as i16
    }
}

fn cfg_f32(config: Option<&ComponentConfig>, key: &str, default: f32) -> CuResult<f32> {
    let value = match config {
        Some(cfg) => cfg.get::<f64>(key)?,
        None => None,
    };
    Ok(value.map(|v| v as f32).unwrap_or(default))
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_3};
    use cu29::cutask::CuMsg;

    fn accel_from_orientation(roll: f32, pitch: f32) -> [f32; 3] {
        // Using aerospace body frame (X forward, Y right, Z down).
        // Gravity-only measurement rotated into body frame.
        let g = 9.81;
        let sr = roll.sin();
        let cr = roll.cos();
        let sp = pitch.sin();
        let cp = pitch.cos();
        [-g * sp, g * sr * cp, g * cr * cp]
    }

    fn process_sample(
        task: &mut CuAhrs,
        ctx: &CuContext,
        payload: ImuPayload,
        mag: Option<MagnetometerPayload>,
        tov_ns: u64,
    ) -> Option<AhrsPose> {
        let tov = Tov::Time(CuTime::from(tov_ns));
        let mut imu_msg = CuMsg::new(Some(payload));
        imu_msg.tov = tov;
        let mut mag_msg = CuMsg::new(mag);
        mag_msg.tov = tov;
        let mut output = CuMsg::new(None);
        let input = (&imu_msg, &mag_msg);
        task.process(ctx, &input, &mut output).unwrap();
        output.payload().copied()
    }

    fn settle_pose(roll: f32, pitch: f32, iterations: usize, step_ns: u64) -> AhrsPose {
        let ctx = CuContext::new_with_clock();
        let mut task = CuAhrs::new_filter();
        let accel = accel_from_orientation(roll, pitch);
        let payload = ImuPayload::from_raw(accel, [0.0; 3], 25.0);

        let mut latest = None;
        for i in 0..iterations {
            latest = process_sample(&mut task, &ctx, payload, None, step_ns * (i as u64 + 1));
        }
        latest.expect("pose should be produced")
    }

    #[test]
    fn level_orientation_stays_zeroed() {
        let pose = settle_pose(0.0, 0.0, 12, 10_000_000);
        assert!(
            pose.roll.get::<radian>().abs() < 0.03,
            "roll {}",
            pose.roll.value
        );
        assert!(
            pose.pitch.get::<radian>().abs() < 0.03,
            "pitch {}",
            pose.pitch.value
        );
    }

    #[test]
    fn pitch_up_is_positive() {
        let target_pitch = FRAC_PI_3; // 60 deg nose up
        let pose = settle_pose(0.0, target_pitch, 120, 10_000_000);
        assert!(
            pose.pitch.get::<radian>() > 0.4,
            "pitch {} should be positive and significantly nose-up (target {})",
            pose.pitch.value,
            target_pitch
        );
        assert!(pose.roll.get::<radian>().abs() < 0.15);
    }

    #[test]
    fn roll_left_is_negative() {
        let target_roll = -FRAC_PI_3; // left wing down
        let pose = settle_pose(target_roll, 0.0, 120, 10_000_000);
        assert!(
            pose.roll.get::<radian>() < -0.4,
            "roll {} should be negative and significantly left-down (target {})",
            pose.roll.value,
            target_roll
        );
        assert!(pose.pitch.get::<radian>().abs() < 0.15);
    }

    #[test]
    fn yaw_integrates_gyro() {
        let ctx = CuContext::new_with_clock();
        let mut task = CuAhrs::new_filter();
        let accel = [0.0, 0.0, 9.81];
        let gyro = [0.0, 0.0, FRAC_PI_2]; // 90 deg/s about +Z
        let payload = ImuPayload::from_raw(accel, gyro, 25.0);

        let mut latest = None;
        for i in 0..10 {
            latest = process_sample(&mut task, &ctx, payload, None, 100_000_000 * (i as u64 + 1));
        }

        let pose = latest.expect("pose should be produced");
        assert!(
            (pose.yaw.get::<radian>() - FRAC_PI_2).abs() < 0.3,
            "yaw {} vs {}",
            pose.yaw.value,
            FRAC_PI_2
        );
    }

    #[test]
    fn magnetometer_correction_anchors_yaw() {
        let ctx = CuContext::new_with_clock();
        let mut task = CuAhrs::new_filter();
        let imu = ImuPayload::from_raw([0.0, 0.0, 9.81], [0.0, 0.0, 0.0], 25.0);
        // Copper payload convention uses +Y for east. AHRS converts to uf-ahrs convention internally.
        let mag = MagnetometerPayload::from_raw([0.0, 20.0, 0.0]);

        let mut latest = None;
        for i in 0..2_000 {
            latest = process_sample(&mut task, &ctx, imu, Some(mag), 10_000_000 * (i as u64 + 1));
        }

        let pose = latest.expect("pose should be produced");
        let yaw_deg = pose.yaw.get::<radian>().to_degrees().rem_euclid(360.0);
        assert!(
            (yaw_deg - 90.0).abs() < 5.0,
            "yaw_deg {} should converge near 90°",
            yaw_deg
        );
    }
}
