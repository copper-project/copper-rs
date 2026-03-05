#![cfg_attr(not(feature = "std"), no_std)]
#![doc = include_str!("../README.md")]

extern crate alloc;
use bincode::{Decode, Encode};
use cu_sensor_payloads::{ImuPayload, MagnetometerPayload};
use cu29::prelude::*;
use cu29::units::si::acceleration::meter_per_second_squared;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f32::Angle;
use dcmimu::DCMIMU;
use serde::{Deserialize, Serialize};

#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

use core::mem::size_of;
use core::ptr;

/// Pose expressed as Euler angles (radians) using the aerospace body frame.
#[derive(
    Debug, Clone, Copy, Default, Encode, Decode, Serialize, Deserialize, PartialEq, Reflect,
)]
pub struct AhrsPose {
    pub roll: Angle,
    pub pitch: Angle,
    pub yaw: Angle,
}

/// Copper AHRS task that fuses IMU payloads into roll/pitch/yaw.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct CuAhrs {
    #[reflect(ignore)]
    dcm: DCMIMU,
    last_tov: Option<CuTime>,
    yaw_offset_rad: f32,
    mag_correction_gain: f32,
    mag_tilt_limit_rad: f32,
}

impl CuAhrs {
    pub const fn new_filter() -> Self {
        Self {
            dcm: DCMIMU::new(),
            last_tov: None,
            yaw_offset_rad: 0.0,
            mag_correction_gain: 0.03,
            mag_tilt_limit_rad: 50.0_f32.to_radians(),
        }
    }

    fn update_pose(&mut self, payload: &ImuPayload, dt_s: f32) -> AhrsPose {
        let accel = [
            payload.accel_x.get::<meter_per_second_squared>(),
            payload.accel_y.get::<meter_per_second_squared>(),
            payload.accel_z.get::<meter_per_second_squared>(),
        ];
        let gyro = [
            payload.gyro_x.get::<radian_per_second>(),
            payload.gyro_y.get::<radian_per_second>(),
            payload.gyro_z.get::<radian_per_second>(),
        ];

        let (angles, _) = self.dcm.update(
            (gyro[0], gyro[1], gyro[2]),
            (accel[0], accel[1], accel[2]),
            dt_s.max(0.0),
        );

        let pose = AhrsPose {
            roll: Angle::new::<radian>(angles.roll),
            pitch: Angle::new::<radian>(angles.pitch),
            yaw: Angle::new::<radian>(angles.yaw),
        };
        pose
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

    fn wrap_angle_rad(value: f32) -> f32 {
        let two_pi = 2.0 * core::f32::consts::PI;
        let mut wrapped = libm::fmodf(value, two_pi);
        if wrapped < 0.0 {
            wrapped += two_pi;
        }
        wrapped
    }

    fn shortest_angle_rad(delta: f32) -> f32 {
        let two_pi = 2.0 * core::f32::consts::PI;
        let mut wrapped = libm::fmodf(delta, two_pi);
        if wrapped <= -core::f32::consts::PI {
            wrapped += two_pi;
        } else if wrapped > core::f32::consts::PI {
            wrapped -= two_pi;
        }
        wrapped
    }

    fn heading_from_mag_level_rad(
        mag_x: f32,
        mag_y: f32,
        mag_z: f32,
        roll_rad: f32,
        pitch_rad: f32,
    ) -> Option<f32> {
        if !mag_x.is_finite()
            || !mag_y.is_finite()
            || !mag_z.is_finite()
            || !roll_rad.is_finite()
            || !pitch_rad.is_finite()
        {
            return None;
        }

        // Body axes are aerospace/NED: X forward, Y right, Z down.
        let sin_r = libm::sinf(roll_rad);
        let cos_r = libm::cosf(roll_rad);
        let sin_p = libm::sinf(pitch_rad);
        let cos_p = libm::cosf(pitch_rad);
        let horizontal_x = mag_x * cos_p + mag_z * sin_p;
        let horizontal_y = mag_x * sin_r * sin_p + mag_y * cos_r - mag_z * sin_r * cos_p;
        let hnorm2 = horizontal_x * horizontal_x + horizontal_y * horizontal_y;
        if !hnorm2.is_finite() || hnorm2 <= 1.0e-12 {
            return None;
        }
        Some(Self::wrap_angle_rad(libm::atan2f(horizontal_y, horizontal_x)))
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
        // SAFETY: DCMIMU is a plain-old-data struct of floats; we snapshot its bytes to preserve state.
        let bytes = unsafe {
            core::slice::from_raw_parts(
                (&self.dcm as *const DCMIMU) as *const u8,
                size_of::<DCMIMU>(),
            )
        };
        Encode::encode(&bytes, encoder)?;
        Encode::encode(&self.last_tov, encoder)?;
        Encode::encode(&self.yaw_offset_rad, encoder)?;
        Ok(())
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        let raw: Vec<u8> = Decode::decode(decoder)?;
        let expected = size_of::<DCMIMU>();
        if raw.len() == expected {
            // SAFETY: We created the vector ourselves from a previous freeze; copy bytes back.
            unsafe {
                let ptr = (&mut self.dcm as *mut DCMIMU) as *mut u8;
                ptr::copy_nonoverlapping(raw.as_ptr(), ptr, expected);
            }
        } else {
            // Mismatch: fall back to a fresh filter to avoid undefined state.
            self.dcm = DCMIMU::new();
        }
        self.last_tov = Decode::decode(decoder)?;
        self.yaw_offset_rad = Decode::decode(decoder)?;
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
        let mut filter = Self::new_filter();
        filter.mag_correction_gain = cfg_f32(config, "mag_correction_gain", 0.03)?.clamp(0.0, 1.0);
        filter.mag_tilt_limit_rad = cfg_f32(config, "mag_tilt_limit_deg", 50.0)?
            .abs()
            .to_radians()
            .clamp(1.0_f32.to_radians(), 89.0_f32.to_radians());
        Ok(filter)
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let (imu_msg, mag_msg) = *input;
        output.tov = imu_msg.tov;
        let Some(payload) = imu_msg.payload() else {
            #[cfg(not(feature = "firmware"))]
            output.metadata.set_status("imu none");
            output.clear_payload();
            return Ok(());
        };

        let dt_s = match self.dt_seconds(&imu_msg.tov) {
            Some(dt) if dt > 0.0 => dt,
            _ => 1e-3,
        };
        let mut pose = self.update_pose(payload, dt_s);

        let roll_rad = pose.roll.get::<radian>();
        let pitch_rad = pose.pitch.get::<radian>();
        let base_yaw_rad = Self::wrap_angle_rad(pose.yaw.get::<radian>());
        let mut corrected_yaw_rad = Self::wrap_angle_rad(base_yaw_rad + self.yaw_offset_rad);
        if let Some(mag) = mag_msg.payload() {
            let tilt_ok =
                roll_rad.abs() <= self.mag_tilt_limit_rad && pitch_rad.abs() <= self.mag_tilt_limit_rad;
            if tilt_ok
                && let Some(mag_heading_rad) = Self::heading_from_mag_level_rad(
                    mag.mag_x.value,
                    mag.mag_y.value,
                    mag.mag_z.value,
                    roll_rad,
                    pitch_rad,
                )
            {
                let err = Self::shortest_angle_rad(mag_heading_rad - corrected_yaw_rad);
                self.yaw_offset_rad =
                    Self::wrap_angle_rad(self.yaw_offset_rad + self.mag_correction_gain * err);
                corrected_yaw_rad = Self::wrap_angle_rad(base_yaw_rad + self.yaw_offset_rad);
            }
        }

        pose.yaw = Angle::new::<radian>(corrected_yaw_rad);

        #[cfg(not(feature = "firmware"))]
        output.metadata.set_status(alloc::format!(
            "r{} p{} y{}",
            pose.roll.get::<radian>().to_degrees().round() as i16,
            pose.pitch.get::<radian>().to_degrees().round() as i16,
            pose.yaw.get::<radian>().to_degrees().round() as i16
        ));
        output.set_payload(pose);

        Ok(())
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
        let pose = settle_pose(0.0, 0.0, 5, 10_000_000);
        assert!(
            pose.roll.get::<radian>().abs() < 1e-3,
            "roll {}",
            pose.roll.value
        );
        assert!(
            pose.pitch.get::<radian>().abs() < 1e-3,
            "pitch {}",
            pose.pitch.value
        );
        assert!(
            pose.yaw.get::<radian>().abs() < 1e-3,
            "yaw {}",
            pose.yaw.value
        );
    }

    #[test]
    fn pitch_up_is_positive() {
        let target_pitch = FRAC_PI_3; // 60 deg nose up
        let pose = settle_pose(0.0, target_pitch, 80, 10_000_000);
        assert!(
            (pose.pitch.get::<radian>() - target_pitch).abs() < 0.1,
            "pitch {} vs {}",
            pose.pitch.value,
            target_pitch
        );
        assert!(pose.roll.get::<radian>().abs() < 0.05);
    }

    #[test]
    fn roll_left_is_negative() {
        let target_roll = -FRAC_PI_3; // left wing down
        let pose = settle_pose(target_roll, 0.0, 80, 10_000_000);
        assert!(
            (pose.roll.get::<radian>() - target_roll).abs() < 0.1,
            "roll {} vs {}",
            pose.roll.value,
            target_roll
        );
        assert!(pose.pitch.get::<radian>().abs() < 0.05);
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
            (pose.yaw.get::<radian>() - FRAC_PI_2).abs() < 0.2,
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
        // Level frame with magnetic east and positive down component.
        let mag = MagnetometerPayload::from_raw([0.0, 20.0, 45.0]);

        let mut latest = None;
        for i in 0..200 {
            latest = process_sample(
                &mut task,
                &ctx,
                imu,
                Some(mag),
                10_000_000 * (i as u64 + 1),
            );
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
