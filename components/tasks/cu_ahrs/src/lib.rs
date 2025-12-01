#![cfg_attr(not(feature = "std"), no_std)]
#![doc = include_str!("../README.md")]

use bincode::{Decode, Encode};
use cu29::prelude::*;
use cu_sensor_payloads::ImuPayload;
use dcmimu::DCMIMU;
use serde::Serialize;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angular_velocity::radian_per_second;

#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(not(feature = "std"))]
use alloc::vec::Vec;
#[cfg(feature = "std")]
use std::vec::Vec;

use core::mem::size_of;
use core::ptr;

/// Pose expressed as Euler angles (radians) using the aerospace body frame.
#[derive(Debug, Clone, Copy, Default, Encode, Decode, Serialize, PartialEq)]
pub struct AhrsPose {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl AhrsPose {
    fn relative_to(self, reference: AhrsPose) -> Self {
        Self {
            roll: self.roll - reference.roll,
            pitch: self.pitch - reference.pitch,
            yaw: self.yaw - reference.yaw,
        }
    }
}

/// Copper AHRS task that fuses IMU payloads into roll/pitch/yaw.
pub struct CuAhrs {
    dcm: DCMIMU,
    reference: Option<AhrsPose>,
    last_tov: Option<CuTime>,
}

impl CuAhrs {
    pub const fn new_filter() -> Self {
        Self {
            dcm: DCMIMU::new(),
            reference: None,
            last_tov: None,
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
            roll: angles.roll,
            pitch: angles.pitch,
            yaw: angles.yaw,
        };

        let reference = self.reference.get_or_insert(pose);
        pose.relative_to(*reference)
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
}

pub mod sinks {
    //! Simple sinks for AHRS outputs.
    use super::*;

    /// Logs incoming RPY to stdout/log and forwards the payload.
    pub struct RpyPrinter;

    impl Freezable for RpyPrinter {}

    impl CuTask for RpyPrinter {
        type Input<'m> = input_msg!(AhrsPose);
        type Output<'m> = output_msg!(AhrsPose);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
        }

        fn process(
            &mut self,
            _clock: &RobotClock,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            if let Some(pose) = input.payload() {
                info!(
                    "AHRS RPY [rad]: roll={:.3} pitch={:.3} yaw={:.3}",
                    pose.roll, pose.pitch, pose.yaw
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
        // Safety: DCMIMU is a plain-old-data struct of floats; we snapshot its bytes to preserve filter state.
        let bytes = unsafe {
            core::slice::from_raw_parts(
                (&self.dcm as *const DCMIMU) as *const u8,
                size_of::<DCMIMU>(),
            )
        };
        Encode::encode(&bytes, encoder)?;
        Encode::encode(&self.reference, encoder)?;
        Encode::encode(&self.last_tov, encoder)?;
        Ok(())
    }

    fn thaw<D: bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), bincode::error::DecodeError> {
        let raw: Vec<u8> = Decode::decode(decoder)?;
        let expected = size_of::<DCMIMU>();
        if raw.len() == expected {
            // Safety: we created the vector ourselves from a previous freeze; copy bytes back.
            unsafe {
                let ptr = (&mut self.dcm as *mut DCMIMU) as *mut u8;
                ptr::copy_nonoverlapping(raw.as_ptr(), ptr, expected);
            }
        } else {
            // Mismatch: fall back to a fresh filter to avoid undefined state.
            self.dcm = DCMIMU::new();
        }
        self.reference = Decode::decode(decoder)?;
        self.last_tov = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuTask for CuAhrs {
    type Input<'m> = input_msg!(ImuPayload);
    type Output<'m> = output_msg!(AhrsPose);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self::new_filter())
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let Some(payload) = input.payload() else {
            output.clear_payload();
            return Ok(());
        };

        let dt_s = match self.dt_seconds(&input.tov) {
            Some(dt) if dt > 0.0 => dt,
            _ => 1e-3,
        };
        let pose = self.update_pose(payload, dt_s);
        output.set_payload(pose);

        Ok(())
    }
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
        clock: &RobotClock,
        payload: ImuPayload,
        tov_ns: u64,
    ) -> Option<AhrsPose> {
        let mut input = CuMsg::new(Some(payload));
        input.tov = Tov::Time(CuTime::from(tov_ns));
        let mut output = CuMsg::new(None);
        task.process(clock, &input, &mut output).unwrap();
        output.payload().copied()
    }

    fn settle_pose(roll: f32, pitch: f32, iterations: usize, step_ns: u64) -> AhrsPose {
        let (clock, _) = RobotClock::mock();
        let mut task = CuAhrs::new_filter();
        let accel = accel_from_orientation(roll, pitch);
        let payload = ImuPayload::from_raw(accel, [0.0; 3], 25.0);

        let mut latest = None;
        for i in 0..iterations {
            latest = process_sample(&mut task, &clock, payload, step_ns * (i as u64 + 1));
        }
        latest.expect("pose should be produced")
    }

    #[test]
    fn level_orientation_stays_zeroed() {
        let pose = settle_pose(0.0, 0.0, 5, 10_000_000);
        assert!(pose.roll.abs() < 1e-3, "roll {}", pose.roll);
        assert!(pose.pitch.abs() < 1e-3, "pitch {}", pose.pitch);
        assert!(pose.yaw.abs() < 1e-3, "yaw {}", pose.yaw);
    }

    #[test]
    fn pitch_up_is_positive() {
        let target_pitch = FRAC_PI_3; // 60 deg nose up
        let pose = settle_pose(0.0, target_pitch, 80, 10_000_000);
        assert!(
            (pose.pitch - target_pitch).abs() < 0.1,
            "pitch {} vs {}",
            pose.pitch,
            target_pitch
        );
        assert!(pose.roll.abs() < 0.05);
    }

    #[test]
    fn roll_left_is_negative() {
        let target_roll = -FRAC_PI_3; // left wing down
        let pose = settle_pose(target_roll, 0.0, 80, 10_000_000);
        assert!(
            (pose.roll - target_roll).abs() < 0.1,
            "roll {} vs {}",
            pose.roll,
            target_roll
        );
        assert!(pose.pitch.abs() < 0.05);
    }

    #[test]
    fn yaw_integrates_gyro() {
        let (clock, _) = RobotClock::mock();
        let mut task = CuAhrs::new_filter();
        let accel = [0.0, 0.0, 9.81];
        let gyro = [0.0, 0.0, FRAC_PI_2]; // 90 deg/s about +Z
        let payload = ImuPayload::from_raw(accel, gyro, 25.0);

        let mut latest = None;
        for i in 0..10 {
            latest = process_sample(&mut task, &clock, payload, 100_000_000 * (i as u64 + 1));
        }

        let pose = latest.expect("pose should be produced");
        assert!(
            (pose.yaw - FRAC_PI_2).abs() < 0.2,
            "yaw {} vs {}",
            pose.yaw,
            FRAC_PI_2
        );
    }
}
