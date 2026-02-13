use bincode::{Decode, Encode};
#[cfg(hardware)]
use cu_linux_resources::LinuxI2c;
use cu29::prelude::*;
#[cfg(hardware)]
use cu29::resource::Owned;
use cu29::resource::{ResourceBindingMap, ResourceBindings, ResourceManager};
use cu29::units::si::acceleration::standard_gravity;
use cu29::units::si::angle::degree;
use cu29::units::si::angular_velocity::degree_per_second;
use cu29::units::si::f32::{Acceleration, Angle, AngularVelocity, MagneticFluxDensity};
use cu29::units::si::magnetic_flux_density::nanotesla;
#[cfg(hardware)]
use embedded_hal::i2c::I2c;
use std::fmt::Display;

#[allow(unused)]
const WT901_I2C_ADDRESS: u8 = 0x50;

#[allow(unused)]
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
enum Registers {
    // Accelerometer addresses
    AccX = 0x34,
    AccY = 0x35,
    AccZ = 0x36,

    // Gyroscope addresses
    GyroX = 0x37,
    GyroY = 0x38,
    GyroZ = 0x39,

    // Magnetometer addresses
    MagX = 0x3A,
    MagY = 0x3B,
    MagZ = 0x3C,

    // Orientation addresses
    Roll = 0x3D,
    Pitch = 0x3E,
    Yaw = 0x3F,
}

impl Registers {
    #[allow(dead_code)]
    fn offset(&self) -> usize {
        ((*self as u8 - Registers::AccX as u8) * 2) as usize
    }
}

use cu29_log_derive::debug;
use cu29_traits::CuError;
use serde::{Deserialize, Serialize};

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct WT901 {
    #[cfg(hardware)]
    #[reflect(ignore)]
    i2c: LinuxI2c,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Binding {
    I2c,
}

pub struct Wt901Resources {
    #[cfg(hardware)]
    pub i2c: Owned<LinuxI2c>,
}

impl<'r> ResourceBindings<'r> for Wt901Resources {
    type Binding = Binding;

    fn from_bindings(
        manager: &'r mut ResourceManager,
        mapping: Option<&ResourceBindingMap<Self::Binding>>,
    ) -> CuResult<Self> {
        #[cfg(hardware)]
        {
            let mapping = mapping.ok_or_else(|| {
                CuError::from("WT901 requires an `i2c` resource mapping in copperconfig")
            })?;
            let path = mapping.get(Binding::I2c).ok_or_else(|| {
                CuError::from("WT901 resources must include `i2c: <bundle.resource>`")
            })?;
            let i2c = manager
                .take::<LinuxI2c>(path.typed())
                .map_err(|e| e.add_cause("Failed to fetch WT901 I2C resource"))?;
            Ok(Self { i2c })
        }
        #[cfg(mock)]
        {
            let _ = manager;
            let _ = mapping;
            Ok(Self {})
        }
    }
}

#[derive(Default, Clone, Debug, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct PositionalReadingsPayload {
    acc_x: Acceleration,
    acc_y: Acceleration,
    acc_z: Acceleration,
    gyro_x: AngularVelocity,
    gyro_y: AngularVelocity,
    gyro_z: AngularVelocity,
    mag_x: MagneticFluxDensity,
    mag_y: MagneticFluxDensity,
    mag_z: MagneticFluxDensity,
    roll: Angle,
    pitch: Angle,
    yaw: Angle,
}

impl Display for PositionalReadingsPayload {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "acc_x: {} g, acc_y: {} g, acc_z: {} g\n gyro_x: {} deg/s, gyro_y: {} deg/s, gyro_z: {} deg/s\nmag_x: {} nT, mag_y: {} nT, mag_z: {} nT\nroll: {} deg, pitch: {} deg, yaw: {} deg",
            self.acc_x.get::<standard_gravity>(),
            self.acc_y.get::<standard_gravity>(),
            self.acc_z.get::<standard_gravity>(),
            self.gyro_x.get::<degree_per_second>(),
            self.gyro_y.get::<degree_per_second>(),
            self.gyro_z.get::<degree_per_second>(),
            self.mag_x.get::<nanotesla>(),
            self.mag_y.get::<nanotesla>(),
            self.mag_z.get::<nanotesla>(),
            self.roll.get::<degree>(),
            self.pitch.get::<degree>(),
            self.yaw.get::<degree>()
        )
    }
}

// Number of registers to read in one go
#[allow(unused)]
const REGISTER_SPAN_SIZE: usize = ((Registers::Yaw as u8 - Registers::AccX as u8) * 2 + 2) as usize;

#[allow(unused)]
impl WT901 {
    fn bulk_position_read(&mut self, pr: &mut PositionalReadingsPayload) -> Result<(), CuError> {
        debug!("Trying to read i2c");

        #[cfg(hardware)]
        {
            let mut buf = [0u8; REGISTER_SPAN_SIZE];
            self.i2c
                .write_read(WT901_I2C_ADDRESS, &[Registers::AccX as u8], &mut buf)
                .expect("Error reading WT901");
            pr.acc_x = convert_acc(get_vec_i16(&buf, Registers::AccX.offset()));
            pr.acc_y = convert_acc(get_vec_i16(&buf, Registers::AccY.offset()));
            pr.acc_z = convert_acc(get_vec_i16(&buf, Registers::AccZ.offset()));
            pr.gyro_x = convert_ang_vel(get_vec_i16(&buf, Registers::GyroX.offset()));
            pr.gyro_y = convert_ang_vel(get_vec_i16(&buf, Registers::GyroY.offset()));
            pr.gyro_z = convert_ang_vel(get_vec_i16(&buf, Registers::GyroZ.offset()));
            pr.mag_x = convert_mag(get_vec_i16(&buf, Registers::MagX.offset()));
            pr.mag_y = convert_mag(get_vec_i16(&buf, Registers::MagY.offset()));
            pr.mag_z = convert_mag(get_vec_i16(&buf, Registers::MagZ.offset()));
            pr.roll = convert_angle(get_vec_i16(&buf, Registers::Roll.offset()));
            pr.pitch = convert_angle(get_vec_i16(&buf, Registers::Pitch.offset()));
            pr.yaw = convert_angle(get_vec_i16(&buf, Registers::Yaw.offset()));
        }
        Ok(())
    }
}

impl Freezable for WT901 {
    // WT901 has no internal state, we can leave the default implementation.
}

impl CuSrcTask for WT901 {
    type Resources<'r> = Wt901Resources;
    type Output<'m> = output_msg!(PositionalReadingsPayload);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let _ = _config;
        #[cfg(hardware)]
        let i2c = _resources.i2c.0;
        Ok(WT901 {
            #[cfg(hardware)]
            i2c,
        })
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        let mut pos = PositionalReadingsPayload::default();
        self.bulk_position_read(&mut pos)?;
        new_msg.set_payload(pos);
        Ok(())
    }
}

/// Get a u16 value out of a u8 buffer
#[inline]
#[allow(dead_code)]
fn get_vec_u16(buf: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([buf[offset], buf[offset + 1]])
}

/// Get a u16 value out of a u8 buffer
#[inline]
#[allow(dead_code)]
fn get_vec_i16(buf: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes([buf[offset], buf[offset + 1]])
}

#[allow(dead_code)]
fn convert_acc(acc: i16) -> Acceleration {
    // the scale is from 0 to 16g
    let acc = acc as f32 / 32768.0 * 16.0;
    Acceleration::new::<standard_gravity>(acc)
}

#[allow(dead_code)]
fn convert_ang_vel(angv: i16) -> AngularVelocity {
    // the scale is from 0 to 2000 deg/s
    let acc = (angv as f32 / 32768.0) * 2000.0;
    AngularVelocity::new::<degree_per_second>(acc)
}

#[allow(dead_code)]
fn convert_mag(mag: i16) -> MagneticFluxDensity {
    // the resolution is 8.333nT/LSB
    let mag = (mag as f32 / 32768.0) * 8.333;
    MagneticFluxDensity::new::<nanotesla>(mag)
}

#[allow(dead_code)]
fn convert_angle(angle: i16) -> Angle {
    let angle = angle as f32 / 32768.0 * 180.0;
    Angle::new::<degree>(angle)
}
