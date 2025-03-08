use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
#[cfg(hardware)]
use embedded_hal::i2c::I2c;
#[cfg(hardware)]
use linux_embedded_hal::{I2CError, I2cdev};
use std::fmt::Display;
use uom::si::acceleration::{meter_per_second_squared, standard_gravity};
use uom::si::angle::{degree, radian};
use uom::si::angular_velocity::{degree_per_second, radian_per_second};
use uom::si::f32::Acceleration;
use uom::si::f32::Angle;
use uom::si::f32::AngularVelocity;
use uom::si::f32::MagneticFluxDensity;
use uom::si::magnetic_flux_density::{nanotesla, tesla};

// FIXME: remove.
const I2C_BUS: &str = "/dev/i2c-9";
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
use serde::de::{Deserialize, Deserializer};
use serde::ser::{Serialize, SerializeStruct, Serializer};
use uom::fmt::DisplayStyle::Abbreviation;

pub struct WT901 {
    #[cfg(hardware)]
    i2c: Box<dyn I2c<Error = I2CError>>,
}

#[derive(Default, Clone, Debug)]
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
        let acc_style = Acceleration::format_args(standard_gravity, Abbreviation);
        let angv_style = AngularVelocity::format_args(degree_per_second, Abbreviation);
        let mag_style = MagneticFluxDensity::format_args(nanotesla, Abbreviation);
        let angle_style = Angle::format_args(degree, Abbreviation);

        write!(
            f,
            "acc_x: {}, acc_y: {}, acc_z: {}\n gyro_x: {}, gyro_y: {}, gyro_z: {}\nmag_x: {}, mag_y: {}, mag_z: {}\nroll: {}, pitch: {}, yaw: {}",
            acc_style.with(self.acc_x), acc_style.with(self.acc_y), acc_style.with(self.acc_z),
            angv_style.with(self.gyro_x), angv_style.with(self.gyro_y), angv_style.with(self.gyro_z),
            mag_style.with(self.mag_x), mag_style.with(self.mag_y), mag_style.with(self.mag_z),
            angle_style.with(self.roll), angle_style.with(self.pitch), angle_style.with(self.yaw)
        )
    }
}

impl Serialize for PositionalReadingsPayload {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut s = serializer.serialize_struct("PositionalReadings", 12)?;
        s.serialize_field("acc_x", &self.acc_x.value)?;
        s.serialize_field("acc_y", &self.acc_y.value)?;
        s.serialize_field("acc_z", &self.acc_z.value)?;
        s.serialize_field("gyro_x", &self.gyro_x.value)?;
        s.serialize_field("gyro_y", &self.gyro_y.value)?;
        s.serialize_field("gyro_z", &self.gyro_z.value)?;
        s.serialize_field("mag_x", &self.mag_x.value)?;
        s.serialize_field("mag_y", &self.mag_y.value)?;
        s.serialize_field("mag_z", &self.mag_z.value)?;
        s.serialize_field("roll", &self.roll.value)?;
        s.serialize_field("pitch", &self.pitch.value)?;
        s.serialize_field("yaw", &self.yaw.value)?;
        s.end()
    }
}

impl<'de> Deserialize<'de> for PositionalReadingsPayload {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let values = <[f32; 12]>::deserialize(deserializer)?;
        Ok(PositionalReadingsPayload {
            acc_x: Acceleration::new::<standard_gravity>(values[0]),
            acc_y: Acceleration::new::<standard_gravity>(values[1]),
            acc_z: Acceleration::new::<standard_gravity>(values[2]),
            gyro_x: AngularVelocity::new::<degree_per_second>(values[3]),
            gyro_y: AngularVelocity::new::<degree_per_second>(values[4]),
            gyro_z: AngularVelocity::new::<degree_per_second>(values[5]),
            mag_x: MagneticFluxDensity::new::<nanotesla>(values[6]),
            mag_y: MagneticFluxDensity::new::<nanotesla>(values[7]),
            mag_z: MagneticFluxDensity::new::<nanotesla>(values[8]),
            roll: Angle::new::<degree>(values[9]),
            pitch: Angle::new::<degree>(values[10]),
            yaw: Angle::new::<degree>(values[11]),
        })
    }
}

impl Encode for PositionalReadingsPayload {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        // Encode in natural SI units
        Encode::encode(&self.acc_x.value, encoder)?;
        bincode::Encode::encode(&self.acc_y.value, encoder)?;
        bincode::Encode::encode(&self.acc_z.value, encoder)?;
        bincode::Encode::encode(&self.gyro_x.value, encoder)?;
        bincode::Encode::encode(&self.gyro_y.value, encoder)?;
        bincode::Encode::encode(&self.gyro_z.value, encoder)?;
        bincode::Encode::encode(&self.mag_x.value, encoder)?;
        bincode::Encode::encode(&self.mag_y.value, encoder)?;
        bincode::Encode::encode(&self.mag_z.value, encoder)?;
        bincode::Encode::encode(&self.roll.value, encoder)?;
        bincode::Encode::encode(&self.pitch.value, encoder)?;
        bincode::Encode::encode(&self.yaw.value, encoder)?;
        Ok(())
    }
}

impl Decode<()> for PositionalReadingsPayload {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(PositionalReadingsPayload {
            // Decode back from the natural SI units
            acc_x: Acceleration::new::<meter_per_second_squared>(bincode::Decode::decode(decoder)?),
            acc_y: Acceleration::new::<meter_per_second_squared>(bincode::Decode::decode(decoder)?),
            acc_z: Acceleration::new::<meter_per_second_squared>(bincode::Decode::decode(decoder)?),
            gyro_x: AngularVelocity::new::<radian_per_second>(bincode::Decode::decode(decoder)?),
            gyro_y: AngularVelocity::new::<radian_per_second>(bincode::Decode::decode(decoder)?),
            gyro_z: AngularVelocity::new::<radian_per_second>(bincode::Decode::decode(decoder)?),
            mag_x: MagneticFluxDensity::new::<tesla>(bincode::Decode::decode(decoder)?),
            mag_y: MagneticFluxDensity::new::<tesla>(bincode::Decode::decode(decoder)?),
            mag_z: MagneticFluxDensity::new::<tesla>(bincode::Decode::decode(decoder)?),
            roll: Angle::new::<radian>(bincode::Decode::decode(decoder)?),
            pitch: Angle::new::<radian>(bincode::Decode::decode(decoder)?),
            yaw: Angle::new::<radian>(bincode::Decode::decode(decoder)?),
        })
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

impl<'cl> CuSrcTask<'cl> for WT901 {
    type Output = output_msg!('cl, PositionalReadingsPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        debug!("Opening {}... ", I2C_BUS);
        #[cfg(hardware)]
        let i2cdev = I2cdev::new(I2C_BUS).unwrap();
        debug!("{} opened.", I2C_BUS);
        Ok(WT901 {
            #[cfg(hardware)]
            i2c: Box::new(i2cdev),
        })
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
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
