use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use copper::clock::RobotClock;
use copper::config::NodeInstanceConfig;
use copper::cutask::{CuMsg, CuSrcTask, CuTaskLifecycle};
use copper::CuResult;
use i2cdev::core::I2CDevice;
use linux_embedded_hal::I2cdev;
use uom::si::acceleration::standard_gravity;
use uom::si::angle::degree;
use uom::si::angular_velocity::degree_per_second;
use uom::si::f32::Acceleration;
use uom::si::f32::Angle;
use uom::si::f32::AngularVelocity;
use uom::si::f32::MagneticFluxDensity;
use uom::si::magnetic_flux_density::nanotesla;

// FIXME: remove.
const I2C_BUS: &str = "/dev/i2c-9";
const WT901_I2C_ADDRESS: u16 = 0x50;

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
enum I2CAddr {
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

impl I2CAddr {
    fn offset(&self) -> usize {
        ((*self as u8 - I2CAddr::AccX as u8) * 2) as usize
    }
}

const TEMP: u8 = 0x40;

#[derive(Default, Debug)]
struct PositionalReadings {
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

impl Encode for PositionalReadings {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.acc_x.value.encode(encoder)?;
        self.acc_y.value.encode(encoder)?;
        self.acc_z.value.encode(encoder)?;
        self.gyro_x.value.encode(encoder)?;
        self.gyro_y.value.encode(encoder)?;
        self.gyro_z.value.encode(encoder)?;
        self.mag_x.value.encode(encoder)?;
        self.mag_y.value.encode(encoder)?;
        self.mag_z.value.encode(encoder)?;
        self.roll.value.encode(encoder)?;
        self.pitch.value.encode(encoder)?;
        self.yaw.value.encode(encoder)?;
        Ok(())
    }
}

impl Decode for PositionalReadings {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(PositionalReadings {
            acc_x: Acceleration::new::<standard_gravity>(f32::decode(decoder)?),
            acc_y: Acceleration::new::<standard_gravity>(f32::decode(decoder)?),
            acc_z: Acceleration::new::<standard_gravity>(f32::decode(decoder)?),
            gyro_x: AngularVelocity::new::<degree_per_second>(f32::decode(decoder)?),
            gyro_y: AngularVelocity::new::<degree_per_second>(f32::decode(decoder)?),
            gyro_z: AngularVelocity::new::<degree_per_second>(f32::decode(decoder)?),
            mag_x: MagneticFluxDensity::new::<nanotesla>(f32::decode(decoder)?),
            mag_y: MagneticFluxDensity::new::<nanotesla>(f32::decode(decoder)?),
            mag_z: MagneticFluxDensity::new::<nanotesla>(f32::decode(decoder)?),
            roll: Angle::new::<degree>(f32::decode(decoder)?),
            pitch: Angle::new::<degree>(f32::decode(decoder)?),
            yaw: Angle::new::<degree>(f32::decode(decoder)?),
        })
    }
}

struct WT901 {
    i2c: I2cdev,
}

impl WT901 {
    fn bulk_position_read(
        &mut self,
        pr: &mut PositionalReadings,
    ) -> Result<(), i2cdev::linux::LinuxI2CError> {
        let buf = self.i2c.smbus_read_i2c_block_data(
            I2CAddr::AccX as u8,
            (I2CAddr::Yaw as u8 - I2CAddr::AccX as u8) * 2 + 2,
        )?;

        pr.acc_x = convert_acc(get_vec_u16(&buf, I2CAddr::AccX.offset()));
        pr.acc_y = convert_acc(get_vec_u16(&buf, I2CAddr::AccY.offset()));
        pr.acc_z = convert_acc(get_vec_u16(&buf, I2CAddr::AccZ.offset()));
        pr.gyro_x = convert_ang_vel(get_vec_u16(&buf, I2CAddr::GyroX.offset()));
        pr.gyro_y = convert_ang_vel(get_vec_u16(&buf, I2CAddr::GyroY.offset()));
        pr.gyro_z = convert_ang_vel(get_vec_u16(&buf, I2CAddr::GyroZ.offset()));
        pr.mag_x = convert_mag(get_vec_u16(&buf, I2CAddr::MagX.offset()));
        pr.mag_y = convert_mag(get_vec_u16(&buf, I2CAddr::MagY.offset()));
        pr.mag_z = convert_mag(get_vec_u16(&buf, I2CAddr::MagZ.offset()));
        pr.roll = convert_angle(get_vec_u16(&buf, I2CAddr::Roll.offset()));
        pr.pitch = convert_angle(get_vec_u16(&buf, I2CAddr::Pitch.offset()));
        pr.yaw = convert_angle(get_vec_u16(&buf, I2CAddr::Yaw.offset()));
        Ok(())
    }
}

impl CuTaskLifecycle for WT901 {
    fn new(config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let i2cdev = I2cdev::new(I2C_BUS).unwrap();
        Ok(WT901 { i2c: i2cdev })
    }
}

impl CuSrcTask for WT901 {
    type Output = PositionalReadings;

    fn process(&mut self, clock: &RobotClock, new_msg: &mut CuMsg<Self::Output>) -> CuResult<()> {
        self.bulk_position_read(&mut new_msg.payload)
            .map_err(|e| format!("Error reading WT901: {:?}", e).into())
    }
}

/// Get a u16 value out of a u8 buffer
#[inline]
fn get_vec_u16(buf: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([buf[offset], buf[offset + 1]])
}

fn convert_acc(acc: u16) -> Acceleration {
    // the scale is from 0 to 16g
    let acc = acc as f32 / 32768.0 * 16.0;
    // renormalize to -8 to 8 + orient the direction correctly
    let acc = -(acc - 16.0 * ((acc + 8.0) / 16.0).floor());
    Acceleration::new::<standard_gravity>(acc)
}

fn convert_ang_vel(acc: u16) -> AngularVelocity {
    // the scale is from 0 to 2000 deg/s
    let acc = ((acc as i32 - 16384) as f32 / 32768.0) * 2000.0;
    AngularVelocity::new::<degree_per_second>(acc)
}

fn convert_mag(mag: u16) -> MagneticFluxDensity {
    // the resolution is 8.333nT/LSB
    let mag = ((mag as i32 - 16384) as f32 / 32768.0) * 8.333;
    MagneticFluxDensity::new::<nanotesla>(mag)
}

fn convert_angle(angle: u16) -> Angle {
    let angle = angle as f32 / 32768.0 * 180.0;
    let angle = angle - 360.0 * ((angle + 180.0) / 360.0).floor();
    Angle::new::<degree>(angle)
}
