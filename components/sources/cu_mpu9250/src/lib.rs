#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use core::fmt::Debug;

use alloc::format;
use cu29::prelude::*;
pub use cu_sensor_payloads::ImuPayload;
use mpu9250::{Device, Imu as ImuOnly, Marg, Mpu9250, NineDOFDevice};

#[cfg(feature = "embedded-hal")]
pub mod embedded_hal;
#[cfg(feature = "linux-embedded-hal")]
pub mod linux_embedded_hal;
#[cfg(feature = "rp235x-hal")]
pub mod rp235x_hal;

#[cfg(feature = "embedded-hal")]
pub use embedded_hal::set_gyro_bias;

fn map_debug_error<E: Debug>(context: &str, err: E) -> CuError {
    CuError::from(format!("{context}: {err:?}"))
}

#[derive(Serialize)]
#[repr(u8)]
pub enum WhoAmI {
    Mpu9250 = 0x71,
    Mpu9255 = 0x73,
    Mpu6500 = 0x70,
    Clone75 = 0x75,
    Ak8963 = 0x48,
    Unknown,
}

impl From<u8> for WhoAmI {
    fn from(v: u8) -> Self {
        match v {
            0x71 => WhoAmI::Mpu9250,
            0x73 => WhoAmI::Mpu9255,
            0x70 => WhoAmI::Mpu6500,
            0x75 => WhoAmI::Clone75,
            0x48 => WhoAmI::Ak8963,
            _ => WhoAmI::Unknown,
        }
    }
}

/// Trait implemented by MPU9250 backends capable of producing an [`ImuPayload`].
pub trait Mpu9250Device: Send + 'static {
    type Error: Debug;

    fn who_am_i(&mut self) -> Result<WhoAmI, Self::Error>;
    fn read_measure(&mut self) -> Result<ImuPayload, Self::Error>;
}

/// Factory trait for constructing MPU9250 sources from a Copper component configuration.
pub trait Mpu9250Factory: Mpu9250Device {
    fn try_new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized;
}

impl<DEV> Mpu9250Device for Mpu9250<DEV, Marg>
where
    DEV: Device + NineDOFDevice + Send + 'static,
    <DEV as Device>::Error: Debug,
{
    type Error = <DEV as Device>::Error;

    fn who_am_i(&mut self) -> Result<WhoAmI, Self::Error> {
        Ok(WhoAmI::from(Mpu9250::who_am_i(self)?))
    }

    fn read_measure(&mut self) -> Result<ImuPayload, Self::Error> {
        let data = self.all::<[f32; 3]>()?;
        Ok(ImuPayload::from_raw(data.accel, data.gyro, data.temp))
    }
}

impl<DEV> Mpu9250Device for Mpu9250<DEV, ImuOnly>
where
    DEV: Device + Send + 'static,
    <DEV as Device>::Error: Debug,
{
    type Error = <DEV as Device>::Error;

    fn who_am_i(&mut self) -> Result<WhoAmI, Self::Error> {
        Ok(WhoAmI::from(Mpu9250::who_am_i(self)?))
    }

    fn read_measure(&mut self) -> Result<ImuPayload, Self::Error> {
        let data = self.all::<[f32; 3]>()?;
        Ok(ImuPayload::from_raw(data.accel, data.gyro, data.temp))
    }
}

/// Copper source task for the MPU9250.
pub struct Mpu9250Source<D>
where
    D: Mpu9250Factory,
{
    driver: D,
}

impl<D> Freezable for Mpu9250Source<D> where D: Mpu9250Factory {}

impl<D> CuSrcTask for Mpu9250Source<D>
where
    D: Mpu9250Factory,
{
    type Output<'m> = output_msg!(ImuPayload);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let driver = D::try_new(config)?;
        Ok(Self { driver })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.driver
            .who_am_i()
            .map_err(|err| map_debug_error("mpu9250 WHO_AM_I", err))?;
        Ok(())
    }

    fn process<'o>(&mut self, clock: &RobotClock, new_msg: &mut Self::Output<'o>) -> CuResult<()> {
        let tov = clock.now(); // best effort here
        let payload = self
            .driver
            .read_measure()
            .map_err(|err| map_debug_error("mpu9250 read", err))?;
        new_msg.tov = Some(tov).into();
        // TODO: Make a good short status message for the sensor.
        // new_msg.metadata.set_status("ok");
        new_msg.set_payload(payload);
        Ok(())
    }
}
