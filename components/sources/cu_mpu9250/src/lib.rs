#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use core::fmt::Debug;

use alloc::format;
use core::marker::PhantomData;
use cu_embedded_registry as reg;
pub use cu_sensor_payloads::ImuPayload;
use cu29::prelude::*;
use embedded_hal_1 as eh1;
use mpu9250::{Device, Imu as ImuOnly, Marg, Mpu9250, NineDOFDevice};

pub mod embedded_hal;

pub use embedded_hal::set_gyro_bias;

fn map_debug_error<E: Debug>(context: &str, err: E) -> CuError {
    CuError::from(format!("{context}: {err:?}"))
}

#[derive(Serialize, Deserialize)]
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
pub struct Mpu9250Source<SPI, CS, D>
where
    SPI: eh1::spi::SpiBus<u8> + Send + 'static,
    SPI::Error: Debug + Send + 'static,
    CS: eh1::digital::OutputPin + Send + 'static,
    CS::Error: Debug + Send + 'static,
    D: eh1::delay::DelayNs + Send + 'static,
{
    driver: embedded_hal::EmbeddedHalDriver<SPI, CS>,
    _pd: PhantomData<D>,
}

impl<SPI, CS, D> Freezable for Mpu9250Source<SPI, CS, D>
where
    SPI: eh1::spi::SpiBus<u8> + Send + 'static,
    SPI::Error: Debug + Send + 'static,
    CS: eh1::digital::OutputPin + Send + 'static,
    CS::Error: Debug + Send + 'static,
    D: eh1::delay::DelayNs + Send + 'static,
{
}

impl<SPI, CS, D> CuSrcTask for Mpu9250Source<SPI, CS, D>
where
    SPI: eh1::spi::SpiBus<u8> + Send + 'static,
    SPI::Error: Debug + Send + 'static,
    CS: eh1::digital::OutputPin + Send + 'static,
    CS::Error: Debug + Send + 'static,
    D: eh1::delay::DelayNs + Send + 'static,
{
    type Resources<'r> = ();
    type Output<'m> = output_msg!(ImuPayload);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let settings = embedded_hal::EmbeddedHalSettings::from_config(config);

        let spi_slot = config
            .and_then(|cfg| cfg.get::<u32>("spi_slot"))
            .unwrap_or(0) as usize;
        let cs_slot = config
            .and_then(|cfg| cfg.get::<u32>("cs_slot"))
            .unwrap_or(0) as usize;
        let delay_slot = config
            .and_then(|cfg| cfg.get::<u32>("delay_slot"))
            .unwrap_or(0) as usize;

        let spi: SPI = reg::take_spi(spi_slot).ok_or_else(|| {
            CuError::from(format!(
                "mpu9250 SPI slot {spi_slot} empty (max {})",
                reg::MAX_SPI_SLOTS - 1
            ))
        })?;
        let cs: CS = reg::take_cs(cs_slot).ok_or_else(|| {
            CuError::from(format!(
                "mpu9250 CS slot {cs_slot} empty (max {})",
                reg::MAX_CS_SLOTS - 1
            ))
        })?;
        let delay: D = reg::take_delay(delay_slot).ok_or_else(|| {
            CuError::from(format!(
                "mpu9250 delay slot {delay_slot} empty (max {})",
                reg::MAX_DELAY_SLOTS - 1
            ))
        })?;

        let driver = embedded_hal::EmbeddedHalDriver::new(spi, cs, delay, settings)?;

        Ok(Self {
            driver,
            _pd: PhantomData,
        })
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
