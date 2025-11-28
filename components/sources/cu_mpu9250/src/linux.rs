use cu29::prelude::*;
use linux_embedded_hal::{Delay, I2cdev};
use mpu9250::{Device, I2cDevice, Imu as ImuOnly, Marg, Mpu9250};

use crate::{map_debug_error, ImuPayload, Mpu9250Device, Mpu9250Factory, Mpu9250Source};

type BusError = <I2cDevice<I2cdev> as Device>::Error;

#[cfg(not(feature = "std"))]
use alloc::format;

enum LinuxDriver {
    Marg(Mpu9250<I2cDevice<I2cdev>, Marg>),
    Imu(Mpu9250<I2cDevice<I2cdev>, ImuOnly>),
}

impl Mpu9250Device for LinuxDriver {
    type Error = BusError;

    fn who_am_i(&mut self) -> Result<u8, Self::Error> {
        match self {
            LinuxDriver::Marg(driver) => driver.who_am_i(),
            LinuxDriver::Imu(driver) => driver.who_am_i(),
        }
    }

    fn read_measure(&mut self) -> Result<ImuPayload, Self::Error> {
        match self {
            LinuxDriver::Marg(driver) => driver.read_measure(),
            LinuxDriver::Imu(driver) => driver.read_measure(),
        }
    }
}

pub struct LinuxMpu9250 {
    driver: LinuxDriver,
}

impl LinuxMpu9250 {
    fn from_path(path: &str) -> CuResult<Self> {
        let mut delay = Delay;
        let i2c = I2cdev::new(path)
            .map_err(|err| CuError::new_with_cause("opening I2C device for MPU9250", err))?;

        match Mpu9250::marg_default(i2c, &mut delay) {
            Ok(driver) => Ok(Self {
                driver: LinuxDriver::Marg(driver),
            }),
            Err(err) => {
                debug!(
                    "mpu9250 marg_default failed ({}), retrying imu-only",
                    format!("{err:?}")
                );
                let i2c = I2cdev::new(path).map_err(|open_err| {
                    CuError::new_with_cause(
                        "opening I2C device for MPU9250 (imu fallback)",
                        open_err,
                    )
                })?;
                let mut delay = Delay;
                let driver = Mpu9250::imu_default(i2c, &mut delay)
                    .map_err(|e| map_debug_error("mpu9250 imu_default", e))?;
                Ok(Self {
                    driver: LinuxDriver::Imu(driver),
                })
            }
        }
    }
}

impl Mpu9250Factory for LinuxMpu9250 {
    fn try_new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let path = config
            .and_then(|cfg| cfg.get::<String>("i2c_path"))
            .unwrap_or_else(|| "/dev/i2c-1".to_string());

        Self::from_path(&path)
    }
}

impl Mpu9250Device for LinuxMpu9250 {
    type Error = BusError;

    fn who_am_i(&mut self) -> Result<u8, Self::Error> {
        self.driver.who_am_i()
    }

    fn read_measure(&mut self) -> Result<ImuPayload, Self::Error> {
        self.driver.read_measure()
    }
}

/// Convenient source alias using the linux-embedded-hal I2C backend.
pub type LinuxMpu9250Source = Mpu9250Source<LinuxMpu9250>;
