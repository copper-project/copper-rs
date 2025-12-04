//! RP235x MPU9250 source placeholder to keep the build working; swap with the real driver when available.

use cu29::prelude::*;

use crate::{ImuPayload, Mpu9250Device, Mpu9250Factory, Mpu9250Source, WhoAmI};

pub struct RpMpu9250;

impl Mpu9250Device for RpMpu9250 {
    type Error = CuError;

    fn who_am_i(&mut self) -> Result<WhoAmI, Self::Error> {
        Ok(WhoAmI::Unknown)
    }

    fn read_measure(&mut self) -> Result<ImuPayload, Self::Error> {
        Ok(ImuPayload::default())
    }
}

impl Mpu9250Factory for RpMpu9250 {
    fn try_new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }
}

/// Source alias matching the Copper config type.
pub type RpMpu9250Source = Mpu9250Source<RpMpu9250>;
