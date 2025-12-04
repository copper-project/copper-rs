use core::fmt::Debug;

use cu29::prelude::*;
use embedded_hal::blocking::delay::DelayMs as Eh0DelayMs;
use embedded_hal::blocking::spi::{Transfer as Eh0Transfer, Write as Eh0Write};
use embedded_hal::digital::v2::OutputPin as Eh0OutputPin;
use embedded_hal_1 as eh1;
use mpu9250::{Marg, Mpu9250};
use uom::si::angular_velocity::radian_per_second;
use uom::si::f32::AngularVelocity as UomAngVel;

use crate::{map_debug_error, ImuPayload, Mpu9250Device, WhoAmI};

pub const DEFAULT_GYRO_CAL_MS: u32 = 0;
pub const DEFAULT_GYRO_SAMPLE_DELAY_MS: u32 = 10;

static mut GYRO_BIAS: [f32; 3] = [0.0; 3];

/// Override the gyro bias that gets subtracted from every sample.
pub fn set_gyro_bias(bias: [f32; 3]) {
    unsafe { GYRO_BIAS = bias };
}

/// Read the current gyro bias that will be subtracted from measurements.
pub fn gyro_bias() -> [f32; 3] {
    unsafe { GYRO_BIAS }
}

/// Per-driver settings derived from Copper component configuration.
#[derive(Clone, Copy)]
pub struct EmbeddedHalSettings {
    pub gyro_cal_ms: u32,
    pub gyro_sample_delay_ms: u32,
}

impl EmbeddedHalSettings {
    pub fn from_config(config: Option<&ComponentConfig>) -> Self {
        let gyro_cal_ms = config
            .and_then(|cfg| cfg.get("gyro_cal_ms"))
            .unwrap_or(DEFAULT_GYRO_CAL_MS);
        let gyro_sample_delay_ms = config
            .and_then(|cfg| cfg.get("gyro_sample_delay_ms"))
            .unwrap_or(DEFAULT_GYRO_SAMPLE_DELAY_MS);

        Self {
            gyro_cal_ms,
            gyro_sample_delay_ms,
        }
    }
}

pub type DriverError<SpiErr, CsErr> = mpu9250::Error<mpu9250::SpiError<SpiErr, CsErr>>;

type SpiDev<SPI, CS> = mpu9250::SpiDevice<Eh0SpiBus<SPI>, Eh0Cs<CS>>;
type SpiBusError<SPI> = <SPI as eh1::spi::ErrorType>::Error;
type CsError<CS> = <CS as eh1::digital::ErrorType>::Error;
type MpuError<SPI, CS> = DriverError<SpiBusError<SPI>, CsError<CS>>;

enum Mpu<SPI, CS> {
    Marg(Mpu9250<SpiDev<SPI, CS>, Marg>),
    Imu(Mpu9250<SpiDev<SPI, CS>, mpu9250::Imu>),
}

impl<SPI, CS> Mpu<SPI, CS>
where
    SPI: eh1::spi::SpiBus<u8>,
    SPI::Error: Debug,
    CS: eh1::digital::OutputPin,
    CS::Error: Debug,
{
    fn who_am_i(&mut self) -> Result<WhoAmI, MpuError<SPI, CS>> {
        let id = match self {
            Mpu::Marg(m) => m.who_am_i()?,
            Mpu::Imu(m) => m.who_am_i()?,
        };
        Ok(WhoAmI::from(id))
    }

    fn read_sample(&mut self) -> Result<ImuPayload, MpuError<SPI, CS>> {
        let (accel, gyro, temp) = match self {
            Mpu::Marg(m) => {
                let data = m.all::<[f32; 3]>()?;
                (data.accel, data.gyro, data.temp)
            }
            Mpu::Imu(m) => {
                let data = m.all::<[f32; 3]>()?;
                (data.accel, data.gyro, data.temp)
            }
        };
        Ok(ImuPayload::from_raw(accel, gyro, temp))
    }
}

/// Generic embedded-hal backed MPU9250 driver that works with any SPI bus/output pin/delay implementing eh1 traits.
pub struct EmbeddedHalDriver<SPI, CS> {
    mpu: Mpu<SPI, CS>,
}

impl<SPI, CS> EmbeddedHalDriver<SPI, CS> {
    fn apply_bias(payload: &mut ImuPayload, bias: [f32; 3]) {
        let bx = UomAngVel::new::<radian_per_second>(bias[0]);
        let by = UomAngVel::new::<radian_per_second>(bias[1]);
        let bz = UomAngVel::new::<radian_per_second>(bias[2]);
        payload.gyro_x -= bx;
        payload.gyro_y -= by;
        payload.gyro_z -= bz;
    }
}

impl<SPI, CS> EmbeddedHalDriver<SPI, CS>
where
    SPI: eh1::spi::SpiBus<u8> + Send + 'static,
    SPI::Error: Debug,
    CS: eh1::digital::OutputPin + Send + 'static,
    CS::Error: Debug,
{
    pub fn new<D: eh1::delay::DelayNs>(
        spi: SPI,
        cs: CS,
        delay: D,
        settings: EmbeddedHalSettings,
    ) -> CuResult<Self> {
        let mut delay = Eh0Delay::new(delay);
        let spi = Eh0SpiBus::new(spi);
        let cs = Eh0Cs::new(cs);

        let mut imu = Mpu9250::imu_default(spi, cs, &mut delay)
            .map_err(|e| map_debug_error("mpu9250 imu_default", e))?;

        let id = imu.who_am_i().unwrap_or(0);
        let mag_supported = matches!(id, 0x71 | 0x73);

        let mut mpu = if mag_supported {
            let (spi, cs) = imu.release();
            match Mpu9250::marg_default(spi, cs, &mut delay) {
                Ok(dev) => Mpu::Marg(dev),
                Err(e) => return Err(map_debug_error("mpu9250 marg_default", e)),
            }
        } else {
            Mpu::Imu(imu)
        };

        if settings.gyro_cal_ms > 0 {
            let bias = calibrate_gyro(
                &mut mpu,
                &mut delay,
                settings.gyro_cal_ms,
                settings.gyro_sample_delay_ms,
            );
            set_gyro_bias(bias);
        }

        Ok(Self { mpu })
    }
}

impl<SPI, CS> Mpu9250Device for EmbeddedHalDriver<SPI, CS>
where
    SPI: eh1::spi::SpiBus<u8> + Send + 'static,
    SPI::Error: Debug,
    CS: eh1::digital::OutputPin + Send + 'static,
    CS::Error: Debug,
{
    type Error = DriverError<SpiBusError<SPI>, CsError<CS>>;

    fn who_am_i(&mut self) -> Result<WhoAmI, Self::Error> {
        self.mpu.who_am_i()
    }

    fn read_measure(&mut self) -> Result<ImuPayload, Self::Error> {
        let mut payload = self.mpu.read_sample()?;
        Self::apply_bias(&mut payload, gyro_bias());
        Ok(payload)
    }
}

struct Eh0SpiBus<SPI> {
    inner: SPI,
}

unsafe impl<SPI: Send> Send for Eh0SpiBus<SPI> {}

impl<SPI> Eh0SpiBus<SPI> {
    fn new(inner: SPI) -> Self {
        Self { inner }
    }
}

impl<SPI> Eh0Write<u8> for Eh0SpiBus<SPI>
where
    SPI: eh1::spi::SpiBus<u8>,
{
    type Error = <SPI as eh1::spi::ErrorType>::Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.inner.write(words)
    }
}

impl<SPI> Eh0Transfer<u8> for Eh0SpiBus<SPI>
where
    SPI: eh1::spi::SpiBus<u8>,
{
    type Error = <SPI as eh1::spi::ErrorType>::Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.inner.transfer_in_place(words)?;
        Ok(words)
    }
}

struct Eh0Cs<CS> {
    inner: CS,
}

unsafe impl<CS: Send> Send for Eh0Cs<CS> {}

impl<CS> Eh0Cs<CS> {
    fn new(inner: CS) -> Self {
        Self { inner }
    }
}

impl<CS> Eh0OutputPin for Eh0Cs<CS>
where
    CS: eh1::digital::OutputPin,
{
    type Error = <CS as eh1::digital::ErrorType>::Error;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.inner.set_low()
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.inner.set_high()
    }
}

struct Eh0Delay<D> {
    inner: D,
}

impl<D> Eh0Delay<D> {
    fn new(inner: D) -> Self {
        Self { inner }
    }
}

impl<D> Eh0DelayMs<u8> for Eh0Delay<D>
where
    D: eh1::delay::DelayNs,
{
    fn delay_ms(&mut self, ms: u8) {
        self.inner.delay_ms(ms as u32);
    }
}

fn calibrate_gyro<SPI, CS, D>(
    mpu: &mut Mpu<SPI, CS>,
    delay: &mut Eh0Delay<D>,
    cal_ms: u32,
    sample_delay_ms: u32,
) -> [f32; 3]
where
    SPI: eh1::spi::SpiBus<u8>,
    CS: eh1::digital::OutputPin,
    D: eh1::delay::DelayNs,
{
    let delay_ms = sample_delay_ms.max(1);
    let samples = (cal_ms / delay_ms).max(1);

    let mut sum = [0.0f32; 3];
    let mut count: u32 = 0;
    let delay_step: u8 = delay_ms.min(u8::MAX as u32) as u8;

    for _ in 0..samples {
        if let Ok(payload) = mpu.read_sample() {
            sum[0] += payload.gyro_x.value;
            sum[1] += payload.gyro_y.value;
            sum[2] += payload.gyro_z.value;
            count += 1;
        }
        delay.delay_ms(delay_step);
    }

    if count == 0 {
        return [0.0; 3];
    }

    let inv = 1.0 / (count as f32);
    [sum[0] * inv, sum[1] * inv, sum[2] * inv]
}
