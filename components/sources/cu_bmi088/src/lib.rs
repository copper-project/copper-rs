//! Copper source driver for the BMI088 6-axis IMU (accelerometer + gyroscope).
//!
//! The BMI088 is a high-performance inertial measurement unit with a 16-bit accelerometer
//! and a 16-bit gyroscope. It is commonly used in flight controllers and robotics applications.
//!
//! This driver uses `embedded-hal` 0.2 traits for SPI communication and GPIO chip-select control,
//! making it portable across different embedded platforms.
//!
//! # Hardware Notes
//!
//! The BMI088 has separate chip-select lines for the accelerometer and gyroscope:
//! - Accelerometer: Requires a dummy byte after register address reads
//! - Gyroscope: Standard SPI read protocol
//!
//! # Example Configuration (copperconfig.ron)
//!
//! ```ron
//! (
//!     id: "imu",
//!     type: "cu_bmi088::Bmi088Source<MySpi, MyAccCs, MyGyrCs, MyDelay>",
//!     resources: {
//!         "spi": "hal.bmi088_spi",
//!         "acc_cs": "hal.bmi088_acc_cs",
//!         "gyr_cs": "hal.bmi088_gyr_cs",
//!         "delay": "hal.bmi088_delay",
//!     },
//! )
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![allow(dead_code)]

use core::fmt::Debug;

pub use cu_sensor_payloads::ImuPayload;
use cu29::prelude::*;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;

// Chip IDs
const BMI088_ACC_CHIP_ID: u8 = 0x1E;
const BMI088_GYR_CHIP_ID: u8 = 0x0F;

// Accelerometer registers
const BMI088_ACC_REG_CHIP_ID: u8 = 0x00;
const BMI088_ACC_REG_ACCEL_X_LSB: u8 = 0x12;
const BMI088_ACC_REG_TEMP_MSB: u8 = 0x22;
const BMI088_ACC_REG_TEMP_LSB: u8 = 0x23;
const BMI088_ACC_REG_ACC_RANGE: u8 = 0x41;
const BMI088_ACC_REG_PWR_CONF: u8 = 0x7C;
const BMI088_ACC_REG_PWR_CTRL: u8 = 0x7D;
const BMI088_ACC_REG_SOFT_RESET: u8 = 0x7E;

// Gyroscope registers
const BMI088_GYR_REG_CHIP_ID: u8 = 0x00;
const BMI088_GYR_REG_X_LSB: u8 = 0x02;
const BMI088_GYR_REG_RANGE: u8 = 0x0F;
const BMI088_GYR_REG_BANDWIDTH: u8 = 0x10;
const BMI088_GYR_REG_POWER_MODE: u8 = 0x11;
const BMI088_GYR_REG_SOFT_RESET: u8 = 0x14;

// Configuration values
const BMI088_SOFT_RESET_CMD: u8 = 0xB6;
const BMI088_ACC_PWR_CONF_ACTIVE: u8 = 0x00;
const BMI088_ACC_PWR_CTRL_ON: u8 = 0x04;
const BMI088_GYR_RANGE_2000: u8 = 0x00;
const BMI088_GYR_BANDWIDTH: u8 = 0x07; // ODR 2000Hz, Filter BW 230Hz
const BMI088_GYR_PWR_NORMAL: u8 = 0x00;

// Conversion factor for gyroscope at 2000 dps range
const GYRO_RAD_PER_LSB: f32 = (2000.0 / 32_768.0) * (core::f32::consts::PI / 180.0);

// Resource definitions for the BMI088 source task.
// The BMI088 requires:
// - `spi`: SPI bus (shared between acc and gyro)
// - `acc_cs`: Accelerometer chip-select GPIO
// - `gyr_cs`: Gyroscope chip-select GPIO
// - `delay`: Delay provider for initialization timing
resources!(for <SPI, ACC, GYR, D>
where
    SPI: Transfer<u8> + Send + Sync + 'static,
    SPI::Error: Debug + Send + 'static,
    ACC: OutputPin + Send + Sync + 'static,
    ACC::Error: Debug + Send + 'static,
    GYR: OutputPin + Send + Sync + 'static,
    GYR::Error: Debug + Send + 'static,
    D: DelayMs<u32> + Send + Sync + 'static,
{
    spi => Owned<SPI>,
    acc_cs => Owned<ACC>,
    gyr_cs => Owned<GYR>,
    delay => Owned<D>,
});

/// Copper source task for the BMI088 IMU.
///
/// This task reads accelerometer and gyroscope data from the BMI088 and outputs
/// an [`ImuPayload`] with measurements in SI units (m/s² for acceleration,
/// rad/s for angular velocity, °C for temperature).
///
/// # Type Parameters
///
/// - `SPI`: SPI bus type implementing `embedded_hal::blocking::spi::Transfer<u8>`
/// - `ACC`: Accelerometer chip-select GPIO implementing `OutputPin`
/// - `GYR`: Gyroscope chip-select GPIO implementing `OutputPin`
/// - `D`: Delay provider implementing `DelayMs<u32>`
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct Bmi088Source<SPI, ACC, GYR, D> {
    #[reflect(ignore)]
    driver: Bmi088Driver<SPI, ACC, GYR, D>,
}

impl<SPI, ACC, GYR, D> TypePath for Bmi088Source<SPI, ACC, GYR, D>
where
    SPI: 'static,
    ACC: 'static,
    GYR: 'static,
    D: 'static,
{
    fn type_path() -> &'static str {
        "cu_bmi088::Bmi088Source"
    }

    fn short_type_path() -> &'static str {
        "Bmi088Source"
    }

    fn type_ident() -> Option<&'static str> {
        Some("Bmi088Source")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_bmi088")
    }

    fn module_path() -> Option<&'static str> {
        Some("")
    }
}

impl<SPI, ACC, GYR, D> Freezable for Bmi088Source<SPI, ACC, GYR, D>
where
    SPI: Transfer<u8> + Send + Sync + 'static,
    SPI::Error: Debug + Send + 'static,
    ACC: OutputPin + Send + Sync + 'static,
    ACC::Error: Debug + Send + 'static,
    GYR: OutputPin + Send + Sync + 'static,
    GYR::Error: Debug + Send + 'static,
    D: DelayMs<u32> + Send + Sync + 'static,
{
}

impl<SPI, ACC, GYR, D> CuSrcTask for Bmi088Source<SPI, ACC, GYR, D>
where
    SPI: Transfer<u8> + Send + Sync + 'static,
    SPI::Error: Debug + Send + 'static,
    ACC: OutputPin + Send + Sync + 'static,
    ACC::Error: Debug + Send + 'static,
    GYR: OutputPin + Send + Sync + 'static,
    GYR::Error: Debug + Send + 'static,
    D: DelayMs<u32> + Send + Sync + 'static,
{
    type Resources<'r> = Resources<SPI, ACC, GYR, D>;
    type Output<'m> = output_msg!(ImuPayload);

    fn new(_config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let driver = Bmi088Driver::new(
            resources.spi.0,
            resources.acc_cs.0,
            resources.gyr_cs.0,
            resources.delay.0,
        )?;
        Ok(Self { driver })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn process<'o>(&mut self, clock: &RobotClock, output: &mut Self::Output<'o>) -> CuResult<()> {
        let tov = clock.now();
        let payload = self.driver.read_measure()?;
        output.tov = Tov::Time(tov);
        output.set_payload(payload);
        Ok(())
    }
}

/// Low-level driver for the BMI088 IMU.
struct Bmi088Driver<SPI, ACC, GYR, D> {
    spi: SPI,
    acc_cs: ACC,
    gyr_cs: GYR,
    #[allow(dead_code)]
    delay: D,
    acc_mps2_per_lsb: f32,
}

impl<SPI, ACC, GYR, D> Bmi088Driver<SPI, ACC, GYR, D>
where
    SPI: Transfer<u8>,
    SPI::Error: Debug,
    ACC: OutputPin,
    ACC::Error: Debug,
    GYR: OutputPin,
    GYR::Error: Debug,
    D: DelayMs<u32>,
{
    /// Initialize the BMI088 driver.
    ///
    /// This performs:
    /// 1. Chip ID verification for both accelerometer and gyroscope
    /// 2. Soft reset of both sensors
    /// 3. Power-on and configuration of accelerometer
    /// 4. Configuration of gyroscope (2000 dps range, 2000 Hz ODR)
    fn new(mut spi: SPI, mut acc_cs: ACC, mut gyr_cs: GYR, mut delay: D) -> CuResult<Self> {
        // Ensure CS lines are high (deselected)
        acc_cs
            .set_high()
            .map_err(|err| map_error("bmi088 acc cs high", err))?;
        gyr_cs
            .set_high()
            .map_err(|err| map_error("bmi088 gyr cs high", err))?;

        // Verify chip IDs
        let acc_id = spi_read_reg_2(&mut spi, &mut acc_cs, BMI088_ACC_REG_CHIP_ID)
            .map_err(|err| map_error("bmi088 acc chip id", err))?;
        let gyr_id = spi_read_reg_1(&mut spi, &mut gyr_cs, BMI088_GYR_REG_CHIP_ID)
            .map_err(|err| map_error("bmi088 gyr chip id", err))?;

        if acc_id != BMI088_ACC_CHIP_ID {
            return Err(CuError::from("bmi088 accel id mismatch"));
        }
        if gyr_id != BMI088_GYR_CHIP_ID {
            return Err(CuError::from("bmi088 gyro id mismatch"));
        }

        // Reset accelerometer
        spi_write_reg(
            &mut spi,
            &mut acc_cs,
            BMI088_ACC_REG_SOFT_RESET,
            BMI088_SOFT_RESET_CMD,
        )
        .map_err(|err| map_error("bmi088 acc reset", err))?;
        delay.delay_ms(10);

        // Power on accelerometer
        spi_write_reg(
            &mut spi,
            &mut acc_cs,
            BMI088_ACC_REG_PWR_CONF,
            BMI088_ACC_PWR_CONF_ACTIVE,
        )
        .map_err(|err| map_error("bmi088 acc pwr conf", err))?;
        delay.delay_ms(1);
        spi_write_reg(
            &mut spi,
            &mut acc_cs,
            BMI088_ACC_REG_PWR_CTRL,
            BMI088_ACC_PWR_CTRL_ON,
        )
        .map_err(|err| map_error("bmi088 acc pwr ctrl", err))?;
        delay.delay_ms(50);

        // Reset and configure gyroscope
        spi_write_reg(
            &mut spi,
            &mut gyr_cs,
            BMI088_GYR_REG_SOFT_RESET,
            BMI088_SOFT_RESET_CMD,
        )
        .map_err(|err| map_error("bmi088 gyro reset", err))?;
        delay.delay_ms(100);
        spi_write_reg(
            &mut spi,
            &mut gyr_cs,
            BMI088_GYR_REG_RANGE,
            BMI088_GYR_RANGE_2000,
        )
        .map_err(|err| map_error("bmi088 gyro range", err))?;
        spi_write_reg(
            &mut spi,
            &mut gyr_cs,
            BMI088_GYR_REG_BANDWIDTH,
            BMI088_GYR_BANDWIDTH,
        )
        .map_err(|err| map_error("bmi088 gyro bandwidth", err))?;
        spi_write_reg(
            &mut spi,
            &mut gyr_cs,
            BMI088_GYR_REG_POWER_MODE,
            BMI088_GYR_PWR_NORMAL,
        )
        .map_err(|err| map_error("bmi088 gyro pwr mode", err))?;

        // Read accelerometer range for scaling
        let acc_range_reg = spi_read_reg_2(&mut spi, &mut acc_cs, BMI088_ACC_REG_ACC_RANGE)
            .map_err(|err| map_error("bmi088 acc range", err))?;
        let acc_range_g = accel_range_g_from_reg(acc_range_reg);
        let acc_mps2_per_lsb = acc_range_g * 9.806_65 / 32_768.0;
        debug!(
            "bmi088 accel range reg={} -> ±{}g",
            acc_range_reg, acc_range_g
        );

        Ok(Self {
            spi,
            acc_cs,
            gyr_cs,
            delay,
            acc_mps2_per_lsb,
        })
    }

    /// Read accelerometer, gyroscope, and temperature data.
    ///
    /// Returns an [`ImuPayload`] with:
    /// - Acceleration in m/s² (NED frame)
    /// - Angular velocity in rad/s (NED frame)
    /// - Temperature in °C
    fn read_measure(&mut self) -> CuResult<ImuPayload> {
        // Read accelerometer (6 bytes: X, Y, Z as 16-bit little-endian)
        let mut acc_buf = [0_u8; 6];
        spi_read_burst_2(
            &mut self.spi,
            &mut self.acc_cs,
            BMI088_ACC_REG_ACCEL_X_LSB,
            &mut acc_buf,
        )
        .map_err(|err| map_error("bmi088 acc burst", err))?;
        let ax = bytes_to_i16(acc_buf[0], acc_buf[1]);
        let ay = bytes_to_i16(acc_buf[2], acc_buf[3]);
        let az = bytes_to_i16(acc_buf[4], acc_buf[5]);

        // Read gyroscope (6 bytes: X, Y, Z as 16-bit little-endian)
        let mut gyr_buf = [0_u8; 6];
        spi_read_burst_1(
            &mut self.spi,
            &mut self.gyr_cs,
            BMI088_GYR_REG_X_LSB,
            &mut gyr_buf,
        )
        .map_err(|err| map_error("bmi088 gyro burst", err))?;
        let gx = bytes_to_i16(gyr_buf[0], gyr_buf[1]);
        let gy = bytes_to_i16(gyr_buf[2], gyr_buf[3]);
        let gz = bytes_to_i16(gyr_buf[4], gyr_buf[5]);

        // Read temperature (11-bit signed, split across 2 registers)
        let temp_msb = spi_read_reg_2(&mut self.spi, &mut self.acc_cs, BMI088_ACC_REG_TEMP_MSB)
            .map_err(|err| map_error("bmi088 temp msb", err))?;
        let temp_lsb = spi_read_reg_2(&mut self.spi, &mut self.acc_cs, BMI088_ACC_REG_TEMP_LSB)
            .map_err(|err| map_error("bmi088 temp lsb", err))?;
        let temp_raw = (temp_msb as i16) * 8 + (temp_lsb as i16) / 32;
        let temp_c = (temp_raw as f32) * 0.125 + 23.0;

        // Remap BMI088 axes into NED body frame: +X forward, +Y right, +Z down.
        // This mapping depends on how the sensor is mounted on your board.
        let accel_mps2 = [
            accel_raw_to_mps2(ay, self.acc_mps2_per_lsb),
            accel_raw_to_mps2(ax, self.acc_mps2_per_lsb),
            accel_raw_to_mps2(az, self.acc_mps2_per_lsb),
        ];
        let gyro_rad = [
            -gyro_raw_to_rad(gy),
            -gyro_raw_to_rad(gx),
            -gyro_raw_to_rad(gz),
        ];

        Ok(ImuPayload::from_raw(accel_mps2, gyro_rad, temp_c))
    }
}

// SPI helper types and functions

#[derive(Debug)]
enum SpiCsError<SpiErr, CsErr> {
    Spi(SpiErr),
    Cs(CsErr),
}

fn map_error<E: Debug>(context: &'static str, _err: E) -> CuError {
    CuError::from(context)
}

fn spi_transfer<SPI, CS>(
    spi: &mut SPI,
    cs: &mut CS,
    buf: &mut [u8],
) -> Result<(), SpiCsError<SPI::Error, CS::Error>>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
{
    cs.set_low().map_err(SpiCsError::Cs)?;
    let transfer_res = spi.transfer(buf).map_err(SpiCsError::Spi);
    let cs_res = cs.set_high().map_err(SpiCsError::Cs);
    if let Err(err) = transfer_res {
        let _ = cs_res;
        return Err(err);
    }
    cs_res?;
    Ok(())
}

fn spi_write_reg<SPI, CS>(
    spi: &mut SPI,
    cs: &mut CS,
    reg: u8,
    value: u8,
) -> Result<(), SpiCsError<SPI::Error, CS::Error>>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
{
    let mut buf = [reg & 0x7f, value];
    spi_transfer(spi, cs, &mut buf)
}

/// Read a single register using standard SPI protocol (for gyroscope).
fn spi_read_reg_1<SPI, CS>(
    spi: &mut SPI,
    cs: &mut CS,
    reg: u8,
) -> Result<u8, SpiCsError<SPI::Error, CS::Error>>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
{
    let mut buf = [reg | 0x80, 0x00];
    spi_transfer(spi, cs, &mut buf)?;
    Ok(buf[1])
}

/// Read a single register with dummy byte (for accelerometer).
fn spi_read_reg_2<SPI, CS>(
    spi: &mut SPI,
    cs: &mut CS,
    reg: u8,
) -> Result<u8, SpiCsError<SPI::Error, CS::Error>>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
{
    let mut buf = [reg | 0x80, 0x00, 0x00];
    spi_transfer(spi, cs, &mut buf)?;
    Ok(buf[2])
}

/// Burst read 6 bytes using standard SPI protocol (for gyroscope).
fn spi_read_burst_1<SPI, CS>(
    spi: &mut SPI,
    cs: &mut CS,
    reg: u8,
    out: &mut [u8; 6],
) -> Result<(), SpiCsError<SPI::Error, CS::Error>>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
{
    let mut buf = [0_u8; 7];
    buf[0] = reg | 0x80;
    spi_transfer(spi, cs, &mut buf)?;
    out.copy_from_slice(&buf[1..7]);
    Ok(())
}

/// Burst read 6 bytes with dummy byte (for accelerometer).
fn spi_read_burst_2<SPI, CS>(
    spi: &mut SPI,
    cs: &mut CS,
    reg: u8,
    out: &mut [u8; 6],
) -> Result<(), SpiCsError<SPI::Error, CS::Error>>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
{
    let mut buf = [0_u8; 8];
    buf[0] = reg | 0x80;
    spi_transfer(spi, cs, &mut buf)?;
    out.copy_from_slice(&buf[2..8]);
    Ok(())
}

// Conversion helpers

fn bytes_to_i16(lsb: u8, msb: u8) -> i16 {
    i16::from_le_bytes([lsb, msb])
}

fn accel_raw_to_mps2(raw: i16, acc_mps2_per_lsb: f32) -> f32 {
    raw as f32 * acc_mps2_per_lsb
}

fn gyro_raw_to_rad(raw: i16) -> f32 {
    raw as f32 * GYRO_RAD_PER_LSB
}

fn accel_range_g_from_reg(range_reg: u8) -> f32 {
    match range_reg & 0x03 {
        0x00 => 3.0,
        0x01 => 6.0,
        0x02 => 12.0,
        _ => 24.0,
    }
}
