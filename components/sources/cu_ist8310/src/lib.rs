//! Copper source driver for the iSentek IST8310 digital magnetometer.
//!
//! This source expects an address-scoped I2C resource so the sensor address is
//! defined by the board resource bundle (hardware mapping), not by component config.

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::{format, string::String};
use core::fmt::Debug;

pub use cu_sensor_payloads::MagnetometerPayload;
use cu29::prelude::*;
use cu29::units::si::magnetic_flux_density::microtesla;

// Registers
const REG_WHOAMI: u8 = 0x00;
const REG_STAT1: u8 = 0x02;
const REG_DATA_X_L: u8 = 0x03;
const REG_CNTRL1: u8 = 0x0A;
const REG_AVGCNTL: u8 = 0x41;
const REG_PDCNTL: u8 = 0x42;

// IDs / bitfields
const IST8310_CHIP_ID: u8 = 0x10;
const STAT1_DRDY: u8 = 0x01;

// Configuration
const CNTRL1_SINGLE_MEASURE: u8 = 0x01;
const AVGCNTL_16X: u8 = 0x24;
const PDCNTL_PULSE_DURATION_NORMAL: u8 = 0xC0;

// 0.3 uT / LSB = 3 mG / LSB
const UT_PER_LSB: f32 = 0.3;

const DETECT_RETRY_LIMIT: usize = 5_000;
const MEASURE_READY_POLL_LIMIT: usize = 10_000;
const I2C_TRANSFER_RETRY_LIMIT: usize = 8;
const OUTPUT_RATE_HZ: u64 = 100;
const OUTPUT_PERIOD_NS: u64 = 1_000_000_000 / OUTPUT_RATE_HZ;
const DRIVER_LOG_PERIOD_NS: u64 = 1_000_000_000;

/// Address-scoped bus interface for IST8310 register I/O.
///
/// Implement this trait in your board bundle resource type so the driver does
/// not carry or configure an I2C address.
pub trait Ist8310Bus: Send + Sync + 'static {
    type Error: Debug + Send + 'static;

    fn write(&mut self, write: &[u8]) -> Result<(), Self::Error>;
    fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error>;
}

struct Ist8310Driver<BUS>
where
    BUS: Ist8310Bus,
{
    bus: BUS,
}

impl<BUS> Ist8310Driver<BUS>
where
    BUS: Ist8310Bus,
{
    fn new(bus: BUS) -> CuResult<Self> {
        let mut driver = Self { bus };
        driver.detect_chip_id()?;
        driver.configure()?;
        Ok(driver)
    }

    fn detect_chip_id(&mut self) -> CuResult<()> {
        let mut last_id: Option<u8> = None;
        let mut read_buf = [0u8; 1];

        for _ in 0..DETECT_RETRY_LIMIT {
            if self.read_reg_buf(REG_WHOAMI, &mut read_buf).is_ok() {
                let id = read_buf[0];
                last_id = Some(id);
                if id == IST8310_CHIP_ID {
                    debug!("ist8310: detected id=0x{:02X}", id);
                    return Ok(());
                }
            }
            backoff_spin();
        }

        let last_id_str = match last_id {
            Some(id) => format!("0x{id:02X}"),
            None => String::from("none"),
        };
        Err(CuError::from(format!(
            "ist8310 detect failed: expected=0x{IST8310_CHIP_ID:02X} last_id={last_id_str}"
        )))
    }

    fn configure(&mut self) -> CuResult<()> {
        // Datasheet recommends setting pulse duration first in standby mode.
        self.write_reg_value(REG_AVGCNTL, AVGCNTL_16X)
            .map_err(|err| map_debug_error("ist8310 configure AVGCNTL", err))?;
        self.write_reg_value(REG_PDCNTL, PDCNTL_PULSE_DURATION_NORMAL)
            .map_err(|err| map_debug_error("ist8310 configure PDCNTL", err))?;
        debug!("ist8310: configured AVG16 and pulse duration normal");
        Ok(())
    }

    fn read_measure(&mut self) -> CuResult<MagnetometerPayload> {
        self.write_reg_value(REG_CNTRL1, CNTRL1_SINGLE_MEASURE)
            .map_err(|err| map_debug_error("ist8310 trigger single measure", err))?;
        self.wait_data_ready()?;

        let mut buf = [0u8; 6];
        self.read_reg_buf(REG_DATA_X_L, &mut buf)?;

        let x_raw = i16::from_le_bytes([buf[0], buf[1]]);
        let y_raw = i16::from_le_bytes([buf[2], buf[3]]);
        let z_raw = i16::from_le_bytes([buf[4], buf[5]]);

        // IST8310 board/alignment conventions in BF/INAV invert Y to right-hand system.
        let mag_x = (x_raw as f32) * UT_PER_LSB;
        let mag_y = -(y_raw as f32) * UT_PER_LSB;
        let mag_z = (z_raw as f32) * UT_PER_LSB;

        Ok(MagnetometerPayload::from_raw([mag_x, mag_y, mag_z]))
    }

    fn wait_data_ready(&mut self) -> CuResult<()> {
        let mut last_status: Option<u8> = None;
        let mut read_errors = 0u32;
        for _ in 0..MEASURE_READY_POLL_LIMIT {
            let status = match self.read_reg(REG_STAT1) {
                Ok(v) => v,
                Err(_) => {
                    read_errors = read_errors.saturating_add(1);
                    backoff_spin();
                    continue;
                }
            };
            last_status = Some(status);
            if (status & STAT1_DRDY) != 0 {
                return Ok(());
            }
            backoff_spin();
        }

        let last_status_str = match last_status {
            Some(status) => format!("0x{status:02X}"),
            None => String::from("none"),
        };
        Err(CuError::from(format!(
            "ist8310 data-ready timeout: last_stat1={last_status_str} read_errors={read_errors}"
        )))
    }

    fn read_reg(&mut self, reg: u8) -> CuResult<u8> {
        let mut byte = [0u8; 1];
        self.read_reg_buf(reg, &mut byte)?;
        Ok(byte[0])
    }

    fn read_reg_buf(&mut self, reg: u8, read: &mut [u8]) -> CuResult<()> {
        for _ in 0..I2C_TRANSFER_RETRY_LIMIT {
            if self.bus.write_read(&[reg], read).is_ok() {
                return Ok(());
            }
            backoff_spin();
        }
        Err(CuError::from(format!(
            "ist8310 i2c write_read failed: reg=0x{reg:02X} len={} retries={I2C_TRANSFER_RETRY_LIMIT}",
            read.len()
        )))
    }

    fn write_reg_value(&mut self, reg: u8, value: u8) -> CuResult<()> {
        for _ in 0..I2C_TRANSFER_RETRY_LIMIT {
            if self.bus.write(&[reg, value]).is_ok() {
                return Ok(());
            }
            backoff_spin();
        }
        Err(CuError::from(format!(
            "ist8310 i2c write failed: reg=0x{reg:02X} value=0x{value:02X} retries={I2C_TRANSFER_RETRY_LIMIT}"
        )))
    }
}

fn backoff_spin() {
    spin_wait(128);
}

fn spin_wait(iterations: usize) {
    for _ in 0..iterations {
        core::hint::spin_loop();
    }
}

fn map_debug_error<E: Debug>(context: &str, err: E) -> CuError {
    CuError::from(format!("{context}: {err:?}"))
}

resources!(for <BUS>
where
    BUS: Ist8310Bus,
{
    i2c => Owned<BUS>,
});

/// Copper source task for IST8310.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct Ist8310Source<BUS>
where
    BUS: Ist8310Bus,
{
    #[reflect(ignore)]
    driver: Ist8310Driver<BUS>,
    last_output_ns: Option<u64>,
    last_log_ns: Option<u64>,
}

impl<BUS> TypePath for Ist8310Source<BUS>
where
    BUS: Ist8310Bus,
{
    fn type_path() -> &'static str {
        "cu_ist8310::Ist8310Source"
    }

    fn short_type_path() -> &'static str {
        "Ist8310Source"
    }

    fn type_ident() -> Option<&'static str> {
        Some("Ist8310Source")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_ist8310")
    }

    fn module_path() -> Option<&'static str> {
        Some("")
    }
}

impl<BUS> Freezable for Ist8310Source<BUS> where BUS: Ist8310Bus {}

impl<BUS> CuSrcTask for Ist8310Source<BUS>
where
    BUS: Ist8310Bus,
{
    type Resources<'r> = Resources<BUS>;
    type Output<'m> = output_msg!(MagnetometerPayload);

    fn new(_config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let driver = Ist8310Driver::new(resources.i2c.0)?;
        Ok(Self {
            driver,
            last_output_ns: None,
            last_log_ns: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        debug!("ist8310: source started");
        Ok(())
    }

    fn process<'o>(&mut self, clock: &RobotClock, output: &mut Self::Output<'o>) -> CuResult<()> {
        let tov = clock.now();
        let now_ns = tov.as_nanos();

        if let Some(last_output_ns) = self.last_output_ns
            && now_ns.saturating_sub(last_output_ns) < OUTPUT_PERIOD_NS
        {
            output.clear_payload();
            output.tov = Tov::None;
            return Ok(());
        }

        let payload = self.driver.read_measure()?;
        self.last_output_ns = Some(now_ns);

        if self
            .last_log_ns
            .is_none_or(|last_log_ns| now_ns.saturating_sub(last_log_ns) >= DRIVER_LOG_PERIOD_NS)
        {
            self.last_log_ns = Some(now_ns);
            info!(
                "ist8310: mag_x_ut={} mag_y_ut={} mag_z_ut={} rate_hz={}",
                payload.mag_x.get::<microtesla>(),
                payload.mag_y.get::<microtesla>(),
                payload.mag_z.get::<microtesla>(),
                OUTPUT_RATE_HZ
            );
        }

        output.tov = Tov::Time(tov);
        output.set_payload(payload);
        Ok(())
    }
}
