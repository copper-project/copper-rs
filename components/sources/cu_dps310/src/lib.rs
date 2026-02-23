//! Copper source driver for the Infineon DPS310 digital barometric pressure sensor.
//!
//! This source expects an address-scoped I2C resource so the sensor address is
//! defined by the board resource bundle (hardware mapping), not by component config.

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::{format, string::String};
use core::fmt::Debug;

pub use cu_sensor_payloads::BarometerPayload;
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

// Registers
const REG_PRS_B2: u8 = 0x00;
const REG_PRS_CFG: u8 = 0x06;
const REG_TMP_CFG: u8 = 0x07;
const REG_MEAS_CFG: u8 = 0x08;
const REG_CFG_REG: u8 = 0x09;
const REG_RESET: u8 = 0x0C;
const REG_ID: u8 = 0x0D;
const REG_TMP_COEF_FIX_KEY1: u8 = 0x0E;
const REG_TMP_COEF_FIX_KEY2: u8 = 0x0F;
const REG_TMP_COEF_FIX_CTRL: u8 = 0x62;
const REG_COEF: u8 = 0x10;
const REG_COEF_SRCE: u8 = 0x28;

// IDs
const DPS310_ID_REV_AND_PROD: u8 = 0x10;
const SPL07_003_CHIP_ID: u8 = 0x11;

// Bitfields
const RESET_SOFT_RST: u8 = 0x09;
const MEAS_CFG_COEF_RDY: u8 = 1 << 7;
const MEAS_CFG_SENSOR_RDY: u8 = 1 << 6;
const MEAS_CFG_TMP_RDY: u8 = 1 << 5;
const MEAS_CFG_MEAS_CTRL_IDLE: u8 = 0x00;
const MEAS_CFG_MEAS_CTRL_TEMP_SING: u8 = 0x02;
const MEAS_CFG_MEAS_CTRL_CONT_P_T: u8 = 0x07;

const PRS_CFG_RATE_32HZ: u8 = 0x50;
const PRS_CFG_PRC_16X: u8 = 0x04;
const TMP_CFG_RATE_32HZ: u8 = 0x50;
const TMP_CFG_PRC_16X: u8 = 0x04;
const COEF_SRCE_TMP_COEF_SRCE: u8 = 0x80;

const CFG_REG_T_SHIFT: u8 = 0x08;
const CFG_REG_P_SHIFT: u8 = 0x04;

// 16x OSR compensation scales
const SCALE_KP_16X: f32 = 253_952.0;
const SCALE_KT_16X: f32 = 253_952.0;

const INIT_READY_POLL_LIMIT: usize = 10_000;
const TEMP_READY_POLL_LIMIT: usize = 10_000;
const DETECT_RETRY_LIMIT: usize = 5_000;
const I2C_TRANSFER_RETRY_LIMIT: usize = 8;
const RESET_SETTLE_SPINS: usize = 8_000_000;
const COEF_CHUNK_READ_LEN: usize = 9;
const OUTPUT_RATE_HZ: u64 = 30;
const OUTPUT_PERIOD_NS: u64 = 1_000_000_000 / OUTPUT_RATE_HZ;
const DRIVER_LOG_PERIOD_NS: u64 = 1_000_000_000;

/// Address-scoped bus interface for DPS310 register I/O.
///
/// Implement this trait in your board bundle resource type so the driver does
/// not carry or configure an I2C address.
pub trait Dps310Bus: Send + Sync + 'static {
    type Error: Debug + Send + 'static;

    fn write(&mut self, write: &[u8]) -> Result<(), Self::Error>;
    fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error>;
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
struct CalibrationCoefficients {
    c0: i16,
    c1: i16,
    c00: i32,
    c10: i32,
    c01: i16,
    c11: i16,
    c20: i16,
    c21: i16,
    c30: i16,
    c31: i16,
    c40: i16,
}

struct Dps310Driver<BUS>
where
    BUS: Dps310Bus,
{
    bus: BUS,
    chip_id: u8,
    calib: CalibrationCoefficients,
}

impl<BUS> Dps310Driver<BUS>
where
    BUS: Dps310Bus,
{
    fn new(bus: BUS) -> CuResult<Self> {
        let mut driver = Self {
            bus,
            chip_id: 0,
            calib: CalibrationCoefficients::default(),
        };

        driver.chip_id = driver.detect_chip_id()?;
        debug!("dps310: detected id=0x{:02X}", driver.chip_id);
        driver.soft_reset()?;
        driver.wait_sensor_ready()?;
        driver.read_calibration_coefficients()?;
        driver.configure()?;
        Ok(driver)
    }

    fn detect_chip_id(&mut self) -> CuResult<u8> {
        let mut last_err: Option<&'static str> = None;
        let mut last_id: Option<u8> = None;
        let mut read_buf = [0u8; 1];
        for _ in 0..DETECT_RETRY_LIMIT {
            match self.bus.write_read(&[REG_ID], &mut read_buf) {
                Ok(()) => {
                    let id = read_buf[0];
                    last_id = Some(id);
                    if id == DPS310_ID_REV_AND_PROD || id == SPL07_003_CHIP_ID {
                        return Ok(id);
                    }
                    last_err = Some("unexpected chip id");
                }
                Err(_) => {
                    last_err = Some("i2c read error");
                }
            }
            backoff_spin();
        }
        let last_id_str = match last_id {
            Some(id) => format!("0x{:02X}", id),
            None => String::from("none"),
        };
        Err(CuError::from(format!(
            "dps310 detect failed: {} last_id={}",
            last_err.unwrap_or("unknown"),
            last_id_str
        )))
    }

    fn soft_reset(&mut self) -> CuResult<()> {
        self.write_reg_value(REG_RESET, RESET_SOFT_RST)
            .map_err(|err| map_debug_error("dps310 soft reset", err))?;
        // Match INAV/BF behavior: allow the sensor reset sequence to settle
        // before polling status or touching configuration registers.
        spin_wait(RESET_SETTLE_SPINS);
        Ok(())
    }

    fn wait_sensor_ready(&mut self) -> CuResult<()> {
        let mut last_status: Option<u8> = None;
        let mut read_errors = 0u32;
        for _ in 0..INIT_READY_POLL_LIMIT {
            let status = match self.read_reg(REG_MEAS_CFG) {
                Ok(v) => v,
                Err(_) => {
                    read_errors = read_errors.saturating_add(1);
                    backoff_spin();
                    continue;
                }
            };
            last_status = Some(status);
            let coeff_ready = (status & MEAS_CFG_COEF_RDY) != 0;
            let sensor_ready = (status & MEAS_CFG_SENSOR_RDY) != 0;
            if coeff_ready && sensor_ready {
                return Ok(());
            }
            backoff_spin();
        }
        let last_status_str = match last_status {
            Some(status) => format!("0x{:02X}", status),
            None => String::from("none"),
        };
        Err(CuError::from(format!(
            "dps310 init timeout: COEF_RDY/SENSOR_RDY not both set, last_meas_cfg={}, read_errors={}",
            last_status_str, read_errors
        )))
    }

    fn read_calibration_coefficients(&mut self) -> CuResult<()> {
        let coef_len = if self.chip_id == SPL07_003_CHIP_ID {
            22
        } else {
            18
        };
        let mut coef = [0u8; 22];
        let mut loaded = false;

        // Fast path: single contiguous read.
        if self.read_reg_buf(REG_COEF, &mut coef[..coef_len]).is_ok() {
            loaded = true;
        }

        // Fallback: chunked reads (Betaflight-style).
        if !loaded {
            let mut offset = 0usize;
            loaded = true;
            while offset < coef_len {
                let chunk = core::cmp::min(COEF_CHUNK_READ_LEN, coef_len - offset);
                let mut buf = [0u8; COEF_CHUNK_READ_LEN];
                if self
                    .read_reg_buf(REG_COEF + offset as u8, &mut buf[..chunk])
                    .is_err()
                {
                    loaded = false;
                    break;
                }
                coef[offset..offset + chunk].copy_from_slice(&buf[..chunk]);
                offset += chunk;
            }
        }

        // Final fallback: byte-wise reads for controllers/sensors that NACK bursts.
        if !loaded {
            debug!("dps310: coef burst read failed, falling back to byte reads");
            for (i, slot) in coef[..coef_len].iter_mut().enumerate() {
                *slot = self.read_reg(REG_COEF + i as u8)?;
            }
        }

        self.calib = parse_coefficients(&coef, self.chip_id == SPL07_003_CHIP_ID);
        debug!(
            "dps310: coeff c00={} c10={} c01={} c11={} c20={} c21={} c30={}",
            self.calib.c00,
            self.calib.c10,
            self.calib.c01,
            self.calib.c11,
            self.calib.c20,
            self.calib.c21,
            self.calib.c30
        );
        Ok(())
    }

    fn configure(&mut self) -> CuResult<()> {
        self.write_reg_value(REG_MEAS_CFG, MEAS_CFG_MEAS_CTRL_IDLE)
            .map_err(|err| map_debug_error("dps310 configure MEAS_CFG idle", err))?;

        // INAV/BF compatibility workaround for known temperature coefficient issue.
        self.write_reg_value(REG_TMP_COEF_FIX_KEY1, 0xA5)
            .map_err(|err| map_debug_error("dps310 fix write 0x0E", err))?;
        self.write_reg_value(REG_TMP_COEF_FIX_KEY2, 0x96)
            .map_err(|err| map_debug_error("dps310 fix write 0x0F", err))?;
        self.write_reg_value(REG_TMP_COEF_FIX_CTRL, 0x02)
            .map_err(|err| map_debug_error("dps310 fix write 0x62", err))?;
        self.write_reg_value(REG_TMP_COEF_FIX_KEY1, 0x00)
            .map_err(|err| map_debug_error("dps310 fix clear 0x0E", err))?;
        self.write_reg_value(REG_TMP_COEF_FIX_KEY2, 0x00)
            .map_err(|err| map_debug_error("dps310 fix clear 0x0F", err))?;

        self.write_reg_value(REG_MEAS_CFG, MEAS_CFG_MEAS_CTRL_TEMP_SING)
            .map_err(|err| map_debug_error("dps310 configure MEAS_CFG temp single", err))?;
        self.wait_temp_ready()?;

        self.set_bits(REG_PRS_CFG, PRS_CFG_RATE_32HZ | PRS_CFG_PRC_16X)
            .map_err(|err| map_debug_error("dps310 configure PRS_CFG", err))?;

        let temp_coef_source = if self.chip_id == SPL07_003_CHIP_ID {
            0
        } else {
            self.read_reg(REG_COEF_SRCE)? & COEF_SRCE_TMP_COEF_SRCE
        };
        self.set_bits(
            REG_TMP_CFG,
            TMP_CFG_RATE_32HZ | TMP_CFG_PRC_16X | temp_coef_source,
        )
        .map_err(|err| map_debug_error("dps310 configure TMP_CFG", err))?;

        self.set_bits(REG_CFG_REG, CFG_REG_P_SHIFT | CFG_REG_T_SHIFT)
            .map_err(|err| map_debug_error("dps310 configure CFG_REG", err))?;

        self.write_reg_value(REG_MEAS_CFG, MEAS_CFG_MEAS_CTRL_CONT_P_T)
            .map_err(|err| map_debug_error("dps310 configure MEAS_CFG cont", err))?;

        debug!("dps310: configured background P+T mode (32Hz, 16x OSR)");
        Ok(())
    }

    fn wait_temp_ready(&mut self) -> CuResult<()> {
        let mut last_status: Option<u8> = None;
        let mut read_errors = 0u32;
        for _ in 0..TEMP_READY_POLL_LIMIT {
            let status = match self.read_reg(REG_MEAS_CFG) {
                Ok(v) => v,
                Err(_) => {
                    read_errors = read_errors.saturating_add(1);
                    backoff_spin();
                    continue;
                }
            };
            last_status = Some(status);
            if (status & MEAS_CFG_TMP_RDY) != 0 {
                return Ok(());
            }
            backoff_spin();
        }
        let last_status_str = match last_status {
            Some(status) => format!("0x{:02X}", status),
            None => String::from("none"),
        };
        Err(CuError::from(format!(
            "dps310 temp-ready timeout: last_meas_cfg={}, read_errors={}",
            last_status_str, read_errors
        )))
    }

    fn read_measure(&mut self) -> CuResult<BarometerPayload> {
        let mut buf = [0u8; 6];
        self.read_reg_buf(REG_PRS_B2, &mut buf)?;

        let pressure_raw = twos_complement(
            ((buf[0] as u32) << 16) | ((buf[1] as u32) << 8) | (buf[2] as u32),
            24,
        );
        let temperature_raw = twos_complement(
            ((buf[3] as u32) << 16) | ((buf[4] as u32) << 8) | (buf[5] as u32),
            24,
        );

        let pressure = compensate_pressure_pa(
            &self.calib,
            pressure_raw,
            temperature_raw,
            self.chip_id == SPL07_003_CHIP_ID,
        );
        let temperature = compensate_temperature_c(&self.calib, temperature_raw);

        Ok(BarometerPayload::from_raw(pressure, temperature))
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
            "dps310 i2c write_read failed: reg=0x{:02X} len={} retries={}",
            reg,
            read.len(),
            I2C_TRANSFER_RETRY_LIMIT
        )))
    }

    fn set_bits(&mut self, reg: u8, bits: u8) -> CuResult<()> {
        let mut value = self.read_reg(reg)?;
        if value & bits != bits {
            value |= bits;
            self.write_reg_value(reg, value)?;
        }
        Ok(())
    }

    fn write_reg_value(&mut self, reg: u8, value: u8) -> CuResult<()> {
        for _ in 0..I2C_TRANSFER_RETRY_LIMIT {
            if self.bus.write(&[reg, value]).is_ok() {
                return Ok(());
            }
            backoff_spin();
        }
        Err(CuError::from(format!(
            "dps310 i2c write failed: reg=0x{:02X} value=0x{:02X} retries={}",
            reg, value, I2C_TRANSFER_RETRY_LIMIT
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

fn parse_coefficients(raw: &[u8; 22], is_spl07: bool) -> CalibrationCoefficients {
    let c31 = if is_spl07 {
        twos_complement(
            ((raw[18] as u32) << 4) | (((raw[19] as u32) >> 4) & 0x0F),
            12,
        ) as i16
    } else {
        0
    };
    let c40 = if is_spl07 {
        twos_complement((((raw[19] as u32) & 0x0F) << 8) | (raw[20] as u32), 12) as i16
    } else {
        0
    };

    CalibrationCoefficients {
        c0: twos_complement(((raw[0] as u32) << 4) | (((raw[1] as u32) >> 4) & 0x0F), 12) as i16,
        c1: twos_complement((((raw[1] as u32) & 0x0F) << 8) | (raw[2] as u32), 12) as i16,
        c00: twos_complement(
            ((raw[3] as u32) << 12) | ((raw[4] as u32) << 4) | (((raw[5] as u32) >> 4) & 0x0F),
            20,
        ),
        c10: twos_complement(
            (((raw[5] as u32) & 0x0F) << 16) | ((raw[6] as u32) << 8) | (raw[7] as u32),
            20,
        ),
        c01: twos_complement(((raw[8] as u32) << 8) | (raw[9] as u32), 16) as i16,
        c11: twos_complement(((raw[10] as u32) << 8) | (raw[11] as u32), 16) as i16,
        c20: twos_complement(((raw[12] as u32) << 8) | (raw[13] as u32), 16) as i16,
        c21: twos_complement(((raw[14] as u32) << 8) | (raw[15] as u32), 16) as i16,
        c30: twos_complement(((raw[16] as u32) << 8) | (raw[17] as u32), 16) as i16,
        c31,
        c40,
    }
}

fn compensate_temperature_c(calib: &CalibrationCoefficients, temperature_raw: i32) -> f32 {
    let t_raw_sc = (temperature_raw as f32) / SCALE_KT_16X;
    (calib.c0 as f32) * 0.5 + (calib.c1 as f32) * t_raw_sc
}

fn compensate_pressure_pa(
    calib: &CalibrationCoefficients,
    pressure_raw: i32,
    temperature_raw: i32,
    is_spl07: bool,
) -> f32 {
    let p_raw_sc = (pressure_raw as f32) / SCALE_KP_16X;
    let t_raw_sc = (temperature_raw as f32) / SCALE_KT_16X;

    let c00 = calib.c00 as f32;
    let c10 = calib.c10 as f32;
    let c20 = calib.c20 as f32;
    let c30 = calib.c30 as f32;
    let c01 = calib.c01 as f32;
    let c11 = calib.c11 as f32;
    let c21 = calib.c21 as f32;

    if is_spl07 {
        let c31 = calib.c31 as f32;
        let c40 = calib.c40 as f32;
        c00 + p_raw_sc * (c10 + p_raw_sc * (c20 + p_raw_sc * (c30 + p_raw_sc * c40)))
            + t_raw_sc * c01
            + t_raw_sc * p_raw_sc * (c11 + p_raw_sc * (c21 + p_raw_sc * c31))
    } else {
        c00 + p_raw_sc * (c10 + p_raw_sc * (c20 + p_raw_sc * c30))
            + t_raw_sc * c01
            + t_raw_sc * p_raw_sc * (c11 + p_raw_sc * c21)
    }
}

fn twos_complement(raw: u32, bit_len: u8) -> i32 {
    if raw & (1u32 << (bit_len - 1)) != 0 {
        (raw as i32) - (1i32 << bit_len)
    } else {
        raw as i32
    }
}

fn map_debug_error<E: Debug>(context: &str, err: E) -> CuError {
    CuError::from(format!("{context}: {err:?}"))
}

resources!(for <BUS>
where
    BUS: Dps310Bus,
{
    i2c => Owned<BUS>,
});

/// Copper source task for DPS310.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct Dps310Source<BUS>
where
    BUS: Dps310Bus,
{
    #[reflect(ignore)]
    driver: Dps310Driver<BUS>,
    last_output_ns: Option<u64>,
    last_log_ns: Option<u64>,
}

impl<BUS> TypePath for Dps310Source<BUS>
where
    BUS: Dps310Bus,
{
    fn type_path() -> &'static str {
        "cu_dps310::Dps310Source"
    }

    fn short_type_path() -> &'static str {
        "Dps310Source"
    }

    fn type_ident() -> Option<&'static str> {
        Some("Dps310Source")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_dps310")
    }

    fn module_path() -> Option<&'static str> {
        Some("")
    }
}

impl<BUS> Freezable for Dps310Source<BUS> where BUS: Dps310Bus {}

impl<BUS> CuSrcTask for Dps310Source<BUS>
where
    BUS: Dps310Bus,
{
    type Resources<'r> = Resources<BUS>;
    type Output<'m> = output_msg!(BarometerPayload);

    fn new(_config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let driver = Dps310Driver::new(resources.i2c.0)?;
        Ok(Self {
            driver,
            last_output_ns: None,
            last_log_ns: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        debug!("dps310: source started");
        Ok(())
    }

    fn process<'o>(&mut self, clock: &CuContext, output: &mut Self::Output<'o>) -> CuResult<()> {
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
                "dps310: pressure_pa={} temp_c={} rate_hz={}",
                payload.pressure.value, payload.temperature.value, OUTPUT_RATE_HZ
            );
        }

        output.tov = Tov::Time(tov);
        output.set_payload(payload);
        Ok(())
    }
}
