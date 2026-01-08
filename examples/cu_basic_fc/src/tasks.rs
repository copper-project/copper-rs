#![allow(dead_code)]

use crate::messages::{BatteryVoltage, BodyCommand, BodyRateSetpoint, ControlInputs, FlightMode};
use alloc::vec::Vec;
use cu_ahrs::AhrsPose;
use cu_bdshot::EscCommand;
use cu_crsf::messages::RcChannelsPayload;
use cu_micoairh743::GreenLed;
use cu_msp_bridge::MspRequestBatch;
use cu_msp_lib::structs::{
    MspAnalog, MspApiVersion, MspBatteryConfig, MspBatteryState, MspDisplayPort,
    MspFlightControllerVersion, MspRequest, MspStatus, MspStatusSensors, MspVoltageMeter,
    MspVoltageMeterConfig,
};
use cu_pid::{PIDControlOutputPayload, PIDController};
use cu_sensor_payloads::ImuPayload;
use cu29::prelude::*;
use uom::si::angular_velocity::degree_per_second;
use uom::si::thermodynamic_temperature::degree_celsius;

mod bmi088;

const LOG_PERIOD_MS: u64 = 500;
const HEAP_LOG_PERIOD_MS: u64 = 500;
const VTX_HEARTBEAT_PERIOD_MS: u64 = 1000;
const VTX_DRAW_PERIOD_MS: u64 = 250;
// Matches Betaflight ARMING_DISABLE_FLAGS_COUNT (log2(ARM_SWITCH) + 1).
const MSP_ARMING_DISABLE_FLAGS_COUNT: u8 = 29;
const VTX_WATERMARK_LINES: [&str; 3] = [" /\\_/\\ ", "( O O )", " > ^ <  [CU29]"];
const VTX_CELL_DIVISOR: u16 = 4;
const VTX_SYM_VOLT: char = '\x06';
const BATTERY_VREF_MV_DEFAULT: u32 = 3300;
const BATTERY_VBAT_SCALE_DEFAULT: u32 = 210;
const BATTERY_VBAT_RES_DIV_VAL_DEFAULT: u32 = 10;
const BATTERY_VBAT_RES_DIV_MULT_DEFAULT: u32 = 1;
const BATTERY_MIN_CELL_CENTIVOLTS_DEFAULT: u16 = 330;
const BATTERY_MAX_CELL_CENTIVOLTS_DEFAULT: u16 = 420;
const BATTERY_WARN_CELL_CENTIVOLTS_DEFAULT: u16 = 350;
const BATTERY_VOLTAGE_METER_SOURCE_ADC: u8 = 1;
const BATTERY_CURRENT_METER_SOURCE_NONE: u8 = 0;
const MSP_VOLTAGE_METER_ID_BATTERY_1: u8 = 10;
const MSP_VOLTAGE_METER_SENSOR_TYPE_ADC_RES_DIV: u8 = 0;
const MSP_VOLTAGE_METER_ADC_SUBFRAME_LEN: u8 = 5;
const MSP_API_PROTOCOL_VERSION: u8 = 1;
const MSP_API_VERSION_MAJOR: u8 = 1;
const MSP_API_VERSION_MINOR: u8 = 47;
const MSP_FC_VERSION_MAJOR: u8 = 4;
const MSP_FC_VERSION_MINOR: u8 = 4;
const MSP_FC_VERSION_PATCH: u8 = 0;

resources!({
    led => Owned<spin::Mutex<GreenLed>>,
});

mod battery_resources {
    use super::*;

    resources!({
        battery_adc => Owned<cu_micoairh743::BatteryAdc>,
    });
}

struct LogRateLimiter {
    last: Option<CuTime>,
}

impl LogRateLimiter {
    const fn new() -> Self {
        Self { last: None }
    }

    fn should_log(&mut self, now: CuTime) -> bool {
        match self.last {
            Some(prev) if now - prev < CuDuration::from_millis(LOG_PERIOD_MS) => false,
            _ => {
                self.last = Some(now);
                true
            }
        }
    }
}

macro_rules! debug_rl {
    ($state:expr, $now:expr, { $($body:tt)* }) => {{
        let mut state = $state.lock();
        if state.should_log($now) {
            $($body)*
        }
    }};
    ($state:expr, $now:expr, $($arg:tt)+) => {{
        debug_rl!($state, $now, { debug!($($arg)+); });
    }};
}

static LOG_CTRL: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());
static LOG_TELEMETRY: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());
static LOG_IMU: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());
static LOG_RC: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());
static LOG_RATE: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());
static LOG_MOTORS: spin::Mutex<LogRateLimiter> = spin::Mutex::new(LogRateLimiter::new());

pub struct LedBeat {
    on: bool,
    led: spin::Mutex<GreenLed>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StatusLabel {
    Disarmed,
    Angle,
    Air,
    Position,
}

impl StatusLabel {
    const fn as_str(self) -> &'static str {
        match self {
            StatusLabel::Disarmed => " XXX ",
            StatusLabel::Angle => "ANGLE",
            StatusLabel::Air => " AIR ",
            StatusLabel::Position => " POS ",
        }
    }
}

pub struct BatteryAdcSource {
    adc: cu_micoairh743::BatteryAdc,
    vref_mv: u32,
    vbat_scale: u32,
    vbat_res_div_val: u32,
    vbat_res_div_mult: u32,
}

impl Freezable for BatteryAdcSource {}

impl CuSrcTask for BatteryAdcSource {
    type Resources<'r> = battery_resources::Resources;
    type Output<'m> = output_msg!(BatteryVoltage);

    fn new_with(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let vref_mv = cfg_u32(config, "vref_mv", BATTERY_VREF_MV_DEFAULT).max(1);
        let vbat_scale = cfg_u32(config, "vbat_scale", BATTERY_VBAT_SCALE_DEFAULT);
        let vbat_res_div_val =
            cfg_u32(config, "vbat_res_div_val", BATTERY_VBAT_RES_DIV_VAL_DEFAULT).max(1);
        let vbat_res_div_mult = cfg_u32(
            config,
            "vbat_res_div_mult",
            BATTERY_VBAT_RES_DIV_MULT_DEFAULT,
        )
        .max(1);
        Ok(Self {
            adc: resources.battery_adc.0,
            vref_mv,
            vbat_scale,
            vbat_res_div_val,
            vbat_res_div_mult,
        })
    }

    fn process(&mut self, clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        let now = clock.now();
        let raw = self.adc.read_raw_blocking();
        let slope = self.adc.slope().max(1);
        let raw_u64 = u64::from(raw);
        let slope_u64 = u64::from(slope);
        let vref_mv = u64::from(self.vref_mv);
        let vbat_scale = u64::from(self.vbat_scale);
        let vbat_res_div_val = u64::from(self.vbat_res_div_val);
        let vbat_res_div_mult = u64::from(self.vbat_res_div_mult);

        // Match Betaflight voltageAdcToVoltage: output in 0.01V steps.
        let denom = slope_u64.saturating_mul(vbat_res_div_val).max(1);
        let mut numer = raw_u64.saturating_mul(vbat_scale).saturating_mul(vref_mv);
        numer /= 10;
        let mut centivolts = (numer + denom / 2) / denom;
        centivolts /= vbat_res_div_mult.max(1);
        let centivolts = centivolts.min(u64::from(u16::MAX)) as u16;

        output.set_payload(BatteryVoltage { centivolts });
        output.tov = Tov::Time(now);
        debug_rl!(
            &LOG_TELEMETRY,
            now,
            "vbat adc raw={} slope={} vref_mv={} centivolts={}",
            raw,
            slope,
            self.vref_mv,
            centivolts
        );
        Ok(())
    }
}

pub struct VtxOsd {
    row: u8,
    cols: u8,
    rows: u8,
    col_center: u8,
    watermark_row: u8,
    watermark_col: u8,
    last_label: Option<StatusLabel>,
    last_heartbeat: Option<CuTime>,
    last_draw: Option<CuTime>,
    last_armed: bool,
    last_voltage_centi: Option<u16>,
}

impl Freezable for VtxOsd {}

impl CuTask for VtxOsd {
    type Input<'m> = input_msg!('m, ControlInputs, BatteryVoltage, MspRequestBatch);
    type Output<'m> = CuMsg<MspRequestBatch>;
    type Resources<'r> = ();

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cols = cfg_u16(config, "cols", 53).max(1).min(u8::MAX as u16) as u8;
        let rows = cfg_u16(config, "rows", 16).max(1).min(u8::MAX as u16) as u8;
        let default_center = (cols / 2) as u16;
        let col_center =
            cfg_u16(config, "col_center", default_center).min(cols.saturating_sub(1) as u16) as u8;
        let row = cfg_u16(config, "row", 13).min(u8::MAX as u16) as u8;
        let watermark_height = VTX_WATERMARK_LINES.len() as u8;
        let default_watermark_row = rows.saturating_sub(watermark_height);
        let watermark_row = cfg_u16(config, "watermark_row", default_watermark_row as u16)
            .min(rows.saturating_sub(1) as u16) as u8;
        let watermark_col =
            cfg_u16(config, "watermark_col", 0).min(cols.saturating_sub(1) as u16) as u8;
        Ok(Self {
            row,
            cols,
            rows,
            col_center,
            watermark_row,
            watermark_col,
            last_label: None,
            last_heartbeat: None,
            last_draw: None,
            last_armed: false,
            last_voltage_centi: None,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (ctrl_msg, batt_msg, incoming_msg) = *input;
        let tov_time = expect_tov_time(ctrl_msg.tov)?;
        output.tov = Tov::Time(tov_time);
        let now = tov_time;
        let mut batch = MspRequestBatch::new();
        let ctrl = ctrl_msg.payload();

        if let Some(voltage) = batt_msg.payload() {
            self.last_voltage_centi = Some(voltage.centivolts);
        }

        // First, handle any incoming MSP requests (telemetry queries from VTX)
        if let Some(incoming_requests) = incoming_msg.payload() {
            self.handle_incoming_requests(&mut batch, incoming_requests);
        }

        if let Some(ctrl) = ctrl {
            self.last_armed = ctrl.armed;
        }

        let heartbeat_due = self
            .last_heartbeat
            .map(|prev| now - prev >= CuDuration::from_millis(VTX_HEARTBEAT_PERIOD_MS))
            .unwrap_or(true);
        if heartbeat_due {
            batch.push(MspRequest::MspDisplayPort(MspDisplayPort::heartbeat()));
            batch.push(MspRequest::MspStatus(build_msp_status(self.last_armed)));
            self.last_heartbeat = Some(now);
        }

        let Some(ctrl) = ctrl else {
            if batch.0.is_empty() {
                output.clear_payload();
            } else {
                output.set_payload(batch);
            }
            return Ok(());
        };

        let label = if ctrl.armed {
            match ctrl.mode {
                FlightMode::Acro => StatusLabel::Air,
                FlightMode::Angle => StatusLabel::Angle,
                FlightMode::PositionHold => StatusLabel::Position,
            }
        } else {
            StatusLabel::Disarmed
        };

        let label_changed = self.last_label != Some(label);
        if label_changed {
            self.last_label = Some(label);
        }

        let text = label.as_str();
        let width = text.len() as u8;
        let col = if self.cols <= width {
            0
        } else {
            let half = width / 2;
            let mut col = self.col_center.saturating_sub(half);
            if col.saturating_add(width) > self.cols {
                col = self.cols.saturating_sub(width);
            }
            col
        };

        if label_changed {
            batch.push(MspRequest::MspDisplayPort(MspDisplayPort::clear_screen()));
            batch.push(MspRequest::MspDisplayPort(MspDisplayPort::write_string(
                self.row, col, 0, text,
            )));
            self.push_cell_voltage(&mut batch);
            self.push_watermark(&mut batch);
        }

        let draw_due = self
            .last_draw
            .map(|prev| now - prev >= CuDuration::from_millis(VTX_DRAW_PERIOD_MS))
            .unwrap_or(true);
        if label_changed || draw_due {
            if !label_changed {
                self.push_cell_voltage(&mut batch);
                self.push_watermark(&mut batch);
            }
            batch.push(MspRequest::MspDisplayPort(MspDisplayPort::draw_screen()));
            self.last_draw = Some(now);
        }

        if batch.0.is_empty() {
            output.clear_payload();
        } else {
            output.set_payload(batch);
        }
        Ok(())
    }
}

impl VtxOsd {
    fn handle_incoming_requests(&self, batch: &mut MspRequestBatch, requests: &MspRequestBatch) {
        let voltage_centi = self.last_voltage_centi.unwrap_or(0);
        let voltage_decivolts = clamp_u8((voltage_centi + 5) / 10);
        let cell_count = estimate_cell_count(voltage_centi);

        let battery_state = MspBatteryState {
            battery_cell_count: cell_count.unwrap_or(0),
            battery_capacity: 0,
            battery_voltage: voltage_decivolts,
            mah_drawn: 0,
            amperage: 0,
            alerts: 0,
            battery_voltage_mv: voltage_centi,
        };
        let analog = MspAnalog {
            battery_voltage: voltage_decivolts,
            mah_drawn: 0,
            rssi: 0,
            amperage: 0,
            battery_voltage_mv: voltage_centi,
        };
        let api_version = MspApiVersion {
            protocol_version: MSP_API_PROTOCOL_VERSION,
            api_version_major: MSP_API_VERSION_MAJOR,
            api_version_minor: MSP_API_VERSION_MINOR,
        };
        let fc_version = MspFlightControllerVersion {
            major: MSP_FC_VERSION_MAJOR,
            minor: MSP_FC_VERSION_MINOR,
            patch: MSP_FC_VERSION_PATCH,
        };

        for request in requests.iter() {
            match request {
                MspRequest::MspApiVersionRequest => {
                    batch.push(MspRequest::MspApiVersion(api_version));
                }
                MspRequest::MspFcVersionRequest => {
                    batch.push(MspRequest::MspFlightControllerVersion(fc_version));
                }
                MspRequest::MspBatteryStateRequest => {
                    batch.push(MspRequest::MspBatteryState(battery_state));
                }
                MspRequest::MspAnalogRequest => {
                    batch.push(MspRequest::MspAnalog(analog));
                }
                _ => {}
            }
        }
    }

    fn push_cell_voltage(&self, batch: &mut MspRequestBatch) {
        let row = self.row.saturating_add(1);
        if row >= self.rows {
            return;
        }
        let text = self.format_cell_voltage();
        let width = text.len() as u8;
        let col = if self.cols <= width {
            0
        } else {
            let half = width / 2;
            let mut col = self.col_center.saturating_sub(half);
            if col.saturating_add(width) > self.cols {
                col = self.cols.saturating_sub(width);
            }
            col
        };
        batch.push(MspRequest::MspDisplayPort(MspDisplayPort::write_string(
            row,
            col,
            0,
            text.as_str(),
        )));
    }

    fn format_cell_voltage(&self) -> alloc::string::String {
        match self.last_voltage_centi {
            Some(total_centi) => {
                let cell_centi = total_centi / VTX_CELL_DIVISOR;
                let whole = cell_centi / 100;
                let frac = cell_centi % 100;
                alloc::format!("{whole}.{frac:02}{VTX_SYM_VOLT}")
            }
            None => alloc::format!("--.--{VTX_SYM_VOLT}"),
        }
    }

    fn push_watermark(&self, batch: &mut MspRequestBatch) {
        if self.rows == 0 || self.cols == 0 {
            return;
        }
        let col = self.watermark_col.min(self.cols.saturating_sub(1));
        let available = self.cols.saturating_sub(col) as usize;
        if available == 0 {
            return;
        }
        for (idx, line) in VTX_WATERMARK_LINES.iter().enumerate() {
            let row = self.watermark_row.saturating_add(idx as u8);
            if row >= self.rows {
                break;
            }
            let mut text = *line;
            if text.len() > available {
                text = &text[..available];
            }
            if text.is_empty() {
                continue;
            }
            batch.push(MspRequest::MspDisplayPort(MspDisplayPort::write_string(
                row, col, 0, text,
            )));
        }
    }
}

fn build_msp_status(armed: bool) -> MspStatus {
    let mut flight_mode_flags = 0;
    if armed {
        flight_mode_flags |= 1;
    }

    MspStatus {
        cycle_time: 0,
        i2c_errors: 0,
        sensors: MspStatusSensors {
            acc: true,
            gyro: true,
            ..Default::default()
        },
        flight_mode_flags,
        current_pid_profile_index: 0,
        average_system_load_percent: 0,
        gyro_cycle_time: 0,
        extra_flight_mode_flags: Vec::new(),
        arming_disable_flags_count: MSP_ARMING_DISABLE_FLAGS_COUNT,
        arming_disable_flags: 0,
        config_state_flags: 0,
        core_temp_celsius: 0,
        control_rate_profile_count: 1,
    }
}

pub struct VtxMspResponder {
    last_voltage_centi: Option<u16>,
    battery_cells: Option<u8>,
    vbat_scale: u8,
    vbat_res_div_val: u8,
    vbat_res_div_mult: u8,
    battery_capacity: u16,
    vbat_min_cell_centivolts: u16,
    vbat_max_cell_centivolts: u16,
    vbat_warn_cell_centivolts: u16,
}

impl Freezable for VtxMspResponder {}

impl CuTask for VtxMspResponder {
    type Input<'m> = input_msg!('m, MspRequestBatch, BatteryVoltage);
    type Output<'m> = CuMsg<MspRequestBatch>;
    type Resources<'r> = ();

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let vbat_scale =
            cfg_u32(config, "vbat_scale", BATTERY_VBAT_SCALE_DEFAULT).min(u32::from(u8::MAX)) as u8;
        let vbat_res_div_val = cfg_u32(config, "vbat_res_div_val", BATTERY_VBAT_RES_DIV_VAL_DEFAULT)
            .max(1)
            .min(u32::from(u8::MAX)) as u8;
        let vbat_res_div_mult = cfg_u32(
            config,
            "vbat_res_div_mult",
            BATTERY_VBAT_RES_DIV_MULT_DEFAULT,
        )
        .max(1)
        .min(u32::from(u8::MAX)) as u8;
        let battery_capacity = cfg_u16(config, "battery_capacity_mah", 0);
        let vbat_min_cell_centivolts = cfg_u16(
            config,
            "vbat_min_cell_centivolts",
            BATTERY_MIN_CELL_CENTIVOLTS_DEFAULT,
        );
        let vbat_max_cell_centivolts = cfg_u16(
            config,
            "vbat_max_cell_centivolts",
            BATTERY_MAX_CELL_CENTIVOLTS_DEFAULT,
        );
        let vbat_warn_cell_centivolts = cfg_u16(
            config,
            "vbat_warn_cell_centivolts",
            BATTERY_WARN_CELL_CENTIVOLTS_DEFAULT,
        );
        let battery_cells = config
            .and_then(|cfg| cfg.get::<u32>("battery_cells"))
            .and_then(|cells| u8::try_from(cells).ok())
            .filter(|cells| *cells > 0);
        Ok(Self {
            last_voltage_centi: None,
            battery_cells,
            vbat_scale,
            vbat_res_div_val,
            vbat_res_div_mult,
            battery_capacity,
            vbat_min_cell_centivolts,
            vbat_max_cell_centivolts,
            vbat_warn_cell_centivolts,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (req_msg, voltage_msg) = *input;
        output.tov = req_msg.tov;

        if let Some(voltage) = voltage_msg.payload() {
            self.last_voltage_centi = Some(voltage.centivolts);
        }

        let mut batch = MspRequestBatch::new();
        let Some(requests) = req_msg.payload() else {
            output.clear_payload();
            return Ok(());
        };

        let voltage_centi = self.last_voltage_centi.unwrap_or(0);
        let voltage_decivolts = clamp_u8((voltage_centi + 5) / 10);
        let cell_count = self
            .battery_cells
            .or_else(|| estimate_cell_count(voltage_centi));
        let min_cell_decivolts = clamp_u8((self.vbat_min_cell_centivolts + 5) / 10);
        let max_cell_decivolts = clamp_u8((self.vbat_max_cell_centivolts + 5) / 10);
        let warn_cell_decivolts = clamp_u8((self.vbat_warn_cell_centivolts + 5) / 10);

        let battery_state = MspBatteryState {
            battery_cell_count: cell_count.unwrap_or(0),
            battery_capacity: self.battery_capacity,
            battery_voltage: voltage_decivolts,
            mah_drawn: 0,
            amperage: 0,
            alerts: 0,
            battery_voltage_mv: voltage_centi,
        };
        let battery_config = MspBatteryConfig {
            vbat_min_cell_voltage: min_cell_decivolts,
            vbat_max_cell_voltage: max_cell_decivolts,
            vbat_warning_cell_voltage: warn_cell_decivolts,
            battery_capacity: self.battery_capacity,
            voltage_meter_source: BATTERY_VOLTAGE_METER_SOURCE_ADC,
            current_meter_source: BATTERY_CURRENT_METER_SOURCE_NONE,
            vbat_min_cell_voltage_mv: self.vbat_min_cell_centivolts,
            vbat_max_cell_voltage_mv: self.vbat_max_cell_centivolts,
            vbat_warning_cell_voltage_mv: self.vbat_warn_cell_centivolts,
        };
        let voltage_meter_config = MspVoltageMeterConfig {
            sensor_count: 1,
            subframe_len: MSP_VOLTAGE_METER_ADC_SUBFRAME_LEN,
            id: MSP_VOLTAGE_METER_ID_BATTERY_1,
            sensor_type: MSP_VOLTAGE_METER_SENSOR_TYPE_ADC_RES_DIV,
            vbat_scale: self.vbat_scale,
            vbat_res_div_val: self.vbat_res_div_val,
            vbat_res_div_mult: self.vbat_res_div_mult,
        };
        let api_version = MspApiVersion {
            protocol_version: MSP_API_PROTOCOL_VERSION,
            api_version_major: MSP_API_VERSION_MAJOR,
            api_version_minor: MSP_API_VERSION_MINOR,
        };
        let fc_version = MspFlightControllerVersion {
            major: MSP_FC_VERSION_MAJOR,
            minor: MSP_FC_VERSION_MINOR,
            patch: MSP_FC_VERSION_PATCH,
        };
        let analog = MspAnalog {
            battery_voltage: voltage_decivolts,
            mah_drawn: 0,
            rssi: 0,
            amperage: 0,
            battery_voltage_mv: voltage_centi,
        };
        let voltage_meter = MspVoltageMeter {
            id: MSP_VOLTAGE_METER_ID_BATTERY_1,
            value: voltage_decivolts,
        };

        for request in requests.iter() {
            match request {
                MspRequest::MspApiVersionRequest => {
                    batch.push(MspRequest::MspApiVersion(api_version));
                }
                MspRequest::MspFcVersionRequest => {
                    batch.push(MspRequest::MspFlightControllerVersion(fc_version));
                }
                MspRequest::MspBatteryConfigRequest => {
                    batch.push(MspRequest::MspBatteryConfig(battery_config));
                }
                MspRequest::MspBatteryStateRequest => {
                    batch.push(MspRequest::MspBatteryState(battery_state));
                }
                MspRequest::MspAnalogRequest => {
                    batch.push(MspRequest::MspAnalog(analog));
                }
                MspRequest::MspVoltageMeterConfigRequest => {
                    batch.push(MspRequest::MspVoltageMeterConfig(voltage_meter_config));
                }
                MspRequest::MspVoltageMetersRequest => {
                    batch.push(MspRequest::MspVoltageMeter(voltage_meter));
                }
                _ => {}
            }
        }

        if !batch.0.is_empty() {
            debug!(
                "MSP responder: sending {} responses, vbat={} cv",
                batch.0.len(),
                voltage_centi
            );
        }

        if batch.0.is_empty() {
            output.clear_payload();
        } else {
            output.set_payload(batch);
        }
        Ok(())
    }
}

fn clamp_u8(value: u16) -> u8 {
    value.min(u16::from(u8::MAX)) as u8
}

fn estimate_cell_count(voltage_centi: u16) -> Option<u8> {
    if voltage_centi == 0 {
        return None;
    }
    let cells = u32::from(voltage_centi).div_ceil(420);
    Some(clamp_u8(cells.min(u32::from(u8::MAX)) as u16))
}

pub struct ImuLogger {
    last_tov: Option<CuTime>,
}

impl Freezable for ImuLogger {}

impl CuSinkTask for ImuLogger {
    type Input<'m> = CuMsg<ImuPayload>;
    type Resources<'r> = ();

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { last_tov: None })
    }

    fn process<'i>(&mut self, _clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()> {
        if let Some(payload) = input.payload() {
            let tov_time = expect_tov_time(input.tov)?;
            debug_rl!(&LOG_IMU, tov_time, {
                let tov_kind = 1_u8;
                let tov_start_ns = tov_time.as_nanos();
                let tov_end_ns = tov_time.as_nanos();
                let dt_us = match self.last_tov {
                    Some(prev) => (tov_time - prev).as_micros(),
                    None => 0,
                };
                self.last_tov = Some(tov_time);
                let gx_dps = payload.gyro_x.get::<degree_per_second>();
                let gy_dps = payload.gyro_y.get::<degree_per_second>();
                let gz_dps = payload.gyro_z.get::<degree_per_second>();
                let temp_c = payload.temperature.get::<degree_celsius>();
                debug!(
                    "imu ax={} m.s⁻² ay={} m.s⁻² az={} m.s⁻² gx={} deg.s⁻¹ gy={} deg.s⁻¹ gz={} deg.s⁻¹ t={} °C tov_kind={} tov_start_ns={} tov_end_ns={} tov_dt_us={}",
                    payload.accel_x.value,
                    payload.accel_y.value,
                    payload.accel_z.value,
                    gx_dps,
                    gy_dps,
                    gz_dps,
                    temp_c,
                    tov_kind,
                    tov_start_ns,
                    tov_end_ns,
                    dt_us
                );
            });
        }
        Ok(())
    }
}

pub type Bmi088Source = bmi088::Bmi088Source<
    cu_micoairh743::Bmi088Spi,
    cu_micoairh743::Bmi088AccCs,
    cu_micoairh743::Bmi088GyrCs,
    cu_micoairh743::Bmi088Delay,
>;

impl Freezable for FlightMode {}
impl Freezable for ControlInputs {}
impl Freezable for BodyRateSetpoint {}
impl Freezable for BodyCommand {}
impl Freezable for BatteryVoltage {}
impl Freezable for LedBeat {}

impl CuSinkTask for LedBeat {
    type Resources<'r> = Resources;
    type Input<'m> = CuMsg<ControlInputs>;

    fn new_with(_config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            on: false,
            led: resources.led.0,
        })
    }

    fn process<'i>(&mut self, _clock: &RobotClock, _inputs: &Self::Input<'i>) -> CuResult<()> {
        // Toggle the green LED so we can see if the Copper loop is alive.
        let mut led = self.led.lock();
        if self.on {
            led.set_low();
        } else {
            led.set_high();
        }
        self.on = !self.on;
        Ok(())
    }
}

pub struct RcMapper {
    rc_min: u16,
    rc_mid: u16,
    rc_max: u16,
    deadband: u16,
    arm_channel: usize,
    arm_min: u16,
    arm_max: u16,
    mode_channel: Option<usize>,
    mode_low_max: Option<u16>,
    mode_mid_max: Option<u16>,
}

impl Freezable for RcMapper {}

impl CuTask for RcMapper {
    type Resources<'r> = ();
    type Input<'m> = CuMsg<RcChannelsPayload>;
    type Output<'m> = CuMsg<ControlInputs>;

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let arm_cfg = config.and_then(|cfg| cfg.get::<u32>("arm_channel"));
        let mut arm_channel = arm_cfg.map(|v| v as usize).unwrap_or(3);
        if arm_channel > 15 {
            warning!(
                "rc mapper arm_channel {} out of range, clamping to 15",
                arm_channel
            );
            arm_channel = 15;
        }
        let arm_min = cfg_u16(config, "arm_min", 1700);
        let arm_max = cfg_u16(config, "arm_max", 1811);
        let mode_cfg = config
            .and_then(|cfg| cfg.get::<u32>("mode_channel"))
            .map(|v| v as usize);
        let mode_low_max = config
            .and_then(|cfg| cfg.get::<u32>("mode_low_max"))
            .map(|v| v.min(u16::MAX as u32) as u16);
        let mode_mid_max = config
            .and_then(|cfg| cfg.get::<u32>("mode_mid_max"))
            .map(|v| v.min(u16::MAX as u32) as u16);

        info!(
            "rc mapper cfg arm_channel={:?} arm_min={} arm_max={} mode_channel={:?} mode_low_max={} mode_mid_max={}",
            arm_cfg,
            arm_min,
            arm_max,
            mode_cfg,
            mode_low_max.unwrap_or(0),
            mode_mid_max.unwrap_or(0)
        );

        Ok(Self {
            rc_min: cfg_u16(config, "rc_min", 172),
            rc_mid: cfg_u16(config, "rc_mid", 992),
            rc_max: cfg_u16(config, "rc_max", 1811),
            deadband: cfg_u16(config, "rc_deadband", 0),
            arm_channel,
            arm_min,
            arm_max,
            mode_channel: mode_cfg,
            mode_low_max,
            mode_mid_max,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        inputs: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let tov_time = expect_tov_time(inputs.tov)?;
        let Some(rc) = inputs.payload() else {
            output.tov = Tov::Time(tov_time);
            output.clear_payload();
            return Ok(());
        };

        let channels = &rc.inner().0;
        let roll = normalize_axis(channels.first().copied().unwrap_or(self.rc_mid), self);
        let pitch = -normalize_axis(channels.get(1).copied().unwrap_or(self.rc_mid), self);
        let throttle = normalize_throttle(
            channels.get(2).copied().unwrap_or(self.rc_min),
            self.rc_min,
            self.rc_max,
        );
        let yaw = normalize_axis(channels.get(3).copied().unwrap_or(self.rc_mid), self);

        let arm_value = channels.get(self.arm_channel).copied().unwrap_or(0);
        let armed = arm_value >= self.arm_min && arm_value <= self.arm_max;

        let mode = match self.mode_channel.and_then(|idx| channels.get(idx).copied()) {
            Some(value) => {
                let (low_max, mid_max) = match (self.mode_low_max, self.mode_mid_max) {
                    (Some(low), Some(mid)) => (low, mid),
                    _ => {
                        let low = ((self.rc_min as u32 + self.rc_mid as u32) / 2) as u16;
                        let mid = ((self.rc_mid as u32 + self.rc_max as u32) / 2) as u16;
                        (low, mid)
                    }
                };
                if value <= low_max {
                    FlightMode::Acro
                } else if value <= mid_max {
                    FlightMode::Angle
                } else {
                    FlightMode::PositionHold
                }
            }
            None => FlightMode::Angle,
        };

        debug_rl!(&LOG_RC, tov_time, {
            let mode_channel = self.mode_channel.unwrap_or(0);
            let mode_value = self
                .mode_channel
                .and_then(|idx| channels.get(idx).copied())
                .unwrap_or(0);
            debug!(
                "rc ch0={} ch1={} ch2={} ch3={} ch4={} ch5={} ch6={} ch7={} ch8={} ch9={} ch10={} ch11={} ch12={} ch13={} ch14={} ch15={} arm_ch0={} arm_raw={} armed={} mode_ch0={} mode_raw={} mode={} mode_low_max={} mode_mid_max={}",
                channels[0],
                channels[1],
                channels[2],
                channels[3],
                channels[4],
                channels[5],
                channels[6],
                channels[7],
                channels[8],
                channels[9],
                channels[10],
                channels[11],
                channels[12],
                channels[13],
                channels[14],
                channels[15],
                self.arm_channel,
                arm_value,
                armed,
                mode_channel,
                mode_value,
                mode_label(mode),
                self.mode_low_max.unwrap_or(0),
                self.mode_mid_max.unwrap_or(0)
            );
        });

        output.tov = Tov::Time(tov_time);
        output.set_payload(ControlInputs {
            roll,
            pitch,
            yaw,
            throttle,
            armed,
            mode,
        });
        Ok(())
    }
}

pub struct ImuCalibrator {
    bias: [f32; 3],
    sum: [f32; 3],
    samples: u32,
    required_samples: u32,
    last_armed: bool,
    calibrating: bool,
}

impl Freezable for ImuCalibrator {}

impl CuTask for ImuCalibrator {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, ImuPayload, ControlInputs);
    type Output<'m> = CuMsg<ImuPayload>;

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cal_ms = cfg_u32(config, "cal_ms", 3000);
        let sample_period_ms = cfg_u32(config, "sample_period_ms", 10);
        let default_samples = (cal_ms / sample_period_ms.max(1)).max(1);
        let required_samples = cfg_u32(config, "cal_samples", default_samples);

        Ok(Self {
            bias: [0.0; 3],
            sum: [0.0; 3],
            samples: 0,
            required_samples,
            last_armed: false,
            calibrating: false,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (imu_msg, ctrl_msg) = *input;
        let imu_tov = expect_tov_time(imu_msg.tov)?;
        let _ = expect_tov_time(ctrl_msg.tov)?;
        let Some(imu) = imu_msg.payload() else {
            output.tov = Tov::Time(imu_tov);
            output.clear_payload();
            return Ok(());
        };
        let armed = ctrl_msg.payload().map(|c| c.armed).unwrap_or(false);

        if !armed {
            self.calibrating = false;
            self.sum = [0.0; 3];
            self.samples = 0;
            self.last_armed = false;
            output.tov = Tov::Time(imu_tov);
            output.set_payload(*imu);
            return Ok(());
        }

        if armed && !self.last_armed {
            self.calibrating = true;
            self.sum = [0.0; 3];
            self.samples = 0;
        }
        self.last_armed = armed;

        if self.calibrating {
            self.sum[0] += imu.gyro_x.value;
            self.sum[1] += imu.gyro_y.value;
            self.sum[2] += imu.gyro_z.value;
            self.samples = self.samples.saturating_add(1);

            if self.samples >= self.required_samples {
                let inv = 1.0 / (self.samples as f32);
                self.bias = [self.sum[0] * inv, self.sum[1] * inv, self.sum[2] * inv];
                self.calibrating = false;
                let bias_deg = [
                    self.bias[0].to_degrees(),
                    self.bias[1].to_degrees(),
                    self.bias[2].to_degrees(),
                ];
                info!(
                    "imu gyro bias x={} deg.s⁻¹ y={} deg.s⁻¹ z={} deg.s⁻¹",
                    bias_deg[0], bias_deg[1], bias_deg[2]
                );
            }

            output.tov = Tov::Time(imu_tov);
            output.clear_payload();
            output.metadata.set_status("cal");
            return Ok(());
        }

        let accel_mps2 = [imu.accel_x.value, imu.accel_y.value, imu.accel_z.value];
        let gyro_rad = [
            imu.gyro_x.value - self.bias[0],
            imu.gyro_y.value - self.bias[1],
            imu.gyro_z.value - self.bias[2],
        ];
        let temp_c = imu.temperature.get::<degree_celsius>();

        output.tov = Tov::Time(imu_tov);
        output.metadata.set_status("ok");
        output.set_payload(ImuPayload::from_raw(accel_mps2, gyro_rad, temp_c));
        Ok(())
    }
}

struct AxisPid {
    pid: PIDController,
    initialized: bool,
}

impl AxisPid {
    fn new(pid: PIDController) -> Self {
        Self {
            pid,
            initialized: false,
        }
    }

    fn reset(&mut self) {
        self.pid.reset();
        self.initialized = false;
    }

    fn reset_integral(&mut self) {
        self.pid.reset_integral();
    }

    fn update(&mut self, measurement: f32, dt: CuDuration) -> Option<PIDControlOutputPayload> {
        // cu_pid expects dt in microseconds (it divides by 1e6), so scale nanoseconds down.
        let dt_pid = CuDuration::from_nanos((dt.as_nanos() / 1_000).max(1));
        if !self.initialized {
            self.pid.init_measurement(measurement);
            self.initialized = true;
            return None;
        }
        Some(self.pid.next_control_output(measurement, dt_pid))
    }
}

pub struct AttitudeController {
    roll_pid: AxisPid,
    pitch_pid: AxisPid,
    angle_limit_rad: f32,
    rate_limit_rad: f32,
    acro_rate_rad: f32,
    acro_expo: f32,
    dt_fallback: CuDuration,
    last_time: Option<CuTime>,
    last_mode: FlightMode,
}

impl Freezable for AttitudeController {}

impl CuTask for AttitudeController {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, AhrsPose, ControlInputs);
    type Output<'m> = CuMsg<BodyRateSetpoint>;

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let angle_limit_rad = cfg_f32(config, "angle_limit_deg", 25.0).to_radians();
        let rate_limit_rad = cfg_f32(config, "rate_limit_dps", 180.0).to_radians();
        let acro_rate_rad = cfg_f32(config, "acro_rate_dps", 360.0).to_radians();
        let acro_expo = normalize_expo(cfg_f32(config, "acro_expo", 0.0));
        let kp = cfg_f32(config, "kp", 4.0);
        let ki = cfg_f32(config, "ki", 0.0);
        let kd = cfg_f32(config, "kd", 0.0);
        let dt_fallback = CuDuration::from_millis(cfg_u32(config, "dt_ms", 10) as u64);

        let roll_pid = PIDController::new(
            kp,
            ki,
            kd,
            0.0,
            rate_limit_rad,
            rate_limit_rad,
            rate_limit_rad,
            rate_limit_rad,
            CuDuration::default(),
        );
        let pitch_pid = PIDController::new(
            kp,
            ki,
            kd,
            0.0,
            rate_limit_rad,
            rate_limit_rad,
            rate_limit_rad,
            rate_limit_rad,
            CuDuration::default(),
        );

        Ok(Self {
            roll_pid: AxisPid::new(roll_pid),
            pitch_pid: AxisPid::new(pitch_pid),
            angle_limit_rad,
            rate_limit_rad,
            acro_rate_rad,
            acro_expo,
            dt_fallback,
            last_time: None,
            last_mode: FlightMode::Angle,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (pose_msg, ctrl_msg) = *input;
        let pose_tov = expect_tov_time(pose_msg.tov)?;
        let _ = expect_tov_time(ctrl_msg.tov)?;
        let output_tov = Tov::Time(pose_tov);
        let Some(pose) = pose_msg.payload() else {
            output.tov = output_tov;
            output.clear_payload();
            return Ok(());
        };
        let Some(ctrl) = ctrl_msg.payload() else {
            output.tov = output_tov;
            output.clear_payload();
            return Ok(());
        };

        if !ctrl.armed {
            self.roll_pid.reset();
            self.pitch_pid.reset();
            self.last_mode = ctrl.mode;
            output.tov = output_tov;
            output.set_payload(BodyRateSetpoint::default());
            return Ok(());
        }

        if ctrl.mode != self.last_mode {
            self.roll_pid.reset();
            self.pitch_pid.reset();
            self.last_mode = ctrl.mode;
        }

        let now = clock.now();
        let dt = dt_or_fallback(&mut self.last_time, now, self.dt_fallback);

        let (roll_rate, pitch_rate) =
            if matches!(ctrl.mode, FlightMode::Angle | FlightMode::PositionHold) {
                let target_roll = (ctrl.roll * self.angle_limit_rad)
                    .clamp(-self.angle_limit_rad, self.angle_limit_rad);
                let target_pitch = (ctrl.pitch * self.angle_limit_rad)
                    .clamp(-self.angle_limit_rad, self.angle_limit_rad);
                let roll_measure = pose.roll - target_roll;
                let pitch_measure = pose.pitch - target_pitch;
                let roll_out = self.roll_pid.update(roll_measure, dt).unwrap_or_default();
                let pitch_out = self.pitch_pid.update(pitch_measure, dt).unwrap_or_default();
                let roll_rate = roll_out.output;
                let pitch_rate = pitch_out.output;
                (
                    roll_rate.clamp(-self.rate_limit_rad, self.rate_limit_rad),
                    pitch_rate.clamp(-self.rate_limit_rad, self.rate_limit_rad),
                )
            } else {
                self.roll_pid.reset();
                self.pitch_pid.reset();
                let roll_cmd = apply_expo(ctrl.roll, self.acro_expo);
                let pitch_cmd = apply_expo(ctrl.pitch, self.acro_expo);
                (
                    (roll_cmd * self.acro_rate_rad).clamp(-self.acro_rate_rad, self.acro_rate_rad),
                    (pitch_cmd * self.acro_rate_rad).clamp(-self.acro_rate_rad, self.acro_rate_rad),
                )
            };

        let yaw_cmd = apply_expo(ctrl.yaw, self.acro_expo);
        let yaw_rate =
            (yaw_cmd * self.acro_rate_rad).clamp(-self.acro_rate_rad, self.acro_rate_rad);

        output.tov = output_tov;
        output.set_payload(BodyRateSetpoint {
            roll: roll_rate,
            pitch: pitch_rate,
            yaw: yaw_rate,
        });
        Ok(())
    }
}

pub struct RateController {
    roll_pid: AxisPid,
    pitch_pid: AxisPid,
    yaw_pid: AxisPid,
    output_limit: f32,
    dt_fallback: CuDuration,
    i_throttle_min: f32,
    airmode_enabled: bool,
    airmode_start_throttle: f32,
    airmode_active: bool,
    last_time: Option<CuTime>,
}

impl Freezable for RateController {}

impl CuTask for RateController {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, BodyRateSetpoint, ImuPayload, ControlInputs);
    type Output<'m> = CuMsg<BodyCommand>;

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let kp = cfg_f32(config, "kp", 0.15);
        let ki = cfg_f32(config, "ki", 0.0);
        let kd = cfg_f32(config, "kd", 0.0);
        let kp_yaw = cfg_f32(config, "kp_yaw", kp);
        let ki_yaw = cfg_f32(config, "ki_yaw", ki);
        let kd_yaw = cfg_f32(config, "kd_yaw", kd);
        let output_limit = cfg_f32(config, "output_limit", 1.0);
        let dt_fallback = CuDuration::from_millis(cfg_u32(config, "dt_ms", 10) as u64);
        let i_throttle_min = cfg_f32(config, "i_throttle_min", 0.05);
        let airmode_enabled = cfg_bool(config, "airmode", false);
        let airmode_start_throttle =
            normalize_percent(cfg_f32(config, "airmode_start_throttle_percent", 25.0));

        let pid = |p: f32, i: f32, d: f32| {
            PIDController::new(
                p,
                i,
                d,
                0.0,
                output_limit,
                output_limit,
                output_limit,
                output_limit,
                CuDuration::default(),
            )
        };

        Ok(Self {
            roll_pid: AxisPid::new(pid(kp, ki, kd)),
            pitch_pid: AxisPid::new(pid(kp, ki, kd)),
            yaw_pid: AxisPid::new(pid(kp_yaw, ki_yaw, kd_yaw)),
            output_limit,
            dt_fallback,
            i_throttle_min,
            airmode_enabled,
            airmode_start_throttle,
            airmode_active: false,
            last_time: None,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (setpoint_msg, imu_msg, ctrl_msg) = *input;
        let _ = expect_tov_time(setpoint_msg.tov)?;
        let imu_tov = expect_tov_time(imu_msg.tov)?;
        let ctrl_tov = expect_tov_time(ctrl_msg.tov)?;
        let output_tov = Tov::Time(imu_tov);
        let Some(setpoint) = setpoint_msg.payload() else {
            output.tov = output_tov;
            output.clear_payload();
            return Ok(());
        };
        let Some(imu) = imu_msg.payload() else {
            output.tov = output_tov;
            output.clear_payload();
            return Ok(());
        };
        let ctrl = ctrl_msg.payload();
        let armed = ctrl.map(|c| c.armed).unwrap_or(false);
        let throttle = ctrl.map(|c| c.throttle).unwrap_or(0.0);

        if !armed {
            self.roll_pid.reset();
            self.pitch_pid.reset();
            self.yaw_pid.reset();
            self.airmode_active = false;
            output.tov = output_tov;
            output.set_payload(BodyCommand::default());
            return Ok(());
        }

        // Latch airmode active once throttle is raised (betaflight behavior)
        if self.airmode_enabled && throttle >= self.airmode_start_throttle {
            self.airmode_active = true;
        } else if !self.airmode_enabled {
            self.airmode_active = false;
        }
        // Note: airmode_active stays true once activated until disarm

        let now = clock.now();
        let dt = dt_or_fallback(&mut self.last_time, now, self.dt_fallback);

        if self.airmode_enabled && !self.airmode_active {
            self.roll_pid.reset();
            self.pitch_pid.reset();
            self.yaw_pid.reset();
            output.tov = output_tov;
            output.set_payload(BodyCommand::default());
            return Ok(());
        }

        let roll_measure = imu.gyro_x.value - setpoint.roll;
        let pitch_measure = imu.gyro_y.value - setpoint.pitch;
        let yaw_measure = imu.gyro_z.value - setpoint.yaw;

        let roll_out = self.roll_pid.update(roll_measure, dt).unwrap_or_default();
        let pitch_out = self.pitch_pid.update(pitch_measure, dt).unwrap_or_default();
        let yaw_out = self.yaw_pid.update(yaw_measure, dt).unwrap_or_default();
        let roll_cmd = roll_out.output;
        let pitch_cmd = pitch_out.output;
        let yaw_cmd = yaw_out.output;

        debug_rl!(&LOG_RATE, ctrl_tov, {
            let sp_roll = setpoint.roll.to_degrees();
            let sp_pitch = setpoint.pitch.to_degrees();
            let sp_yaw = setpoint.yaw.to_degrees();
            let gyro_roll = imu.gyro_x.value.to_degrees();
            let gyro_pitch = imu.gyro_y.value.to_degrees();
            let gyro_yaw = imu.gyro_z.value.to_degrees();
            info!(
                "rate_pid dt_us={} sp_r={} gyro_r={} out_r={} p_r={} i_r={} d_r={}",
                dt.as_micros(),
                sp_roll,
                gyro_roll,
                roll_out.output,
                roll_out.p,
                roll_out.i,
                roll_out.d
            );
            info!(
                "rate_pid sp_p={} gyro_p={} out_p={} p_p={} i_p={} d_p={}",
                sp_pitch, gyro_pitch, pitch_out.output, pitch_out.p, pitch_out.i, pitch_out.d
            );
            info!(
                "rate_pid sp_y={} gyro_y={} out_y={} p_y={} i_y={} d_y={}",
                sp_yaw, gyro_yaw, yaw_out.output, yaw_out.p, yaw_out.i, yaw_out.d
            );
        });

        if throttle < self.i_throttle_min && !self.airmode_active {
            self.roll_pid.reset_integral();
            self.pitch_pid.reset_integral();
            self.yaw_pid.reset_integral();
        }

        output.tov = output_tov;
        output.set_payload(BodyCommand {
            roll: roll_cmd.clamp(-self.output_limit, self.output_limit),
            pitch: pitch_cmd.clamp(-self.output_limit, self.output_limit),
            yaw: yaw_cmd.clamp(-self.output_limit, self.output_limit),
        });
        Ok(())
    }
}

const QUADX_MIX: [(f32, f32, f32); 4] = [
    (-1.0, -1.0, -1.0), // rear right
    (-1.0, 1.0, 1.0),   // front right
    (1.0, -1.0, 1.0),   // rear left
    (1.0, 1.0, -1.0),   // front left
];
const DSHOT_MIN_ARM_CMD: u16 = 100;

struct MotorLogState {
    values: [u16; 4],
}

impl MotorLogState {
    const fn new() -> Self {
        Self { values: [0; 4] }
    }
}

static MOTOR_LOG: spin::Mutex<MotorLogState> = spin::Mutex::new(MotorLogState::new());

pub struct QuadXMixer {
    motor_index: usize,
    props_out: bool,
    airmode_idle: f32, // Motor idle value when in airmode at low throttle (0.0-1.0)
}

impl Freezable for QuadXMixer {}

impl CuTask for QuadXMixer {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, ControlInputs, BodyCommand);
    type Output<'m> = CuMsg<EscCommand>;

    fn new_with(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let motor_index = cfg_usize(config, "motor_index", 0);
        if motor_index >= QUADX_MIX.len() {
            return Err(CuError::from("motor_index out of range for QuadX"));
        }
        let props_out = config
            .and_then(|cfg| cfg.get::<bool>("props_out"))
            .unwrap_or(true);
        let airmode_idle = cfg_f32(config, "airmode_idle_percent", 8.0) / 100.0;

        Ok(Self {
            motor_index,
            props_out,
            airmode_idle: airmode_idle.clamp(0.0, 0.3), // Max 30% idle
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (ctrl_msg, cmd_msg) = *input;
        let ctrl_tov = expect_tov_time(ctrl_msg.tov)?;
        let cmd_tov = expect_tov_time(cmd_msg.tov)?;
        let output_tov = Tov::Time(cmd_tov);
        let Some(ctrl) = ctrl_msg.payload() else {
            output.tov = output_tov;
            output.set_payload(EscCommand::disarm());
            return Ok(());
        };

        let command = match cmd_msg.payload() {
            Some(cmd) if ctrl.armed => {
                let (roll_coeff, pitch_coeff, mut yaw_coeff) = QUADX_MIX[self.motor_index];
                if self.props_out {
                    yaw_coeff = -yaw_coeff;
                }
                let mix = cmd.roll * roll_coeff + cmd.pitch * pitch_coeff + cmd.yaw * yaw_coeff;
                let throttle = ctrl.throttle.clamp(0.0, 1.0);

                // Apply airmode idle at low throttle (for control authority during power loops)
                // When throttle < 50%, scale from idle to normal. Above 50%, normal mixing.
                let motor = if throttle < 0.5 {
                    // Low throttle: blend from idle (at 0%) to normal (at 50%)
                    let blend = throttle * 2.0; // 0.0 at 0%, 1.0 at 50%
                    let base = self.airmode_idle * (1.0 - blend) + throttle * blend;
                    (base + mix).clamp(0.0, 1.0)
                } else {
                    // High throttle: normal mixing
                    (throttle + mix).clamp(0.0, 1.0)
                };

                let throttle_raw = (motor * 2047.0) as u16;
                EscCommand {
                    throttle: throttle_raw.max(DSHOT_MIN_ARM_CMD),
                    request_telemetry: true,
                }
            }
            _ => EscCommand::disarm(),
        };

        {
            let mut state = MOTOR_LOG.lock();
            if self.motor_index < state.values.len() {
                state.values[self.motor_index] = command.throttle;
            }
        }

        if self.motor_index == 0 {
            debug_rl!(&LOG_MOTORS, ctrl_tov, {
                let state = MOTOR_LOG.lock();
                debug!(
                    "motors cmd0={} cmd1={} cmd2={} cmd3={}",
                    state.values[0], state.values[1], state.values[2], state.values[3]
                );
            });
        }

        output.tov = output_tov;
        output.set_payload(command);
        Ok(())
    }
}

fn normalize_axis(raw: u16, cfg: &RcMapper) -> f32 {
    let raw_i = raw as i32;
    let mid_i = cfg.rc_mid as i32;
    let deadband = cfg.deadband as i32;
    if (raw_i - mid_i).abs() <= deadband {
        return 0.0;
    }
    if raw >= cfg.rc_mid {
        let denom = (cfg.rc_max.saturating_sub(cfg.rc_mid)) as f32;
        if denom <= 0.0 {
            return 0.0;
        }
        ((raw - cfg.rc_mid) as f32 / denom).clamp(0.0, 1.0)
    } else {
        let denom = (cfg.rc_mid.saturating_sub(cfg.rc_min)) as f32;
        if denom <= 0.0 {
            return 0.0;
        }
        -((cfg.rc_mid - raw) as f32 / denom).clamp(0.0, 1.0)
    }
}

fn normalize_throttle(raw: u16, min: u16, max: u16) -> f32 {
    if max <= min {
        return 0.0;
    }
    let span = (max - min) as f32;
    ((raw.saturating_sub(min)) as f32 / span).clamp(0.0, 1.0)
}

fn expect_tov_time(tov: Tov) -> CuResult<CuTime> {
    match tov {
        Tov::Time(time) => Ok(time),
        Tov::Range(range) => {
            info!(
                "tov mismatch: expected time, got range start={} end={}",
                range.start.as_nanos(),
                range.end.as_nanos()
            );
            Err(CuError::from("Expected TOV::Time"))
        }
        Tov::None => {
            info!("tov mismatch: expected time, got none");
            Err(CuError::from("Expected TOV::Time"))
        }
    }
}

fn mode_label(mode: FlightMode) -> &'static str {
    match mode {
        FlightMode::Angle => "angle",
        FlightMode::Acro => "air",
        FlightMode::PositionHold => "position",
    }
}

fn dt_or_fallback(last_time: &mut Option<CuTime>, now: CuTime, fallback: CuDuration) -> CuDuration {
    let dt = last_time.map(|prev| now - prev);
    *last_time = Some(now);
    match dt {
        Some(duration) if duration > CuDuration::MIN => duration,
        _ => fallback,
    }
}

fn normalize_percent(raw: f32) -> f32 {
    let value = if raw > 1.0 { raw / 100.0 } else { raw };
    value.clamp(0.0, 1.0)
}

fn normalize_expo(raw: f32) -> f32 {
    normalize_percent(raw)
}

fn apply_expo(value: f32, expo: f32) -> f32 {
    let abs = value.abs();
    let weight = expo * abs * abs * abs + (1.0 - expo);
    value * weight
}

fn cfg_f32(config: Option<&ComponentConfig>, key: &str, default: f32) -> f32 {
    config
        .and_then(|cfg| cfg.get::<f64>(key))
        .map(|v| v as f32)
        .unwrap_or(default)
}

fn cfg_u32(config: Option<&ComponentConfig>, key: &str, default: u32) -> u32 {
    config
        .and_then(|cfg| cfg.get::<u32>(key))
        .unwrap_or(default)
}

fn cfg_u16(config: Option<&ComponentConfig>, key: &str, default: u16) -> u16 {
    config
        .and_then(|cfg| cfg.get::<u32>(key))
        .map(|v| v.min(u16::MAX as u32) as u16)
        .unwrap_or(default)
}

fn cfg_bool(config: Option<&ComponentConfig>, key: &str, default: bool) -> bool {
    config
        .and_then(|cfg| cfg.get::<bool>(key))
        .unwrap_or(default)
}

fn cfg_usize(config: Option<&ComponentConfig>, key: &str, default: usize) -> usize {
    config
        .and_then(|cfg| cfg.get::<u32>(key))
        .map(|v| v as usize)
        .unwrap_or(default)
}
