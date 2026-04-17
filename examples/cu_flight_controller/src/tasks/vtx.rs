use crate::messages::{BatteryVoltage, ControlInputs, FlightMode, GeographicHeading};
use crate::tasks;
use crate::tasks::{
    BATTERY_CURRENT_METER_SOURCE_NONE, BATTERY_MAX_CELL_CENTIVOLTS_DEFAULT,
    BATTERY_MIN_CELL_CENTIVOLTS_DEFAULT, BATTERY_VBAT_RES_DIV_MULT_DEFAULT,
    BATTERY_VBAT_RES_DIV_VAL_DEFAULT, BATTERY_VBAT_SCALE_DEFAULT, BATTERY_VOLTAGE_METER_SOURCE_ADC,
    BATTERY_WARN_CELL_CENTIVOLTS_DEFAULT, MSP_API_PROTOCOL_VERSION, MSP_API_VERSION_MAJOR,
    MSP_API_VERSION_MINOR, MSP_ARMING_DISABLE_FLAGS_COUNT, MSP_FC_VERSION_MAJOR,
    MSP_FC_VERSION_MINOR, MSP_FC_VERSION_PATCH, MSP_VOLTAGE_METER_ADC_SUBFRAME_LEN,
    MSP_VOLTAGE_METER_ID_BATTERY_1, MSP_VOLTAGE_METER_SENSOR_TYPE_ADC_RES_DIV, StatusLabel,
    VTX_CELL_DIVISOR, VTX_DRAW_PERIOD_MS, VTX_HEARTBEAT_PERIOD_MS, VTX_WATERMARK_LINES,
};
use alloc::vec::Vec;
use cu_gnss_payloads::GnssFixSolution;
use cu_msp_bridge::MspRequestBatch;
use cu_msp_lib::structs::{
    MspAnalog, MspApiVersion, MspBatteryConfig, MspBatteryState, MspDisplayPort,
    MspFlightControllerVersion, MspRequest, MspStatus, MspStatusSensors, MspVoltageMeter,
    MspVoltageMeterConfig,
};
use cu_sensor_payloads::BarometerPayload;
use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::electric_potential::volt;
use cu29::units::si::velocity::meter_per_second;

const VTX_SYM_VOLT: char = '\x06';
const VTX_SYM_DEGREE: char = '\x08';
const VTX_SYM_ALTITUDE: char = '\x7f';
const VTX_SYM_METER: char = '\x0c';
const VTX_SYM_SPEED: u8 = 0x70;
const VTX_SYM_KPH: u8 = 0x9E;
const VTX_SYM_LATITUDE: u8 = 0x89;
const VTX_SYM_LONGITUDE: u8 = 0x98;
const VTX_ALT_UNKNOWN: &str = "---.-";
const VTX_SPEED_UNKNOWN: &str = "---.-";
const VTX_ALT_FIELD_WIDTH: u8 = 8;

macro_rules! status_if_not_firmware {
    ($metadata:expr, $status:expr) => {{
        #[cfg(not(feature = "firmware"))]
        {
            $metadata.set_status($status);
        }
    }};
}

fn voltage_to_centivolts(voltage_v: f32) -> u16 {
    let scaled = voltage_v * 100.0;
    if !scaled.is_finite() || scaled <= 0.0 {
        return 0;
    }
    let centivolts = (scaled + 0.5) as u32;
    centivolts.min(u16::MAX as u32) as u16
}

fn push_request_or_drop(batch: &mut MspRequestBatch, request: MspRequest, dropped: &mut usize) {
    if batch.push(request).is_err() {
        *dropped = dropped.saturating_add(1);
    }
}

#[derive(Reflect)]
pub struct VtxOsd {
    row: u8,
    cols: u8,
    rows: u8,
    col_center: u8,
    heading_row: u8,
    heading_col_center: u8,
    gps_row: u8,
    gps_col_center: u8,
    alt_row: u8,
    alt_col: u8,
    speed_row: u8,
    speed_col: u8,
    watermark_row: u8,
    watermark_col: u8,
    last_label: Option<StatusLabel>,
    last_heartbeat: Option<CuTime>,
    last_draw: Option<CuTime>,
    last_armed: bool,
    last_mode: FlightMode,
    last_voltage_centi: Option<u16>,
    takeoff_pressure_pa: Option<f32>,
    pressure_sum_pa: f32,
    pressure_samples: u32,
    last_pressure_pa: Option<f32>,
    last_heading_deg: Option<f32>,
    last_lat_deg: Option<f64>,
    last_lon_deg: Option<f64>,
    last_ground_speed_mps: Option<f32>,
}

impl Freezable for VtxOsd {}

impl CuTask for VtxOsd {
    type Input<'m> = input_msg!(
        'm,
        ControlInputs,
        BarometerPayload,
        GeographicHeading,
        BatteryVoltage,
        MspRequestBatch,
        GnssFixSolution
    );
    type Output<'m> = CuMsg<MspRequestBatch>;
    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cols = tasks::cfg_u16(config, "cols", 53)?
            .max(1)
            .min(u8::MAX as u16) as u8;
        let rows = tasks::cfg_u16(config, "rows", 16)?
            .max(1)
            .min(u8::MAX as u16) as u8;
        let default_center = (cols / 2) as u16;
        let col_center = tasks::cfg_u16(config, "col_center", default_center)?
            .min(cols.saturating_sub(1) as u16) as u8;
        let row = tasks::cfg_u16(config, "row", 13)?.min(u8::MAX as u16) as u8;
        let heading_row =
            tasks::cfg_u16(config, "heading_row", 1)?.min(rows.saturating_sub(1) as u16) as u8;
        let heading_col_center = tasks::cfg_u16(config, "heading_col_center", default_center)?
            .min(cols.saturating_sub(1) as u16) as u8;
        let default_gps_row: u8 = 0;
        let gps_row = tasks::cfg_u16(config, "gps_row", u16::from(default_gps_row))?
            .min(rows.saturating_sub(1) as u16) as u8;
        let gps_col_center =
            tasks::cfg_u16(config, "gps_col_center", u16::from(heading_col_center))?
                .min(cols.saturating_sub(1) as u16) as u8;
        let default_alt_col = cols.saturating_sub(19) as u16;
        let alt_row =
            tasks::cfg_u16(config, "alt_row", 7)?.min(rows.saturating_sub(1) as u16) as u8;
        let alt_col = tasks::cfg_u16(config, "alt_col", default_alt_col)?
            .min(cols.saturating_sub(1) as u16) as u8;
        let default_speed_col =
            cols.saturating_sub(alt_col.saturating_add(VTX_ALT_FIELD_WIDTH)) as u16;
        let speed_row = tasks::cfg_u16(config, "speed_row", u16::from(alt_row))?
            .min(rows.saturating_sub(1) as u16) as u8;
        let speed_col = tasks::cfg_u16(config, "speed_col", default_speed_col)?
            .min(cols.saturating_sub(1) as u16) as u8;
        let watermark_height = VTX_WATERMARK_LINES.len() as u8;
        let default_watermark_row = rows.saturating_sub(watermark_height);
        let watermark_row = tasks::cfg_u16(config, "watermark_row", default_watermark_row as u16)?
            .min(rows.saturating_sub(1) as u16) as u8;
        let watermark_col =
            tasks::cfg_u16(config, "watermark_col", 0)?.min(cols.saturating_sub(1) as u16) as u8;
        Ok(Self {
            row,
            cols,
            rows,
            col_center,
            heading_row,
            heading_col_center,
            gps_row,
            gps_col_center,
            alt_row,
            alt_col,
            speed_row,
            speed_col,
            watermark_row,
            watermark_col,
            last_label: None,
            last_heartbeat: None,
            last_draw: None,
            last_armed: false,
            last_mode: FlightMode::Angle,
            last_voltage_centi: None,
            takeoff_pressure_pa: None,
            pressure_sum_pa: 0.0,
            pressure_samples: 0,
            last_pressure_pa: None,
            last_heading_deg: None,
            last_lat_deg: None,
            last_lon_deg: None,
            last_ground_speed_mps: None,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (ctrl_msg, baro_msg, heading_msg, batt_msg, incoming_msg, gnss_msg) = *input;
        let tov_time = tasks::expect_tov_time(ctrl_msg.tov)?;
        output.tov = Tov::Time(tov_time);
        let now = tov_time;
        let mut batch = MspRequestBatch::new();
        let mut dropped_requests = 0usize;
        let ctrl = ctrl_msg.payload();

        if let Some(heading) = heading_msg.payload() {
            self.last_heading_deg = Some(heading.heading.get::<degree>());
        }

        if let Some(voltage) = batt_msg.payload() {
            self.last_voltage_centi = Some(voltage_to_centivolts(voltage.voltage.get::<volt>()));
        }
        if let Some(fix) = gnss_msg.payload() {
            if fix.gnss_fix_ok && !fix.invalid_llh {
                let lat_deg = fix.position.latitude_degrees();
                let lon_deg = fix.position.longitude_degrees();
                let speed_mps = fix.ground_speed.get::<meter_per_second>();
                self.last_lat_deg = if lat_deg.is_finite() {
                    Some(lat_deg.clamp(-90.0, 90.0))
                } else {
                    None
                };
                self.last_lon_deg = if lon_deg.is_finite() {
                    Some(lon_deg.clamp(-180.0, 180.0))
                } else {
                    None
                };
                self.last_ground_speed_mps = if speed_mps.is_finite() && speed_mps >= 0.0 {
                    Some(speed_mps)
                } else {
                    None
                };
            } else {
                self.last_lat_deg = None;
                self.last_lon_deg = None;
                self.last_ground_speed_mps = None;
            }
        }

        // First, handle any incoming MSP requests (telemetry queries from VTX)
        if let Some(incoming_requests) = incoming_msg.payload() {
            self.handle_incoming_requests(&mut batch, incoming_requests, &mut dropped_requests);
        }

        let prev_armed = self.last_armed;
        if let Some(ctrl) = ctrl {
            self.last_armed = ctrl.armed;
            self.last_mode = ctrl.mode;
        }
        let armed = self.last_armed;
        let calibrating = false;

        if !armed {
            self.takeoff_pressure_pa = None;
            self.pressure_sum_pa = 0.0;
            self.pressure_samples = 0;
            self.last_pressure_pa = None;
        } else {
            if armed && !prev_armed {
                self.takeoff_pressure_pa = None;
                self.pressure_sum_pa = 0.0;
                self.pressure_samples = 0;
            }

            let pressure_pa = baro_msg
                .payload()
                .and_then(|baro| sanitize_pressure_pa(baro.pressure.value));
            if let Some(pressure_pa) = pressure_pa {
                self.last_pressure_pa = Some(pressure_pa);
                if self.takeoff_pressure_pa.is_none() && calibrating {
                    self.pressure_sum_pa += pressure_pa;
                    self.pressure_samples = self.pressure_samples.saturating_add(1);
                }
            }

            if self.takeoff_pressure_pa.is_none() && !calibrating {
                if self.pressure_samples > 0 {
                    self.takeoff_pressure_pa =
                        Some(self.pressure_sum_pa / (self.pressure_samples as f32));
                } else if let Some(pressure_pa) = self.last_pressure_pa {
                    self.takeoff_pressure_pa = Some(pressure_pa);
                }
            }
        }

        let heartbeat_due = self
            .last_heartbeat
            .map(|prev| now - prev >= CuDuration::from_millis(VTX_HEARTBEAT_PERIOD_MS))
            .unwrap_or(true);
        if heartbeat_due {
            push_request_or_drop(
                &mut batch,
                MspRequest::MspDisplayPort(MspDisplayPort::heartbeat()),
                &mut dropped_requests,
            );
            push_request_or_drop(
                &mut batch,
                MspRequest::MspStatus(build_msp_status(armed)),
                &mut dropped_requests,
            );
            self.last_heartbeat = Some(now);
        }

        let label = if calibrating {
            StatusLabel::Calibrating
        } else if armed {
            match self.last_mode {
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
            push_request_or_drop(
                &mut batch,
                MspRequest::MspDisplayPort(MspDisplayPort::clear_screen()),
                &mut dropped_requests,
            );
            push_request_or_drop(
                &mut batch,
                MspRequest::MspDisplayPort(MspDisplayPort::write_string(self.row, col, 0, text)),
                &mut dropped_requests,
            );
            self.push_heading(&mut batch, &mut dropped_requests);
            self.push_gps_position(&mut batch, &mut dropped_requests);
            self.push_cell_voltage(&mut batch, &mut dropped_requests);
            self.push_ground_speed(&mut batch, &mut dropped_requests);
            self.push_relative_altitude(&mut batch, &mut dropped_requests);
            self.push_watermark(&mut batch, &mut dropped_requests);
        }

        let draw_due = self
            .last_draw
            .map(|prev| now - prev >= CuDuration::from_millis(VTX_DRAW_PERIOD_MS))
            .unwrap_or(true);
        if label_changed || draw_due {
            if !label_changed {
                self.push_heading(&mut batch, &mut dropped_requests);
                self.push_gps_position(&mut batch, &mut dropped_requests);
                self.push_cell_voltage(&mut batch, &mut dropped_requests);
                self.push_ground_speed(&mut batch, &mut dropped_requests);
                self.push_relative_altitude(&mut batch, &mut dropped_requests);
                self.push_watermark(&mut batch, &mut dropped_requests);
            }
            push_request_or_drop(
                &mut batch,
                MspRequest::MspDisplayPort(MspDisplayPort::draw_screen()),
                &mut dropped_requests,
            );
            self.last_draw = Some(now);
        }

        if batch.0.is_empty() {
            status_if_not_firmware!(
                output.metadata,
                if dropped_requests == 0 {
                    format!("osd {}", label.as_str().trim())
                } else {
                    format!("osd {} d{}", label.as_str().trim(), dropped_requests)
                }
            );
            output.clear_payload();
        } else {
            status_if_not_firmware!(
                output.metadata,
                if dropped_requests == 0 {
                    format!("osd {} q{}", label.as_str().trim(), batch.0.len())
                } else {
                    format!(
                        "osd {} q{} d{}",
                        label.as_str().trim(),
                        batch.0.len(),
                        dropped_requests
                    )
                }
            );
            output.set_payload(batch);
        }
        Ok(())
    }
}

impl VtxOsd {
    fn push_heading(&self, batch: &mut MspRequestBatch, dropped: &mut usize) {
        if self.rows == 0 || self.cols == 0 {
            return;
        }
        let row = self.heading_row.min(self.rows.saturating_sub(1));
        let text = self.format_heading();
        let width = text.len() as u8;
        let col = if self.cols <= width {
            0
        } else {
            let half = width / 2;
            let mut col = self.heading_col_center.saturating_sub(half);
            if col.saturating_add(width) > self.cols {
                col = self.cols.saturating_sub(width);
            }
            col
        };
        push_request_or_drop(
            batch,
            MspRequest::MspDisplayPort(MspDisplayPort::write_string(row, col, 0, text.as_str())),
            dropped,
        );
    }

    fn push_gps_position(&self, batch: &mut MspRequestBatch, dropped: &mut usize) {
        if self.rows == 0 || self.cols == 0 {
            return;
        }
        let row = self.gps_row.min(self.rows.saturating_sub(1));
        let text = self.format_gps_position();
        let width = text.len() as u8;
        let col = if self.cols <= width {
            0
        } else {
            let half = width / 2;
            let mut col = self.gps_col_center.saturating_sub(half);
            if col.saturating_add(width) > self.cols {
                col = self.cols.saturating_sub(width);
            }
            col
        };
        let available = self.cols.saturating_sub(col) as usize;
        if available == 0 {
            return;
        }
        let mut display_bytes = text.as_slice();
        if display_bytes.len() > available {
            display_bytes = &display_bytes[..available];
        }
        if display_bytes.is_empty() {
            return;
        }
        push_request_or_drop(
            batch,
            MspRequest::MspDisplayPort(MspDisplayPort::write_bytes(row, col, 0, display_bytes)),
            dropped,
        );
    }

    fn handle_incoming_requests(
        &self,
        batch: &mut MspRequestBatch,
        requests: &MspRequestBatch,
        dropped: &mut usize,
    ) {
        let voltage_centi = self.last_voltage_centi.unwrap_or(0);
        let voltage_decivolts = tasks::clamp_u8((voltage_centi + 5) / 10);
        let cell_count = tasks::estimate_cell_count(voltage_centi);

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
                    push_request_or_drop(batch, MspRequest::MspApiVersion(api_version), dropped);
                }
                MspRequest::MspFcVersionRequest => {
                    push_request_or_drop(
                        batch,
                        MspRequest::MspFlightControllerVersion(fc_version),
                        dropped,
                    );
                }
                MspRequest::MspBatteryStateRequest => {
                    push_request_or_drop(
                        batch,
                        MspRequest::MspBatteryState(battery_state),
                        dropped,
                    );
                }
                MspRequest::MspAnalogRequest => {
                    push_request_or_drop(batch, MspRequest::MspAnalog(analog), dropped);
                }
                _ => {}
            }
        }
    }

    fn push_cell_voltage(&self, batch: &mut MspRequestBatch, dropped: &mut usize) {
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
        push_request_or_drop(
            batch,
            MspRequest::MspDisplayPort(MspDisplayPort::write_string(row, col, 0, text.as_str())),
            dropped,
        );
    }

    fn push_relative_altitude(&self, batch: &mut MspRequestBatch, dropped: &mut usize) {
        if self.rows == 0 || self.cols == 0 {
            return;
        }
        let row = self.alt_row.min(self.rows.saturating_sub(1));
        let col = self.alt_col.min(self.cols.saturating_sub(1));
        let available = self.cols.saturating_sub(col) as usize;
        if available == 0 {
            return;
        }

        let text = self.format_relative_altitude();
        let mut display_text = text.as_str();
        if display_text.len() > available {
            display_text = &display_text[..available];
        }
        if display_text.is_empty() {
            return;
        }

        push_request_or_drop(
            batch,
            MspRequest::MspDisplayPort(MspDisplayPort::write_string(row, col, 0, display_text)),
            dropped,
        );
    }

    fn push_ground_speed(&self, batch: &mut MspRequestBatch, dropped: &mut usize) {
        if self.rows == 0 || self.cols == 0 {
            return;
        }
        let row = self.speed_row.min(self.rows.saturating_sub(1));
        let col = self.speed_col.min(self.cols.saturating_sub(1));
        let available = self.cols.saturating_sub(col) as usize;
        if available == 0 {
            return;
        }

        let text = self.format_ground_speed();
        let mut display_bytes = text.as_slice();
        if display_bytes.len() > available {
            display_bytes = &display_bytes[..available];
        }
        if display_bytes.is_empty() {
            return;
        }

        push_request_or_drop(
            batch,
            MspRequest::MspDisplayPort(MspDisplayPort::write_bytes(row, col, 0, display_bytes)),
            dropped,
        );
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

    fn format_relative_altitude(&self) -> alloc::string::String {
        match (self.takeoff_pressure_pa, self.last_pressure_pa) {
            (Some(reference_pa), Some(pressure_pa))
                if let Some(altitude_m) = relative_altitude_m(reference_pa, pressure_pa) =>
            {
                let altitude_m = altitude_m.clamp(-999.9, 9999.9);
                let value = format_altitude_field_no_spaces(altitude_m);
                alloc::format!("{VTX_SYM_ALTITUDE}{value}{VTX_SYM_METER}")
            }
            _ => alloc::format!("{VTX_SYM_ALTITUDE}{VTX_ALT_UNKNOWN}{VTX_SYM_METER}"),
        }
    }

    fn format_ground_speed(&self) -> Vec<u8> {
        match self.last_ground_speed_mps {
            Some(speed_mps) if speed_mps.is_finite() && speed_mps >= 0.0 => {
                let speed_kmh = (speed_mps * 3.6).clamp(0.0, 999.9);
                let mut bytes = Vec::with_capacity(7);
                bytes.push(VTX_SYM_SPEED);
                bytes.extend_from_slice(alloc::format!("{speed_kmh:05.1}").as_bytes());
                bytes.push(VTX_SYM_KPH);
                bytes
            }
            _ => {
                let mut bytes = Vec::with_capacity(7);
                bytes.push(VTX_SYM_SPEED);
                bytes.extend_from_slice(VTX_SPEED_UNKNOWN.as_bytes());
                bytes.push(VTX_SYM_KPH);
                bytes
            }
        }
    }

    fn format_gps_position(&self) -> Vec<u8> {
        match (self.last_lat_deg, self.last_lon_deg) {
            (Some(lat), Some(lon)) if lat.is_finite() && lon.is_finite() => {
                let mut bytes = Vec::with_capacity(24);
                bytes.push(VTX_SYM_LATITUDE);
                bytes.extend_from_slice(alloc::format!("{lat:+010.6}").as_bytes());
                bytes.push(VTX_SYM_LONGITUDE);
                bytes.extend_from_slice(alloc::format!("{lon:+011.6}").as_bytes());
                bytes
            }
            _ => {
                let mut bytes = Vec::with_capacity(24);
                bytes.push(VTX_SYM_LATITUDE);
                bytes.extend_from_slice(b"+--.------");
                bytes.push(VTX_SYM_LONGITUDE);
                bytes.extend_from_slice(b"+---.------");
                bytes
            }
        }
    }

    fn format_heading(&self) -> alloc::string::String {
        match self.last_heading_deg {
            Some(heading_deg) if heading_deg.is_finite() => {
                let normalized = wrap_heading_deg(heading_deg);
                let rounded = libm::roundf(normalized) as i32;
                let wrapped = if rounded >= 360 { 0 } else { rounded.max(0) };
                alloc::format!("{wrapped:03}{VTX_SYM_DEGREE}")
            }
            _ => alloc::format!("---{VTX_SYM_DEGREE}"),
        }
    }

    fn push_watermark(&self, batch: &mut MspRequestBatch, dropped: &mut usize) {
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
            push_request_or_drop(
                batch,
                MspRequest::MspDisplayPort(MspDisplayPort::write_string(row, col, 0, text)),
                dropped,
            );
        }
    }
}

fn sanitize_pressure_pa(pressure_pa: f32) -> Option<f32> {
    if pressure_pa.is_finite() && pressure_pa > 0.0 {
        Some(pressure_pa)
    } else {
        None
    }
}

fn relative_altitude_m(reference_pa: f32, pressure_pa: f32) -> Option<f32> {
    let reference_pa = sanitize_pressure_pa(reference_pa)?;
    let pressure_pa = sanitize_pressure_pa(pressure_pa)?;
    let ratio = pressure_pa / reference_pa;
    if ratio <= 0.0 {
        return None;
    }
    let altitude_m: f32 = 44_330.0_f32 * (1.0_f32 - libm::powf(ratio, 1.0_f32 / 5.255_f32));
    if altitude_m.is_finite() {
        Some(altitude_m)
    } else {
        None
    }
}

fn format_altitude_field_no_spaces(altitude_m: f32) -> alloc::string::String {
    // Render a fixed 6-character field with no spaces so symbol and value stay visually tight.
    // Positive: 0000.0..9999.9, Negative: -000.0..-999.9
    let tenths = libm::roundf(altitude_m * 10.0_f32) as i32;
    if tenths < 0 {
        let abs_tenths = tenths.saturating_abs();
        let whole = (abs_tenths / 10).min(999);
        let frac = abs_tenths % 10;
        alloc::format!("-{whole:03}.{frac}")
    } else {
        let whole = (tenths / 10).min(9999);
        let frac = tenths % 10;
        alloc::format!("{whole:04}.{frac}")
    }
}

fn wrap_heading_deg(value: f32) -> f32 {
    let mut wrapped = libm::fmodf(value, 360.0);
    if wrapped < 0.0 {
        wrapped += 360.0;
    }
    wrapped
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

#[derive(Reflect)]
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

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let vbat_scale = tasks::cfg_u32(config, "vbat_scale", BATTERY_VBAT_SCALE_DEFAULT)?
            .min(u32::from(u8::MAX)) as u8;
        let vbat_res_div_val =
            tasks::cfg_u32(config, "vbat_res_div_val", BATTERY_VBAT_RES_DIV_VAL_DEFAULT)?
                .max(1)
                .min(u32::from(u8::MAX)) as u8;
        let vbat_res_div_mult = tasks::cfg_u32(
            config,
            "vbat_res_div_mult",
            BATTERY_VBAT_RES_DIV_MULT_DEFAULT,
        )?
        .max(1)
        .min(u32::from(u8::MAX)) as u8;
        let battery_capacity = tasks::cfg_u16(config, "battery_capacity_mah", 0)?;
        let vbat_min_cell_centivolts = tasks::cfg_u16(
            config,
            "vbat_min_cell_centivolts",
            BATTERY_MIN_CELL_CENTIVOLTS_DEFAULT,
        )?;
        let vbat_max_cell_centivolts = tasks::cfg_u16(
            config,
            "vbat_max_cell_centivolts",
            BATTERY_MAX_CELL_CENTIVOLTS_DEFAULT,
        )?;
        let vbat_warn_cell_centivolts = tasks::cfg_u16(
            config,
            "vbat_warn_cell_centivolts",
            BATTERY_WARN_CELL_CENTIVOLTS_DEFAULT,
        )?;
        let battery_cells = match config {
            Some(cfg) => cfg
                .get::<u32>("battery_cells")?
                .and_then(|cells| u8::try_from(cells).ok())
                .filter(|cells| *cells > 0),
            None => None,
        };
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
        _ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (req_msg, voltage_msg) = *input;
        output.tov = req_msg.tov;

        if let Some(voltage) = voltage_msg.payload() {
            self.last_voltage_centi = Some(voltage_to_centivolts(voltage.voltage.get::<volt>()));
        }

        let mut batch = MspRequestBatch::new();
        let mut dropped_requests = 0usize;
        let Some(requests) = req_msg.payload() else {
            status_if_not_firmware!(output.metadata, "msp wait");
            output.clear_payload();
            return Ok(());
        };

        let voltage_centi = self.last_voltage_centi.unwrap_or(0);
        let voltage_decivolts = tasks::clamp_u8((voltage_centi + 5) / 10);
        let cell_count = self
            .battery_cells
            .or_else(|| tasks::estimate_cell_count(voltage_centi));
        let min_cell_decivolts = tasks::clamp_u8((self.vbat_min_cell_centivolts + 5) / 10);
        let max_cell_decivolts = tasks::clamp_u8((self.vbat_max_cell_centivolts + 5) / 10);
        let warn_cell_decivolts = tasks::clamp_u8((self.vbat_warn_cell_centivolts + 5) / 10);

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
                    push_request_or_drop(
                        &mut batch,
                        MspRequest::MspApiVersion(api_version),
                        &mut dropped_requests,
                    );
                }
                MspRequest::MspFcVersionRequest => {
                    push_request_or_drop(
                        &mut batch,
                        MspRequest::MspFlightControllerVersion(fc_version),
                        &mut dropped_requests,
                    );
                }
                MspRequest::MspBatteryConfigRequest => {
                    push_request_or_drop(
                        &mut batch,
                        MspRequest::MspBatteryConfig(battery_config),
                        &mut dropped_requests,
                    );
                }
                MspRequest::MspBatteryStateRequest => {
                    push_request_or_drop(
                        &mut batch,
                        MspRequest::MspBatteryState(battery_state),
                        &mut dropped_requests,
                    );
                }
                MspRequest::MspAnalogRequest => {
                    push_request_or_drop(
                        &mut batch,
                        MspRequest::MspAnalog(analog),
                        &mut dropped_requests,
                    );
                }
                MspRequest::MspVoltageMeterConfigRequest => {
                    push_request_or_drop(
                        &mut batch,
                        MspRequest::MspVoltageMeterConfig(voltage_meter_config),
                        &mut dropped_requests,
                    );
                }
                MspRequest::MspVoltageMetersRequest => {
                    push_request_or_drop(
                        &mut batch,
                        MspRequest::MspVoltageMeter(voltage_meter),
                        &mut dropped_requests,
                    );
                }
                _ => {}
            }
        }

        if !batch.0.is_empty() {
            debug!(
                "MSP responder: sending {} responses, vbat={} cv, dropped={}",
                batch.0.len(),
                voltage_centi,
                dropped_requests
            );
        }

        if batch.0.is_empty() {
            status_if_not_firmware!(
                output.metadata,
                if dropped_requests == 0 {
                    format!("msp r{} q0", requests.0.len())
                } else {
                    format!("msp r{} q0 d{}", requests.0.len(), dropped_requests)
                }
            );
            output.clear_payload();
        } else {
            status_if_not_firmware!(
                output.metadata,
                if dropped_requests == 0 {
                    format!("msp r{} q{}", requests.0.len(), batch.0.len())
                } else {
                    format!(
                        "msp r{} q{} d{}",
                        requests.0.len(),
                        batch.0.len(),
                        dropped_requests
                    )
                }
            );
            output.set_payload(batch);
        }
        Ok(())
    }
}
