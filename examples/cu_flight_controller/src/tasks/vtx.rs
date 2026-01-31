use crate::messages::{BatteryVoltage, ControlInputs, FlightMode};
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
use cu_msp_bridge::MspRequestBatch;
use cu_msp_lib::structs::{
    MspAnalog, MspApiVersion, MspBatteryConfig, MspBatteryState, MspDisplayPort,
    MspFlightControllerVersion, MspRequest, MspStatus, MspStatusSensors, MspVoltageMeter,
    MspVoltageMeterConfig,
};
use cu29::prelude::*;

const VTX_SYM_VOLT: char = '\x06';
const MSP_REQUEST_BATCH_OVERFLOW: &str = "MSP request batch overflow";

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
        let tov_time = tasks::expect_tov_time(ctrl_msg.tov)?;
        output.tov = Tov::Time(tov_time);
        let now = tov_time;
        let mut batch = MspRequestBatch::new();
        let ctrl = ctrl_msg.payload();

        if let Some(voltage) = batt_msg.payload() {
            self.last_voltage_centi = Some(voltage.centivolts);
        }

        // First, handle any incoming MSP requests (telemetry queries from VTX)
        if let Some(incoming_requests) = incoming_msg.payload() {
            self.handle_incoming_requests(&mut batch, incoming_requests)?;
        }

        if let Some(ctrl) = ctrl {
            self.last_armed = ctrl.armed;
        }

        let heartbeat_due = self
            .last_heartbeat
            .map(|prev| now - prev >= CuDuration::from_millis(VTX_HEARTBEAT_PERIOD_MS))
            .unwrap_or(true);
        if heartbeat_due {
            batch.push(
                MspRequest::MspDisplayPort(MspDisplayPort::heartbeat()),
                MSP_REQUEST_BATCH_OVERFLOW,
            )?;
            batch.push(
                MspRequest::MspStatus(build_msp_status(self.last_armed)),
                MSP_REQUEST_BATCH_OVERFLOW,
            )?;
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
            batch.push(
                MspRequest::MspDisplayPort(MspDisplayPort::clear_screen()),
                MSP_REQUEST_BATCH_OVERFLOW,
            )?;
            batch.push(
                MspRequest::MspDisplayPort(MspDisplayPort::write_string(self.row, col, 0, text)),
                MSP_REQUEST_BATCH_OVERFLOW,
            )?;
            self.push_cell_voltage(&mut batch)?;
            self.push_watermark(&mut batch)?;
        }

        let draw_due = self
            .last_draw
            .map(|prev| now - prev >= CuDuration::from_millis(VTX_DRAW_PERIOD_MS))
            .unwrap_or(true);
        if label_changed || draw_due {
            if !label_changed {
                self.push_cell_voltage(&mut batch)?;
                self.push_watermark(&mut batch)?;
            }
            batch.push(
                MspRequest::MspDisplayPort(MspDisplayPort::draw_screen()),
                MSP_REQUEST_BATCH_OVERFLOW,
            )?;
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
    fn handle_incoming_requests(
        &self,
        batch: &mut MspRequestBatch,
        requests: &MspRequestBatch,
    ) -> CuResult<()> {
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
                    batch.push(
                        MspRequest::MspApiVersion(api_version),
                        MSP_REQUEST_BATCH_OVERFLOW,
                    )?;
                }
                MspRequest::MspFcVersionRequest => {
                    batch.push(
                        MspRequest::MspFlightControllerVersion(fc_version),
                        MSP_REQUEST_BATCH_OVERFLOW,
                    )?;
                }
                MspRequest::MspBatteryStateRequest => {
                    batch.push(
                        MspRequest::MspBatteryState(battery_state),
                        MSP_REQUEST_BATCH_OVERFLOW,
                    )?;
                }
                MspRequest::MspAnalogRequest => {
                    batch.push(MspRequest::MspAnalog(analog), MSP_REQUEST_BATCH_OVERFLOW)?;
                }
                _ => {}
            }
        }
        Ok(())
    }

    fn push_cell_voltage(&self, batch: &mut MspRequestBatch) -> CuResult<()> {
        let row = self.row.saturating_add(1);
        if row >= self.rows {
            return Ok(());
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
        batch.push(
            MspRequest::MspDisplayPort(MspDisplayPort::write_string(row, col, 0, text.as_str())),
            MSP_REQUEST_BATCH_OVERFLOW,
        )?;
        Ok(())
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

    fn push_watermark(&self, batch: &mut MspRequestBatch) -> CuResult<()> {
        if self.rows == 0 || self.cols == 0 {
            return Ok(());
        }
        let col = self.watermark_col.min(self.cols.saturating_sub(1));
        let available = self.cols.saturating_sub(col) as usize;
        if available == 0 {
            return Ok(());
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
            batch.push(
                MspRequest::MspDisplayPort(MspDisplayPort::write_string(row, col, 0, text)),
                MSP_REQUEST_BATCH_OVERFLOW,
            )?;
        }
        Ok(())
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
                    batch.push(
                        MspRequest::MspApiVersion(api_version),
                        MSP_REQUEST_BATCH_OVERFLOW,
                    )?;
                }
                MspRequest::MspFcVersionRequest => {
                    batch.push(
                        MspRequest::MspFlightControllerVersion(fc_version),
                        MSP_REQUEST_BATCH_OVERFLOW,
                    )?;
                }
                MspRequest::MspBatteryConfigRequest => {
                    batch.push(
                        MspRequest::MspBatteryConfig(battery_config),
                        MSP_REQUEST_BATCH_OVERFLOW,
                    )?;
                }
                MspRequest::MspBatteryStateRequest => {
                    batch.push(
                        MspRequest::MspBatteryState(battery_state),
                        MSP_REQUEST_BATCH_OVERFLOW,
                    )?;
                }
                MspRequest::MspAnalogRequest => {
                    batch.push(MspRequest::MspAnalog(analog), MSP_REQUEST_BATCH_OVERFLOW)?;
                }
                MspRequest::MspVoltageMeterConfigRequest => {
                    batch.push(
                        MspRequest::MspVoltageMeterConfig(voltage_meter_config),
                        MSP_REQUEST_BATCH_OVERFLOW,
                    )?;
                }
                MspRequest::MspVoltageMetersRequest => {
                    batch.push(
                        MspRequest::MspVoltageMeter(voltage_meter),
                        MSP_REQUEST_BATCH_OVERFLOW,
                    )?;
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
