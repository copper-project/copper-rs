extern crate alloc;

mod protocol;

use alloc::collections::VecDeque;
use alloc::vec::Vec;
use core::fmt;
use cu_gnss_payloads::{
    GnssAccuracy, GnssCommandAck, GnssEpochTime, GnssEvent, GnssFixSolution, GnssInfoText,
    GnssRawUbxFrame, GnssRfStatus, GnssSatelliteState, GnssSatsInView, GnssSignalState,
};
use cu_linux_resources::LinuxSerialPort;
use cu29::prelude::*;
use cu29::resource::{Owned, ResourceBindingMap, ResourceBindings, ResourceManager};
use std::io::{ErrorKind, Read, Write};

use crate::protocol::{UbxFrame, decode_frame_to_event, extract_next_ubx_frame};

const CLASS_NAV: u8 = 0x01;
const ID_NAV_PVT: u8 = 0x07;
const ID_NAV_SAT: u8 = 0x35;
const ID_NAV_SIG: u8 = 0x43;
const CLASS_MON: u8 = 0x0A;
const ID_MON_RF: u8 = 0x38;

const DEFAULT_READ_BUFFER_BYTES: usize = 4096;
const DEFAULT_MAX_PENDING_EVENTS: usize = 256;

const DEFAULT_POLL_NAV_PVT_MS: u64 = 200;
const DEFAULT_POLL_NAV_SAT_MS: u64 = 1000;
const DEFAULT_POLL_NAV_SIG_MS: u64 = 1000;
const DEFAULT_POLL_MON_RF_MS: u64 = 2000;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Binding {
    Serial,
}

pub struct UbloxResources {
    pub serial: Owned<LinuxSerialPort>,
}

impl<'r> ResourceBindings<'r> for UbloxResources {
    type Binding = Binding;

    fn from_bindings(
        manager: &'r mut ResourceManager,
        mapping: Option<&ResourceBindingMap<Self::Binding>>,
    ) -> CuResult<Self> {
        let mapping = mapping.ok_or_else(|| {
            CuError::from("UbxSource requires a `serial` resource mapping in copperconfig")
        })?;
        let path = mapping.get(Binding::Serial).ok_or_else(|| {
            CuError::from("UbxSource resources must include `serial: <bundle.resource>`")
        })?;

        let serial = manager
            .take::<LinuxSerialPort>(path.typed())
            .map_err(|e| e.add_cause("Failed to fetch UBX serial resource"))?;

        Ok(Self { serial })
    }
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct UbxSource {
    #[reflect(ignore)]
    serial: LinuxSerialPort,
    #[reflect(ignore)]
    read_buffer: Vec<u8>,
    #[reflect(ignore)]
    frame_buffer: Vec<u8>,
    #[reflect(ignore)]
    pending_events: VecDeque<GnssEvent>,
    max_pending_events: usize,
    emit_raw_unknown: bool,
    poll_nav_pvt_every_ms: u64,
    poll_nav_sat_every_ms: u64,
    poll_nav_sig_every_ms: u64,
    poll_mon_rf_every_ms: u64,
    #[reflect(ignore)]
    last_poll_nav_pvt_ns: Option<u64>,
    #[reflect(ignore)]
    last_poll_nav_sat_ns: Option<u64>,
    #[reflect(ignore)]
    last_poll_nav_sig_ns: Option<u64>,
    #[reflect(ignore)]
    last_poll_mon_rf_ns: Option<u64>,
}

impl fmt::Debug for UbxSource {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UbxSource")
            .field("emit_raw_unknown", &self.emit_raw_unknown)
            .field("poll_nav_pvt_every_ms", &self.poll_nav_pvt_every_ms)
            .field("poll_nav_sat_every_ms", &self.poll_nav_sat_every_ms)
            .field("poll_nav_sig_every_ms", &self.poll_nav_sig_every_ms)
            .field("poll_mon_rf_every_ms", &self.poll_mon_rf_every_ms)
            .field("pending_events", &self.pending_events.len())
            .finish()
    }
}

impl Freezable for UbxSource {}

impl CuSrcTask for UbxSource {
    type Resources<'r> = UbloxResources;
    type Output<'m> = output_msg!(
        GnssEpochTime,
        GnssFixSolution,
        GnssAccuracy,
        GnssSatsInView,
        GnssSatelliteState,
        GnssSignalState,
        GnssRfStatus,
        GnssInfoText,
        GnssCommandAck,
        GnssRawUbxFrame
    );

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let read_buffer_bytes = config_u32(
            config,
            "read_buffer_bytes",
            DEFAULT_READ_BUFFER_BYTES as u32,
        )? as usize;
        let max_pending_events = config_u32(
            config,
            "max_pending_events",
            DEFAULT_MAX_PENDING_EVENTS as u32,
        )? as usize;
        let max_pending_events = max_pending_events.max(1);

        let emit_raw_unknown = config_bool(config, "emit_raw_unknown", true)?;

        let poll_nav_pvt_every_ms = config_u64(config, "poll_nav_pvt_ms", DEFAULT_POLL_NAV_PVT_MS)?;
        let poll_nav_sat_every_ms = config_u64(config, "poll_nav_sat_ms", DEFAULT_POLL_NAV_SAT_MS)?;
        let poll_nav_sig_every_ms = config_u64(config, "poll_nav_sig_ms", DEFAULT_POLL_NAV_SIG_MS)?;
        let poll_mon_rf_every_ms = config_u64(config, "poll_mon_rf_ms", DEFAULT_POLL_MON_RF_MS)?;

        Ok(Self {
            serial: resources.serial.0,
            read_buffer: vec![0_u8; read_buffer_bytes.max(64)],
            frame_buffer: Vec::with_capacity(4096),
            pending_events: VecDeque::with_capacity(max_pending_events),
            max_pending_events,
            emit_raw_unknown,
            poll_nav_pvt_every_ms,
            poll_nav_sat_every_ms,
            poll_nav_sig_every_ms,
            poll_mon_rf_every_ms,
            last_poll_nav_pvt_ns: None,
            last_poll_nav_sat_ns: None,
            last_poll_nav_sig_ns: None,
            last_poll_mon_rf_ns: None,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.send_poll(CLASS_NAV, ID_NAV_PVT)?;
        self.send_poll(CLASS_NAV, ID_NAV_SAT)?;
        self.send_poll(CLASS_NAV, ID_NAV_SIG)?;
        self.send_poll(CLASS_MON, ID_MON_RF)?;
        Ok(())
    }

    fn process(&mut self, clock: &RobotClock, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        clear_outputs(new_msg);

        let now_ns = clock.now().as_nanos();

        if Self::is_poll_due(
            now_ns,
            self.poll_nav_pvt_every_ms,
            self.last_poll_nav_pvt_ns,
        ) {
            self.send_poll(CLASS_NAV, ID_NAV_PVT)?;
            self.last_poll_nav_pvt_ns = Some(now_ns);
        }
        if Self::is_poll_due(
            now_ns,
            self.poll_nav_sat_every_ms,
            self.last_poll_nav_sat_ns,
        ) {
            self.send_poll(CLASS_NAV, ID_NAV_SAT)?;
            self.last_poll_nav_sat_ns = Some(now_ns);
        }
        if Self::is_poll_due(
            now_ns,
            self.poll_nav_sig_every_ms,
            self.last_poll_nav_sig_ns,
        ) {
            self.send_poll(CLASS_NAV, ID_NAV_SIG)?;
            self.last_poll_nav_sig_ns = Some(now_ns);
        }
        if Self::is_poll_due(now_ns, self.poll_mon_rf_every_ms, self.last_poll_mon_rf_ns) {
            self.send_poll(CLASS_MON, ID_MON_RF)?;
            self.last_poll_mon_rf_ns = Some(now_ns);
        }

        if let Some(event) = self.pending_events.pop_front() {
            self.emit_event(clock, new_msg, event);
            return Ok(());
        }

        self.read_and_decode()?;

        if let Some(event) = self.pending_events.pop_front() {
            self.emit_event(clock, new_msg, event);
        }

        Ok(())
    }
}

impl UbxSource {
    fn send_poll(&mut self, class_id: u8, msg_id: u8) -> CuResult<()> {
        let frame = UbxFrame::from_message(class_id, msg_id, &[]);
        let bytes = frame.to_wire_bytes();
        self.serial
            .write_all(&bytes)
            .map_err(|e| CuError::new_with_cause("UBX poll write failed", e))
    }

    fn is_poll_due(now_ns: u64, period_ms: u64, last_ns: Option<u64>) -> bool {
        if period_ms == 0 {
            return false;
        }

        let period_ns = period_ms.saturating_mul(1_000_000);
        match last_ns {
            Some(last) => now_ns.saturating_sub(last) >= period_ns,
            None => true,
        }
    }

    fn read_and_decode(&mut self) -> CuResult<()> {
        loop {
            match self.serial.read(&mut self.read_buffer) {
                Ok(0) => break,
                Ok(n) => {
                    self.frame_buffer.extend_from_slice(&self.read_buffer[..n]);
                    self.decode_from_buffer();

                    if n < self.read_buffer.len() {
                        break;
                    }
                }
                Err(e) if e.kind() == ErrorKind::TimedOut || e.kind() == ErrorKind::WouldBlock => {
                    break;
                }
                Err(e) => return Err(CuError::new_with_cause("UBX serial read failed", e)),
            }
        }

        Ok(())
    }

    fn decode_from_buffer(&mut self) {
        while let Some(frame) = extract_next_ubx_frame(&mut self.frame_buffer) {
            if let Some(event) = decode_frame_to_event(&frame, self.emit_raw_unknown) {
                self.push_pending_event(event);
            }
        }
    }

    fn push_pending_event(&mut self, event: GnssEvent) {
        if self.pending_events.len() >= self.max_pending_events {
            self.pending_events.pop_front();
        }
        self.pending_events.push_back(event);
    }

    fn emit_event(
        &self,
        clock: &RobotClock,
        out: &mut <Self as CuSrcTask>::Output<'_>,
        event: GnssEvent,
    ) {
        let tov = Tov::Time(clock.now());
        match event {
            GnssEvent::None => {}
            GnssEvent::NavEpoch(nav) => {
                out.0.tov = tov;
                out.0.set_payload(nav.time);
                out.1.tov = tov;
                out.1.set_payload(nav.fix);
                out.2.tov = tov;
                out.2.set_payload(nav.accuracy);
            }
            GnssEvent::SatelliteState(sat) => {
                out.3.tov = tov;
                out.3.set_payload(GnssSatsInView {
                    itow_ms: sat.itow_ms,
                    count: sat.satellites.len() as u16,
                });
                out.4.tov = tov;
                out.4.set_payload(sat);
            }
            GnssEvent::SignalState(sig) => {
                out.5.tov = tov;
                out.5.set_payload(sig);
            }
            GnssEvent::RfStatus(rf) => {
                out.6.tov = tov;
                out.6.set_payload(rf);
            }
            GnssEvent::InfoText(info) => {
                out.7.tov = tov;
                out.7.set_payload(info);
            }
            GnssEvent::CommandAck(ack) => {
                out.8.tov = tov;
                out.8.set_payload(ack);
            }
            GnssEvent::RawUbx(raw) => {
                out.9.tov = tov;
                out.9.set_payload(raw);
            }
        }
    }
}

fn clear_outputs(out: &mut <UbxSource as CuSrcTask>::Output<'_>) {
    out.0.clear_payload();
    out.1.clear_payload();
    out.2.clear_payload();
    out.3.clear_payload();
    out.4.clear_payload();
    out.5.clear_payload();
    out.6.clear_payload();
    out.7.clear_payload();
    out.8.clear_payload();
    out.9.clear_payload();
}

fn config_u32(config: Option<&ComponentConfig>, key: &str, default: u32) -> CuResult<u32> {
    if let Some(cfg) = config {
        Ok(cfg.get::<u32>(key)?.unwrap_or(default))
    } else {
        Ok(default)
    }
}

fn config_u64(config: Option<&ComponentConfig>, key: &str, default: u64) -> CuResult<u64> {
    if let Some(cfg) = config {
        Ok(cfg.get::<u64>(key)?.unwrap_or(default))
    } else {
        Ok(default)
    }
}

fn config_bool(config: Option<&ComponentConfig>, key: &str, default: bool) -> CuResult<bool> {
    if let Some(cfg) = config {
        Ok(cfg.get::<bool>(key)?.unwrap_or(default))
    } else {
        Ok(default)
    }
}
