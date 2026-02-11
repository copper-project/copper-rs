use core::sync::atomic::{AtomicU64, Ordering};

use cu_gnss_payloads::GnssEvent;
use cu29::prelude::*;

pub mod state {
    use super::*;

    static TOTAL_EVENTS: AtomicU64 = AtomicU64::new(0);
    static NAV_EVENTS: AtomicU64 = AtomicU64::new(0);
    static SAT_EVENTS: AtomicU64 = AtomicU64::new(0);
    static SIG_EVENTS: AtomicU64 = AtomicU64::new(0);
    static RF_EVENTS: AtomicU64 = AtomicU64::new(0);
    static INFO_EVENTS: AtomicU64 = AtomicU64::new(0);
    static ACK_EVENTS: AtomicU64 = AtomicU64::new(0);
    static RAW_EVENTS: AtomicU64 = AtomicU64::new(0);

    pub fn record(event: &GnssEvent) {
        TOTAL_EVENTS.fetch_add(1, Ordering::Relaxed);
        match event {
            GnssEvent::NavEpoch(_) => {
                NAV_EVENTS.fetch_add(1, Ordering::Relaxed);
            }
            GnssEvent::SatelliteState(_) => {
                SAT_EVENTS.fetch_add(1, Ordering::Relaxed);
            }
            GnssEvent::SignalState(_) => {
                SIG_EVENTS.fetch_add(1, Ordering::Relaxed);
            }
            GnssEvent::RfStatus(_) => {
                RF_EVENTS.fetch_add(1, Ordering::Relaxed);
            }
            GnssEvent::InfoText(_) => {
                INFO_EVENTS.fetch_add(1, Ordering::Relaxed);
            }
            GnssEvent::CommandAck(_) => {
                ACK_EVENTS.fetch_add(1, Ordering::Relaxed);
            }
            GnssEvent::RawUbx(_) => {
                RAW_EVENTS.fetch_add(1, Ordering::Relaxed);
            }
            GnssEvent::None => {}
        }
    }

    pub fn summary() -> String {
        format!(
            "total={} nav={} sat={} sig={} rf={} info={} ack={} raw={}",
            TOTAL_EVENTS.load(Ordering::Relaxed),
            NAV_EVENTS.load(Ordering::Relaxed),
            SAT_EVENTS.load(Ordering::Relaxed),
            SIG_EVENTS.load(Ordering::Relaxed),
            RF_EVENTS.load(Ordering::Relaxed),
            INFO_EVENTS.load(Ordering::Relaxed),
            ACK_EVENTS.load(Ordering::Relaxed),
            RAW_EVENTS.load(Ordering::Relaxed)
        )
    }

    pub fn total_events() -> u64 {
        TOTAL_EVENTS.load(Ordering::Relaxed)
    }
}

#[derive(Reflect)]
pub struct EventPrinterSink;

impl Freezable for EventPrinterSink {}

impl CuSinkTask for EventPrinterSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(GnssEvent);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        let Some(event) = input.payload() else {
            return Ok(());
        };

        state::record(event);

        let total = state::total_events();
        if total <= 5 || total % 25 == 0 {
            println!("[gnss] {}", state::summary());
        }

        Ok(())
    }
}
