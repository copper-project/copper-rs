use core::sync::atomic::{AtomicU64, Ordering};

use cu_gnss_payloads::{
    GnssAccuracy, GnssCommandAck, GnssEpochTime, GnssFixSolution, GnssFixType, GnssInfoSeverity,
    GnssInfoText, GnssRawUbxFrame, GnssRfStatus, GnssSatelliteState, GnssSatsInView,
    GnssSignalState,
};
use cu29::prelude::*;
use uom::si::angle::degree;
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

pub mod state {
    use super::*;

    static TIME_EVENTS: AtomicU64 = AtomicU64::new(0);
    static FIX_EVENTS: AtomicU64 = AtomicU64::new(0);
    static ACCURACY_EVENTS: AtomicU64 = AtomicU64::new(0);
    static SAT_IN_VIEW_EVENTS: AtomicU64 = AtomicU64::new(0);
    static SAT_STATE_EVENTS: AtomicU64 = AtomicU64::new(0);
    static LAST_SATS_IN_VIEW: AtomicU64 = AtomicU64::new(0);
    static LAST_SATS_USED: AtomicU64 = AtomicU64::new(0);
    static LAST_HORIZONTAL_ACC_MM: AtomicU64 = AtomicU64::new(0);

    const MM_PER_M: f32 = 1000.0;

    pub fn mark_time() {
        TIME_EVENTS.fetch_add(1, Ordering::Relaxed);
    }

    pub fn mark_fix(sats_used: u8) {
        FIX_EVENTS.fetch_add(1, Ordering::Relaxed);
        LAST_SATS_USED.store(sats_used as u64, Ordering::Relaxed);
    }

    pub fn mark_accuracy(horizontal_m: f32) {
        ACCURACY_EVENTS.fetch_add(1, Ordering::Relaxed);

        let horizontal_mm = if horizontal_m.is_finite() && horizontal_m > 0.0 {
            (horizontal_m * MM_PER_M).round() as u64
        } else {
            0
        };
        LAST_HORIZONTAL_ACC_MM.store(horizontal_mm, Ordering::Relaxed);
    }

    pub fn mark_sats_in_view(count: u16) {
        SAT_IN_VIEW_EVENTS.fetch_add(1, Ordering::Relaxed);
        LAST_SATS_IN_VIEW.store(count as u64, Ordering::Relaxed);
    }

    pub fn mark_sat_state() {
        SAT_STATE_EVENTS.fetch_add(1, Ordering::Relaxed);
    }

    pub fn total_events() -> u64 {
        TIME_EVENTS.load(Ordering::Relaxed)
            + FIX_EVENTS.load(Ordering::Relaxed)
            + ACCURACY_EVENTS.load(Ordering::Relaxed)
            + SAT_IN_VIEW_EVENTS.load(Ordering::Relaxed)
            + SAT_STATE_EVENTS.load(Ordering::Relaxed)
    }

    pub fn summary() -> String {
        let h_acc_m = LAST_HORIZONTAL_ACC_MM.load(Ordering::Relaxed) as f32 / MM_PER_M;
        format!(
            "time={} fix={} accuracy={} sats_in_view_msgs={} sat_state={} last_sats_in_view={} last_sats_used={} last_h_acc={:.2}m",
            TIME_EVENTS.load(Ordering::Relaxed),
            FIX_EVENTS.load(Ordering::Relaxed),
            ACCURACY_EVENTS.load(Ordering::Relaxed),
            SAT_IN_VIEW_EVENTS.load(Ordering::Relaxed),
            SAT_STATE_EVENTS.load(Ordering::Relaxed),
            LAST_SATS_IN_VIEW.load(Ordering::Relaxed),
            LAST_SATS_USED.load(Ordering::Relaxed),
            h_acc_m,
        )
    }
}

#[derive(Default, Reflect)]
pub struct TimeSink {
    seen: u64,
}

impl Freezable for TimeSink {}

impl CuSinkTask for TimeSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(GnssEpochTime);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(time) = input.payload() {
            state::mark_time();
            self.seen += 1;

            if self.seen <= 3 || self.seen.is_multiple_of(10) {
                let utc = format!(
                    "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
                    time.year, time.month, time.day, time.hour, time.minute, time.second
                );
                debug!(
                    "[gnss/time] itow={} utc={} valid_date={} valid_time={} resolved={}",
                    time.itow_ms, utc, time.valid_date, time.valid_time, time.fully_resolved
                );
            }
        }
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct FixSink {
    seen: u64,
    last_fix_type: Option<GnssFixType>,
    last_fix_ok: Option<bool>,
    last_sats_used: Option<u8>,
}

impl Freezable for FixSink {}

impl CuSinkTask for FixSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(GnssFixSolution);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(fix) = input.payload() {
            state::mark_fix(fix.num_satellites_used);
            self.seen += 1;

            let fix_state_changed = self.last_fix_type != Some(fix.fix_type)
                || self.last_fix_ok != Some(fix.gnss_fix_ok);
            let sats_used_changed = self.last_sats_used != Some(fix.num_satellites_used);

            if fix_state_changed || self.seen <= 3 || self.seen.is_multiple_of(20) {
                let fix_type = format!("{:?}", fix.fix_type);
                let lat = format!("{:.7}", fix.latitude.get::<degree>());
                let lon = format!("{:.7}", fix.longitude.get::<degree>());
                let h_msl = format!("{:.2}", fix.height_msl.get::<meter>());
                let speed = format!("{:.2}", fix.ground_speed.get::<meter_per_second>());
                info!(
                    "[gnss/fix] type={} fix_ok={} sats_used={} lat={} lon={} h_msl={}m speed={}m/s invalid_llh={}",
                    fix_type,
                    fix.gnss_fix_ok,
                    fix.num_satellites_used,
                    lat,
                    lon,
                    h_msl,
                    speed,
                    fix.invalid_llh
                );
            } else if sats_used_changed {
                debug!("[gnss/fix] sats_used={}", fix.num_satellites_used);
            }

            self.last_fix_type = Some(fix.fix_type);
            self.last_fix_ok = Some(fix.gnss_fix_ok);
            self.last_sats_used = Some(fix.num_satellites_used);
        }
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct AccuracySink {
    seen: u64,
}

impl Freezable for AccuracySink {}

impl CuSinkTask for AccuracySink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(GnssAccuracy);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(accuracy) = input.payload() {
            state::mark_accuracy(accuracy.horizontal.get::<meter>());
            self.seen += 1;

            if self.seen <= 3 || self.seen.is_multiple_of(5) {
                let h_acc = format!("{:.2}", accuracy.horizontal.get::<meter>());
                let v_acc = format!("{:.2}", accuracy.vertical.get::<meter>());
                let speed_acc = format!("{:.2}", accuracy.speed.get::<meter_per_second>());
                let heading_acc = format!("{:.2}", accuracy.heading.get::<degree>());
                let pdop = format!("{:.2}", accuracy.position_dop);
                info!(
                    "[gnss/accuracy] h_acc={}m v_acc={}m speed_acc={}m/s heading_acc={}deg pdop={}",
                    h_acc, v_acc, speed_acc, heading_acc, pdop
                );
            }
        }
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct SatsInViewSink {
    seen: u64,
    last_count: Option<u16>,
}

impl Freezable for SatsInViewSink {}

impl CuSinkTask for SatsInViewSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(GnssSatsInView);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(sats) = input.payload() {
            state::mark_sats_in_view(sats.count);
            self.seen += 1;

            match self.last_count {
                None => {
                    info!("[gnss/sats] in_view={} (first sample)", sats.count);
                }
                Some(prev) if prev != sats.count => {
                    let delta = sats.count as i32 - prev as i32;
                    let delta = format!("{:+}", delta);
                    info!(
                        "[gnss/sats] in_view={} delta={} (count can move with visibility/tracking quality)",
                        sats.count, delta
                    );
                }
                Some(_) if self.seen.is_multiple_of(10) => {
                    debug!("[gnss/sats] in_view={} (stable)", sats.count);
                }
                _ => {}
            }

            self.last_count = Some(sats.count);
        }
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct SatelliteStateSink;

impl Freezable for SatelliteStateSink {}

impl CuSinkTask for SatelliteStateSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(GnssSatelliteState);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(sat_state) = input.payload() {
            state::mark_sat_state();

            let in_view = sat_state.satellites.len();
            let used_for_nav = sat_state
                .satellites
                .iter()
                .filter(|sat| sat.used_for_navigation)
                .count();

            let strongest_cno = sat_state
                .satellites
                .iter()
                .map(|sat| sat.cno_dbhz)
                .max()
                .unwrap_or(0);

            let (cno_sum, cno_count) = sat_state
                .satellites
                .iter()
                .filter(|sat| sat.cno_dbhz > 0)
                .fold((0_u32, 0_u32), |(sum, count), sat| {
                    (sum + sat.cno_dbhz as u32, count + 1)
                });

            let avg_cno = if cno_count > 0 {
                cno_sum as f32 / cno_count as f32
            } else {
                0.0
            };
            let avg_cno = format!("{:.1}", avg_cno);

            info!(
                "[gnss/sat_state] in_view={} used_for_nav={} strongest_cno={}dBHz avg_cno={}dBHz",
                in_view, used_for_nav, strongest_cno, avg_cno
            );
        }
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct InfoTextSink;

impl Freezable for InfoTextSink {}

impl CuSinkTask for InfoTextSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(GnssInfoText);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(msg) = input.payload() {
            match msg.severity {
                GnssInfoSeverity::Debug => debug!("[gnss/info] {}", msg.text.as_str()),
                GnssInfoSeverity::Notice | GnssInfoSeverity::Test => {
                    info!("[gnss/info] {}", msg.text.as_str())
                }
                GnssInfoSeverity::Warning => warning!("[gnss/info] {}", msg.text.as_str()),
                GnssInfoSeverity::Error => error!("[gnss/info] {}", msg.text.as_str()),
            }
        }
        Ok(())
    }
}

#[derive(Reflect)]
pub struct DropUnusedASink;

impl Freezable for DropUnusedASink {}

impl CuSinkTask for DropUnusedASink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, GnssSignalState, GnssRfStatus);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[derive(Reflect)]
pub struct DropUnusedBSink;

impl Freezable for DropUnusedBSink {}

impl CuSinkTask for DropUnusedBSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, GnssCommandAck, GnssRawUbxFrame);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}
