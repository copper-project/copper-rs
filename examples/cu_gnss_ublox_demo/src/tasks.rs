use core::sync::atomic::{AtomicU64, Ordering};

use cu_gnss_payloads::{
    GnssAccuracy, GnssCommandAck, GnssEpochTime, GnssFixSolution, GnssInfoText, GnssRawUbxFrame,
    GnssRfStatus, GnssSatelliteState, GnssSatsInView, GnssSignalState,
};
use cu29::prelude::*;

pub mod state {
    use super::*;

    static TIME_EVENTS: AtomicU64 = AtomicU64::new(0);
    static FIX_EVENTS: AtomicU64 = AtomicU64::new(0);
    static SAT_IN_VIEW_EVENTS: AtomicU64 = AtomicU64::new(0);

    pub fn mark_time() {
        TIME_EVENTS.fetch_add(1, Ordering::Relaxed);
    }

    pub fn mark_fix() {
        FIX_EVENTS.fetch_add(1, Ordering::Relaxed);
    }

    pub fn mark_sats_in_view() {
        SAT_IN_VIEW_EVENTS.fetch_add(1, Ordering::Relaxed);
    }

    pub fn total_events() -> u64 {
        TIME_EVENTS.load(Ordering::Relaxed)
            + FIX_EVENTS.load(Ordering::Relaxed)
            + SAT_IN_VIEW_EVENTS.load(Ordering::Relaxed)
    }

    pub fn summary() -> String {
        format!(
            "time={} fix={} sats_in_view={}",
            TIME_EVENTS.load(Ordering::Relaxed),
            FIX_EVENTS.load(Ordering::Relaxed),
            SAT_IN_VIEW_EVENTS.load(Ordering::Relaxed)
        )
    }
}

#[derive(Reflect)]
pub struct TimeSink;

impl Freezable for TimeSink {}

impl CuSinkTask for TimeSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(GnssEpochTime);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if input.payload().is_some() {
            state::mark_time();
            let total = state::total_events();
            if total <= 5 || total % 25 == 0 {
                println!("[gnss] {}", state::summary());
            }
        }
        Ok(())
    }
}

#[derive(Reflect)]
pub struct FixSink;

impl Freezable for FixSink {}

impl CuSinkTask for FixSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(GnssFixSolution);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if input.payload().is_some() {
            state::mark_fix();
        }
        Ok(())
    }
}

#[derive(Reflect)]
pub struct SatsInViewSink;

impl Freezable for SatsInViewSink {}

impl CuSinkTask for SatsInViewSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(GnssSatsInView);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if input.payload().is_some() {
            state::mark_sats_in_view();
        }
        Ok(())
    }
}

#[derive(Reflect)]
pub struct DropUnusedASink;

impl Freezable for DropUnusedASink {}

impl CuSinkTask for DropUnusedASink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(
        'm,
        GnssAccuracy,
        GnssSatelliteState,
        GnssSignalState,
        GnssRfStatus,
        GnssInfoText
    );

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
