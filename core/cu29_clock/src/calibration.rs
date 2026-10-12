//! Measure a clock's hardware-counter frequency against its supplied RTC.

const CALIBRATION_PERIOD_NS: u64 = 10_000_000;

#[derive(Clone, Copy)]
pub(crate) struct Calibration {
    pub counter: u64,
    pub frequency_hz: u64,
}

pub(crate) fn measure(
    read_raw_counter: fn() -> u64,
    read_rtc_ns: impl Fn() -> u64 + Send + Sync + 'static,
    sleep_ns: impl Fn(u64) + Send + Sync + 'static,
) -> Option<Calibration> {
    let start_counter = read_raw_counter();
    let start_time = read_rtc_ns();

    sleep_ns(CALIBRATION_PERIOD_NS);

    let end_counter = read_raw_counter();
    let end_time = read_rtc_ns();

    let counter_diff = end_counter.saturating_sub(start_counter);
    let time_diff_ns = end_time.saturating_sub(start_time);

    if counter_diff > 0 {
        assert!(
            time_diff_ns > 0,
            "cu29_clock calibration failed: RTC delta is zero; check RTC hardware/clock source"
        );
        let freq_ns_u128 =
            (u128::from(counter_diff) * 1_000_000_000u128) / u128::from(time_diff_ns);
        let freq_ns = u64::try_from(freq_ns_u128).unwrap_or(u64::MAX);
        return Some(Calibration {
            counter: start_counter,
            frequency_hz: freq_ns,
        });
    }
    None
}
