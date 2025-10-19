use portable_atomic::{AtomicU64, Ordering};

// Frequency estimation for converting raw counter-values to nanoseconds
static FREQUENCY_NS: AtomicU64 = AtomicU64::new(0);
static INIT_COUNTER: AtomicU64 = AtomicU64::new(0);
static INIT_TIME_NS: AtomicU64 = AtomicU64::new(0);

const CALIBRATION_PERIOD_NS: u64 = 10_000_000;

/// Calibrates the high-precision clock vs. a real time clock
pub fn calibrate(
    read_raw_counter: fn() -> u64,
    read_rtc_ns: impl Fn() -> u64 + Send + Sync + 'static,
    sleep_ns: impl Fn(u64) + Send + Sync + 'static,
) {
    let start_counter = read_raw_counter();
    let start_time = read_rtc_ns();

    sleep_ns(CALIBRATION_PERIOD_NS);

    let end_counter = read_raw_counter();
    let end_time = read_rtc_ns();

    let counter_diff = end_counter.saturating_sub(start_counter);
    let time_diff_ns = end_time.saturating_sub(start_time);

    if counter_diff > 0 {
        let freq_ns = ((counter_diff as u128 * 1_000_000_000) / time_diff_ns as u128) as u64;
        FREQUENCY_NS.store(freq_ns, Ordering::Relaxed);
        INIT_COUNTER.store(start_counter, Ordering::Relaxed);
        INIT_TIME_NS.store(start_time, Ordering::Relaxed);
    }
}

/// Translate the high-precision counter-value to real time nanoseconds
pub fn counter_to_nanos(read_raw_counter: fn() -> u64) -> u64 {
    let freq = FREQUENCY_NS.load(Ordering::Relaxed);
    let init_counter = INIT_COUNTER.load(Ordering::Relaxed);
    let init_time_ns = INIT_TIME_NS.load(Ordering::Relaxed);
    let counter = read_raw_counter();
    let counter_diff = counter.saturating_sub(init_counter);

    init_time_ns.saturating_add(((counter_diff as u128 * 1_000_000_000) / freq as u128) as u64)
}
