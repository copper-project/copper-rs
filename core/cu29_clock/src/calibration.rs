use portable_atomic::{AtomicU64, Ordering};

// Frequency estimation for converting raw counter-values to nanoseconds
#[cfg(not(target_pointer_width = "32"))]
static FREQUENCY_NS: AtomicU64 = AtomicU64::new(0);
#[cfg(target_pointer_width = "32")]
static SCALE: CounterScale = CounterScale::new();
static INIT_COUNTER: AtomicU64 = AtomicU64::new(0);
static INIT_TIME_NS: AtomicU64 = AtomicU64::new(0);

const CALIBRATION_PERIOD_NS: u64 = 10_000_000;

// Keep the reduced ratio in one atomic so a reader sees one calibration and
// needs only one load for board clock rates. Both terms fit in 32 bits at
// frequencies up to u32::MAX Hz; wider denominators use the original formula.
#[cfg(any(target_pointer_width = "32", test))]
struct CounterScale {
    ratio: AtomicU64,
    wide_hz: AtomicU64,
}

#[cfg(any(target_pointer_width = "32", test))]
impl CounterScale {
    const fn new() -> Self {
        Self {
            ratio: AtomicU64::new(0),
            wide_hz: AtomicU64::new(0),
        }
    }

    fn calibrate(&self, hz: u64) {
        let (mut gcd, mut remainder) = (1_000_000_000, hz);
        while remainder != 0 {
            (gcd, remainder) = (remainder, gcd % remainder);
        }
        let numerator = 1_000_000_000 / gcd;
        let denominator = hz / gcd;
        if hz != 0 && denominator <= u64::from(u32::MAX) {
            self.ratio
                .store((numerator << 32) | denominator, Ordering::Release);
        } else {
            self.wide_hz.store(hz, Ordering::Relaxed);
            self.ratio.store(0, Ordering::Release);
        }
    }

    fn nanos(&self, ticks: u64) -> u64 {
        let ratio = self.ratio.load(Ordering::Acquire);
        if ratio == 0 {
            let hz = self.wide_hz.load(Ordering::Relaxed);
            return ((u128::from(ticks) * 1_000_000_000) / u128::from(hz)) as u64;
        }

        let num = ratio >> 32;
        let den = u64::from(ratio as u32);
        // Preserve 1.2's narrowing cast when the nanoseconds exceed u64::MAX.
        if den.is_power_of_two() {
            let shift = den.trailing_zeros();
            let fraction = ((ticks & (den - 1)) * num) >> shift;
            return (ticks >> shift).wrapping_mul(num).wrapping_add(fraction);
        }
        if let Some(product) = ticks.checked_mul(num) {
            return product / den;
        }
        let fraction = ((ticks % den) * num) / den;
        (ticks / den).wrapping_mul(num).wrapping_add(fraction)
    }
}

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
        assert!(
            time_diff_ns > 0,
            "cu29_clock calibration failed: RTC delta is zero; check RTC hardware/clock source"
        );
        let freq_ns_u128 =
            (u128::from(counter_diff) * 1_000_000_000u128) / u128::from(time_diff_ns);
        let freq_ns = u64::try_from(freq_ns_u128).unwrap_or(u64::MAX);
        #[cfg(target_pointer_width = "32")]
        SCALE.calibrate(freq_ns);
        #[cfg(not(target_pointer_width = "32"))]
        FREQUENCY_NS.store(freq_ns, Ordering::Relaxed);
        INIT_COUNTER.store(start_counter, Ordering::Relaxed);
        INIT_TIME_NS.store(start_time, Ordering::Relaxed);
    }
}

/// Translate the high-precision counter-value to real time nanoseconds
pub fn counter_to_nanos(read_raw_counter: fn() -> u64) -> u64 {
    let init_counter = INIT_COUNTER.load(Ordering::Relaxed);
    let init_time_ns = INIT_TIME_NS.load(Ordering::Relaxed);
    let counter = read_raw_counter();
    let counter_diff = counter.saturating_sub(init_counter);

    // NOTE: If `freq == 0`, calibration did not succeed (typically broken RTC path).
    // Panic on division by zero is intentional fail-fast behavior.
    #[cfg(target_pointer_width = "32")]
    let elapsed = SCALE.nanos(counter_diff);
    #[cfg(not(target_pointer_width = "32"))]
    let elapsed = {
        let freq = FREQUENCY_NS.load(Ordering::Relaxed);
        ((u128::from(counter_diff) * 1_000_000_000) / u128::from(freq)) as u64
    };
    init_time_ns.saturating_add(elapsed)
}

#[cfg(test)]
mod tests {
    use super::CounterScale;

    #[test]
    fn reduced_scale_matches_1_2_conversion_across_calibrations() {
        let scale = CounterScale::new();
        for hz in [
            1,
            24_000_000,
            400_000_000,
            480_000_000,
            999_999_937,
            1_000_000_000,
            1 << 60,
            u64::MAX,
            80_000_000,
        ] {
            scale.calibrate(hz);
            for ticks in [
                0,
                1,
                3,
                hz - 1,
                hz,
                u64::MAX / 5,
                u64::MAX / 2,
                u64::MAX - 1,
                u64::MAX,
            ] {
                let expected = (u128::from(ticks) * 1_000_000_000 / u128::from(hz)) as u64;
                assert_eq!(scale.nanos(ticks), expected, "hz={hz} ticks={ticks}");
            }
        }
    }

    #[test]
    #[should_panic]
    fn zero_frequency_still_fails_on_read() {
        let scale = CounterScale::new();
        scale.calibrate(480_000_000);
        scale.calibrate(0);
        scale.nanos(1);
    }
}
