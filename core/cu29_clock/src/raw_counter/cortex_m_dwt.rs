// Source: Cortex-M DWT cycle counter (`DWT.CYCCNT`).
// A single, non-reentrant Copper reader samples the free-running counter
// less than 2^32 core cycles apart. Other CYCCNT readers can sample independently.

use cortex_m::peripheral::{DWT, Peripherals};

pub fn initialize() {
    // SAFETY: This runs during early init when we have exclusive access to peripherals.
    let mut cp = unsafe { Peripherals::steal() };
    cp.DCB.enable_trace();
    DWT::unlock();
    cp.DWT.enable_cycle_counter();
    read_raw_counter(); // Capture the initial sample even if CYCCNT was already running.
}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    static mut ACCUMULATOR: u64 = 0;
    let now = DWT::cycle_count();
    // SAFETY: The backend assumes a single, non-reentrant Copper reader.
    unsafe {
        let previous = ACCUMULATOR;
        // The low word holds the last sample. Thumb-2 needs only SUBS, ADDS, ADC for the delta and carry.
        let delta = now.wrapping_sub(previous as u32);
        let extended = previous.wrapping_add(u64::from(delta));
        ACCUMULATOR = extended;
        extended
    }
}
