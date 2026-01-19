// --- Cortex-M (bare-metal)

use cortex_m::peripheral::{DWT, Peripherals};

pub fn initialize() {
    // SAFETY: This runs during early init when we have exclusive access to peripherals.
    let mut cp = unsafe { Peripherals::steal() };
    cp.DCB.enable_trace();
    DWT::unlock();
    cp.DWT.set_cycle_count(0);
    cp.DWT.enable_cycle_counter();
}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    // The cycle counter is 32bit, so we need a way to not overflow.
    // Note: as it is 32bit and in the ns range, it can hold on ~4s between two reads.
    // This accumulator acts as a 64bit counter that is available on the other platforms.
    static mut ACCUMULATOR: u64 = 0;
    // SAFETY: We only use DWT for reading and reinitializing the cycle counter.
    let mut cp = unsafe { Peripherals::steal() };
    let to_add = cp.DWT.cyccnt.read() as u64;
    cp.DWT.set_cycle_count(0);
    // SAFETY: ACCUMULATOR is only accessed from this function on a single core.
    unsafe {
        ACCUMULATOR += to_add;
        ACCUMULATOR
    }
}
