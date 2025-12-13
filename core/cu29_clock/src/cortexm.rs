// --- Cortex-M (bare-metal)

use cortex_m::peripheral::{DWT, Peripherals};

pub fn initialize() {
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
    let mut cp = unsafe { Peripherals::steal() };
    let to_add = cp.DWT.cyccnt.read() as u64;
    cp.DWT.set_cycle_count(0);
    unsafe {
        ACCUMULATOR += to_add;
        ACCUMULATOR
    }
}
