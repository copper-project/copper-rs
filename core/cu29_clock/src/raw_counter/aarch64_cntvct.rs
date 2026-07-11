// Source: ARM generic timer virtual count register (`CNTVCT_EL0`).
// Guarantee: monotonic fixed-frequency counter when firmware/OS enables EL0
// access. The counter is system-level rather than a per-thread wall clock.
pub fn initialize() {}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    let mut counter: u64;
    // SAFETY: Reading the virtual counter register is a side-effect-free CPU instruction.
    unsafe {
        core::arch::asm!("mrs {}, cntvct_el0", out(reg) counter);
    }
    counter
}
