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
