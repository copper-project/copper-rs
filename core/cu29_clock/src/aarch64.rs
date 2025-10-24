pub fn initialize() {}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    let mut counter: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntvct_el0", out(reg) counter);
    }
    counter
}
