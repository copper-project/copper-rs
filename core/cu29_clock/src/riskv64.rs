pub fn initialize() {}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    let counter: u64;
    // SAFETY: Reading the cycle counter register is a side-effect-free CPU instruction.
    unsafe {
        core::arch::asm!("rdcycle {}", out(reg) counter);
    }
    counter
}
