pub fn initialize() {}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    let counter: u64;
    unsafe {
        core::arch::asm!("rdcycle {}", out(reg) counter);
    }
    counter
}
