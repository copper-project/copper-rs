pub fn initialize() {}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    // SAFETY: RDTSC is a side-effect-free instruction on x86_64.
    unsafe { core::arch::x86_64::_rdtsc() }
}
