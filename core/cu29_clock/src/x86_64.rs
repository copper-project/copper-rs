pub fn initialize() {}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    unsafe { core::arch::x86_64::_rdtsc() }
}
