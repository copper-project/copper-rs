// Source: x86-64 time-stamp counter (`RDTSC`).
// Guarantee: fast cycle-like counter on Copper's supported x86-64 hosts. This
// assumes an invariant, synchronized TSC; older or unusual systems can violate
// that across cores or power states.
pub fn initialize() {}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    // SAFETY: RDTSC is a side-effect-free instruction on x86_64.
    unsafe { core::arch::x86_64::_rdtsc() }
}
